module tdoa_location_led(
    input clk,
    input rst_n,
    input [7:0] tdoa_cnt,       // 无符号时刻差（采样周期，需确保τ范围：0~63为mic1先收，64~127为mic2先收）
    input [31:0] max_corr,      // 互相关最大值（有效性判断）
    output reg [15:0] source_dist,  // 声源相对距离（mm，正→mic1，负→mic2）
    output reg led_left,        // 左侧灯（靠近mic1时亮）
    output reg led_right        // 右侧灯（靠近mic2时亮）
);

// 1. 参数优化：适配无符号tdoa_cnt，降低死区避免小值被过滤
parameter SAMPLE_FREQ = 16'd48000; // 采样频率（48kHz）
parameter SOUND_SPEED = 16'd340;   // 声速（340m/s）
parameter MIC_DIST = 16'd200;      // 两麦克风间距（200mm）
parameter MIN_CORR = 32'd100000;   // 最小有效互相关值（根据实际调试调整）
parameter DEAD_ZONE = 16'd3;       // 死区降至3mm，避免小距离差被过滤
parameter SCALE = 20'd1000000;     // 定点运算缩放因子（1e6）
parameter TDOA_MID = 8'd64;        // tdoa_cnt中点：区分mic1/mic2先收（0~63→mic1，65~127→mic2）

// 2. 中间变量：增加寄存器打拍，优化时序；扩展位宽保留精度
reg [31:0] tdoa_us_reg;       // 时间差寄存器（打拍1次）
reg signed [19:0] dist_diff_reg; // 距离差寄存器（20位，避免溢出）
reg signed [15:0] signed_dist_reg; // 相对距离寄存器（打拍1次）
reg valid_flag;               // 有效信号标志（打拍同步）

// 3. 第一步：计算时间差（扩展位宽+打拍，保留精度）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        tdoa_us_reg <= 32'd0;
        valid_flag <= 1'b0;
    end else begin
        // 计算时间差（μs）：用32位运算保留小数部分，避免截断
        tdoa_us_reg <= (tdoa_cnt * SCALE) / SAMPLE_FREQ;
        // 有效标志打拍，与时间差同步
        valid_flag <= (max_corr > MIN_CORR) ? 1'b1 : 1'b0;
    end
end

// 4. 第二步：计算距离差+相对距离（处理符号+打拍）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        dist_diff_reg <= 20'sd0;
        signed_dist_reg <= 16'sd0;
        source_dist <= 16'd0;
    end else if(valid_flag) begin
        // 处理符号：tdoa_cnt < 64→mic1先收（距离差正）；>64→mic2先收（距离差负）
        if(tdoa_cnt < TDOA_MID) begin
            // mic1先收：距离差 = 声速×时间差（正）
            dist_diff_reg <= (SOUND_SPEED * tdoa_us_reg) / 1000;
        end else begin
            // mic2先收：距离差 = -（声速×(tdoa_cnt-64)）（负）
            dist_diff_reg <= - (SOUND_SPEED * ((tdoa_cnt - TDOA_MID) * SCALE / SAMPLE_FREQ)) / 1000;
        end
        
        // 计算相对距离（相对于中点）：距离差 / 2（算术右移保留符号）
        signed_dist_reg <= dist_diff_reg >>> 1;
        source_dist <= signed_dist_reg[15:0]; // 输出距离
    end else begin
        dist_diff_reg <= 20'sd0;
        signed_dist_reg <= 16'sd0;
        source_dist <= 16'd0;
    end
end

// 5. 第三步：灯控逻辑（基于同步后的相对距离，避免时序问题）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        led_left <= 1'b0;
        led_right <= 1'b0;
    end else if(valid_flag) begin
        if(signed_dist_reg > DEAD_ZONE) begin
            // 靠近mic1：左灯亮，右灯灭
            led_left <= 1'b1;
            led_right <= 1'b0;
        end else if(signed_dist_reg < -DEAD_ZONE) begin
            // 靠近mic2：右灯亮，左灯灭
            led_left <= 1'b0;
            led_right <= 1'b1;
        end else begin
            // 死区（靠近中点）：两灯灭
            led_left <= 1'b0;
            led_right <= 1'b0;
        end
    end else begin
        // 无效信号：两灯灭
        led_left <= 1'b0;
        led_right <= 1'b0;
    end
end

endmodule
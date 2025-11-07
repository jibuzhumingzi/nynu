module spar_4mic_localization (
    input               clk,                // 系统时钟（50MHz）
    input               rst_n,              // 复位（低有效）
    input       [23:0]  mic1_data,          // 麦克风1数据
    input       [23:0]  mic2_data,          // 麦克风2数据
    input       [23:0]  mic3_data,          // 麦克风3数据
    input       [23:0]  mic4_data,          // 麦克风4数据
    input               data_valid,         // 数据有效信号
    // 主LED（单个麦克风）
    output reg          led1,               led2,               led3,               led4,
    // 中间LED（两个麦克风之间）
    output reg          led12,              led23,              led34,              led41
);



// --------------------------
// 1. 参数配置
// --------------------------


parameter MAIN_THRESHOLD_MWDIA = 24'd4500000;          // 8000000
parameter MAIN_THRESHOLD_MWDIA_3_4 = 24'd4200000;

parameter AVG_DEPTH      = 64;             // 64组滑动平均
parameter MAIN_THRESHOLD = 24'd7000000;          // 8000000
parameter MID_THRESHOLD  = 24'd3000000;          // 2000000
parameter MIN_HOLD_CNT   = 600;            // 状态保持时间

// --------------------------
// 2. 信号预处理与平滑
// --------------------------
reg [23:0] mic1_amp, mic2_amp, mic3_amp, mic4_amp;
reg [23:0] mic1_avg, mic2_avg, mic3_avg, mic4_avg;

// 幅度计算（取绝对值）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mic1_amp <= 24'd0;
        mic2_amp <= 24'd0;
        mic3_amp <= 24'd0;
        mic4_amp <= 24'd0;
    end else if (data_valid) begin
        mic1_amp <= mic1_data[23] ? (~mic1_data + 1'b1) : mic1_data;
        mic2_amp <= mic2_data[23] ? (~mic2_data + 1'b1) : mic2_data;
        mic3_amp <= mic3_data[23] ? (~mic3_data + 1'b1) : mic3_data;
        mic4_amp <= mic4_data[23] ? (~mic4_data + 1'b1) : mic4_data;
    end
end

// 64组滑动平均
reg [23:0] mic1_buf [0:AVG_DEPTH-1];
reg [23:0] mic2_buf [0:AVG_DEPTH-1];
reg [23:0] mic3_buf [0:AVG_DEPTH-1];
reg [23:0] mic4_buf [0:AVG_DEPTH-1];
reg [5:0]  avg_ptr;
reg [29:0] mic1_sum, mic2_sum, mic3_sum, mic4_sum;

integer i;
initial begin
    for (i=0; i<AVG_DEPTH; i=i+1) begin
        mic1_buf[i] = 24'd0;
        mic2_buf[i] = 24'd0;
        mic3_buf[i] = 24'd0;
        mic4_buf[i] = 24'd0;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        avg_ptr <= 6'd0;
        mic1_sum <= 30'd0;
        mic2_sum <= 30'd0;
        mic3_sum <= 30'd0;
        mic4_sum <= 30'd0;
    end else if (data_valid) begin
        mic1_sum <= mic1_sum - mic1_buf[avg_ptr] + mic1_amp;
        mic2_sum <= mic2_sum - mic2_buf[avg_ptr] + mic2_amp;
        mic3_sum <= mic3_sum - mic3_buf[avg_ptr] + mic3_amp;
        mic4_sum <= mic4_sum - mic4_buf[avg_ptr] + mic4_amp;
        
        mic1_buf[avg_ptr] <= mic1_amp;
        mic2_buf[avg_ptr] <= mic2_amp;
        mic3_buf[avg_ptr] <= mic3_amp;
        mic4_buf[avg_ptr] <= mic4_amp;
        
        avg_ptr <= avg_ptr + 1'b1;
    end
end

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mic1_avg <= 24'd0;
        mic2_avg <= 24'd0;
        mic3_avg <= 24'd0;
        mic4_avg <= 24'd0;
    end else begin
        mic1_avg <= mic1_sum >> 6;
        mic2_avg <= mic2_sum >> 6;
        mic3_avg <= mic3_sum >> 6;
        mic4_avg <= mic4_sum >> 6;
    end
end

// --------------------------
// 3. 状态判断（主状态+中间状态）
// --------------------------
reg [3:0] main_state;       // 主状态：4'b0001(led1), 4'b0010(led2), 4'b0100(led3), 4'b1000(led4)
reg [3:0] mid_state;        // 中间状态：4'b0001(led12), 4'b0010(led23), 4'b0100(led34), 4'b1000(led41)
reg [9:0] hold_cnt;         // 状态保持计数器

// 状态判断逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        main_state <= 4'b0001;  // 初始主状态（led1）
        mid_state <= 4'b0000;   // 初始无中间状态
        hold_cnt <= 10'd0;
    end else if (data_valid) begin
        hold_cnt <= (hold_cnt < MIN_HOLD_CNT) ? hold_cnt + 1'b1 : MIN_HOLD_CNT;
        
        // 只有状态锁定中间状态的优先级：中间状态优先于主状态（避免同时重叠）
        if (hold_cnt >= MIN_HOLD_CNT) begin
            // 中间状态判断：两麦克风均明显高于另外两个，且彼此差距小
            // 1. mic1与mic2之间
            if (mic1_avg > mic3_avg  + MAIN_THRESHOLD_MWDIA &&
                mic1_avg > mic4_avg  + MAIN_THRESHOLD_MWDIA &&
                mic2_avg > mic3_avg  + MAIN_THRESHOLD_MWDIA &&
                mic2_avg > mic4_avg  + MAIN_THRESHOLD_MWDIA &&
                (mic1_avg > mic2_avg  - MID_THRESHOLD) &&  // 差距<10%
                (mic1_avg < mic2_avg  + MID_THRESHOLD)) begin
                mid_state <= 4'b0001;
                main_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            // 2. mic2与mic3之间
            else if (mic2_avg > mic1_avg + MAIN_THRESHOLD_MWDIA &&
                     mic2_avg > mic4_avg + MAIN_THRESHOLD_MWDIA &&
                     mic3_avg > mic1_avg + MAIN_THRESHOLD_MWDIA &&
                     mic3_avg > mic4_avg + MAIN_THRESHOLD_MWDIA &&
                     (mic2_avg > mic3_avg - MID_THRESHOLD) &&
                     (mic2_avg < mic3_avg +  MID_THRESHOLD)) begin
                mid_state <= 4'b0010;
                main_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            // 3. mic3与mic4之间
            else if (mic3_avg > mic1_avg + MAIN_THRESHOLD_MWDIA_3_4 &&
                     mic3_avg > mic2_avg + MAIN_THRESHOLD_MWDIA_3_4 &&
                     mic4_avg > mic1_avg + MAIN_THRESHOLD_MWDIA_3_4 &&
                     mic4_avg > mic2_avg + MAIN_THRESHOLD_MWDIA_3_4 &&
                     (mic3_avg > mic4_avg - MID_THRESHOLD) &&
                     (mic3_avg < mic4_avg + MID_THRESHOLD)) begin
                mid_state <= 4'b0100;
                main_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            // 4. mic4与mic1之间（环形布局）
            else if (mic4_avg > mic2_avg + MAIN_THRESHOLD_MWDIA &&
                     mic4_avg > mic3_avg + MAIN_THRESHOLD_MWDIA &&
                     mic1_avg > mic2_avg + MAIN_THRESHOLD_MWDIA &&
                     mic1_avg > mic3_avg + MAIN_THRESHOLD_MWDIA &&
                     (mic4_avg > mic1_avg - MID_THRESHOLD) &&
                     (mic4_avg < mic1_avg + MID_THRESHOLD)) begin
                mid_state <= 4'b1000;
                main_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            // 主状态判断：单个麦克风明显高于其他
            else if (mic1_avg > mic2_avg + MAIN_THRESHOLD &&
                     mic1_avg > mic3_avg + MAIN_THRESHOLD &&
                     mic1_avg > mic4_avg + MAIN_THRESHOLD) begin
                main_state <= 4'b0001;
                mid_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            else if (mic2_avg> mic1_avg + MAIN_THRESHOLD &&
                     mic2_avg> mic3_avg + MAIN_THRESHOLD &&
                     mic2_avg> mic4_avg + MAIN_THRESHOLD) begin
                main_state <= 4'b0010;
                mid_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            else if (mic3_avg > mic1_avg + MAIN_THRESHOLD &&
                     mic3_avg > mic2_avg + MAIN_THRESHOLD &&
                     mic3_avg > mic4_avg + MAIN_THRESHOLD) begin
                main_state <= 4'b0100;
                mid_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            else if (mic4_avg > mic1_avg  + MAIN_THRESHOLD &&
                     mic4_avg > mic2_avg  + MAIN_THRESHOLD &&
                     mic4_avg > mic3_avg  + MAIN_THRESHOLD) begin
                main_state <= 4'b1000;
                mid_state <= 4'b0000;
                hold_cnt <= 10'd0;
            end
            // 否则保持当前状态
        end
    end
end

// --------------------------
// 4. LED输出控制
// --------------------------
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        // 初始状态：主LED1亮，其他灭
        led1 <= 1'b1;  led2 <= 1'b0;  led3 <= 1'b0;  led4 <= 1'b0;
        led12 <= 1'b0; led23 <= 1'b0; led34 <= 1'b0; led41 <= 1'b0;
    end else begin
        // 主LED输出
        led1 <= main_state[0];
        led2 <= main_state[1];
        led3 <= main_state[2];
        led4 <= main_state[3];
        // 中间LED输出
        led12 <= mid_state[0];
        led23 <= mid_state[1];
        led34 <= mid_state[2];
        led41 <= mid_state[3];
    end
end

endmodule
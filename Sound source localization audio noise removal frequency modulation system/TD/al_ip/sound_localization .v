module dual_mic_code_localization (
    input clk,                  
    input rst_n,                
    input [31:0] adc_left,      
    input [31:0] adc_right,     
    input adc_valid,            
    output reg led_left,        
    output reg led_mid,         
    output reg led_right        
);

// 参数定义
parameter DATA_WIDTH = 32;         
parameter FRAME_LEN = 64;          // 减少帧长度提高响应速度
parameter NOISE_THRESHOLD = 32'd10;  // 合理噪声阈值
parameter CODE_DIM = 4;            // 增加编码维度到4维
parameter COMPARE_THRESH = 32'd10; // 比较器阈值

// 信号定义
reg [DATA_WIDTH-1:0] left_buf[0:FRAME_LEN-1];
reg [DATA_WIDTH-1:0] right_buf[0:FRAME_LEN-1];
reg [5:0] sample_idx;
reg frame_ready;

// ==================== 特征编码器 ====================
// 左麦克风特征编码
reg [31:0] left_features[0:CODE_DIM-1];
// 右麦克风特征编码  
reg [31:0] right_features[0:CODE_DIM-1];

// 特征编码器计算
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sample_idx <= 6'd0;
        frame_ready <= 1'b0;
        for (integer i = 0; i < CODE_DIM; i = i + 1) begin
            left_features[i] <= 32'd0;
            right_features[i] <= 32'd0;
        end
    end else if (adc_valid) begin
        // 数据缓冲
        left_buf[sample_idx] <= (adc_left[31]) ? (~adc_left + 1'b1) : adc_left;
        right_buf[sample_idx] <= (adc_right[31]) ? (~adc_right + 1'b1) : adc_right;
        
        if (sample_idx == FRAME_LEN-1) begin
            sample_idx <= 6'd0;
            frame_ready <= 1'b1;
        end else begin
            sample_idx <= sample_idx + 6'd1;
            frame_ready <= 1'b0;
        end
        
        // 实时特征提取（编码器核心）
        if (frame_ready) begin
            // 重置特征值
            for (integer i = 0; i < CODE_DIM; i = i + 1) begin
                left_features[i] <= 32'd0;
                right_features[i] <= 32'd0;
            end
            
            // 计算帧特征
            for (integer i = 0; i < FRAME_LEN; i = i + 1) begin
                // 特征0: 能量累积
                left_features[0] <= left_features[0] + left_buf[i];
                right_features[0] <= right_features[0] + right_buf[i];
                
                // 特征1: 过阈值计数（高频成分）
                if (left_buf[i] > NOISE_THRESHOLD * 2) 
                    left_features[1] <= left_features[1] + 32'd1;
                if (right_buf[i] > NOISE_THRESHOLD * 2)
                    right_features[1] <= right_features[1] + 32'd1;
                    
                // 特征2: 峰值检测
                if (i > 0 && i < FRAME_LEN-1) begin
                    if (left_buf[i] > left_buf[i-1] && left_buf[i] > left_buf[i+1])
                        left_features[2] <= left_features[2] + 32'd1;
                    if (right_buf[i] > right_buf[i-1] && right_buf[i] > right_buf[i+1])
                        right_features[2] <= right_features[2] + 32'd1;
                end
                
                // 特征3: 斜率变化（信号活跃度）
                if (i > 0) begin
                    if (left_buf[i] > left_buf[i-1] + NOISE_THRESHOLD)
                        left_features[3] <= left_features[3] + 32'd1;
                    if (right_buf[i] > right_buf[i-1] + NOISE_THRESHOLD)
                        right_features[3] <= right_features[3] + 32'd1;
                end
            end
        end
    end
end

// ==================== 数据比较器 ====================
reg [2:0] vote_result;  // 投票结果 [左,中,右]
reg [31:0] feature_diff[0:CODE_DIM-1];

// 多维度特征比较器
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        vote_result <= 3'b000;
        led_left <= 1'b0;
        led_mid <= 1'b0;
        led_right <= 1'b0;
        for (integer i = 0; i < CODE_DIM; i = i + 1)
            feature_diff[i] <= 32'd0;
    end else if (frame_ready) begin
        // 重置投票
        vote_result <= 3'b000;
        
        // 各维度特征比较
        for (integer i = 0; i < CODE_DIM; i = i + 1) begin
            feature_diff[i] <= left_features[i] - right_features[i];
            
            // 比较器决策
            if (feature_diff[i] > COMPARE_THRESH) begin
                vote_result[2] <= vote_result[2] + 1'b1;  // 左声道强
            end else if (feature_diff[i] < -COMPARE_THRESH) begin
                vote_result[0] <= vote_result[0] + 1'b1;  // 右声道强  
            end else begin
                vote_result[1] <= vote_result[1] + 1'b1;  // 中间
            end
        end
        
        // 最终决策：多数投票
        if (vote_result[2] > vote_result[1] && vote_result[2] > vote_result[0]) begin
            led_left <= 1'b1;
            led_mid <= 1'b0;
            led_right <= 1'b0;
        end else if (vote_result[0] > vote_result[1] && vote_result[0] > vote_result[2]) begin
            led_left <= 1'b0;
            led_mid <= 1'b0;
            led_right <= 1'b1;
        end else begin
            led_left <= 1'b0;
            led_mid <= 1'b1;
            led_right <= 1'b0;
        end
    end else begin
        // 保持LED状态一段时间以便观察
        // 可选：添加定时器自动清除LED状态
    end
end

// ==================== 调试输出 ====================
// 可选：添加调试信号观察内部状态
/*
output wire [31:0] debug_left_energy,
output wire [31:0] debug_right_energy,
output wire [2:0] debug_vote,

assign debug_left_energy = left_features[0];
assign debug_right_energy = right_features[0];
assign debug_vote = vote_result;
*/

endmodule
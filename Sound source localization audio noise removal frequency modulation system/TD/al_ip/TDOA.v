module sound_source_localization #(
    parameter AUDIO_WIDTH   = 16,    // 音频数据位宽
    parameter FFT_SIZE      = 256,   // FFT点数
    parameter FFT_OUT_WIDTH = 24,    // FFT输出位宽(16+8)
    parameter FREQ_BINS     = 129    // 有效频率点数(FFT_SIZE/2+1)
)(
    input                       clk,
    input                       rst_n,
    input  [AUDIO_WIDTH-1:0]    mic_left,      // 左麦克风数据
    input  [AUDIO_WIDTH-1:0]    mic_right,     // 右麦克风数据  
    input                       audio_valid,   // 音频数据有效
    output reg [2:0]            direction,     // 声源方向 [左,中,右]
    output 		                loc_valid,     // 定位结果有效
    output [15:0]               debug_tdoa     // 调试：TDOA估计
);

// 内部信号定义
wire [FFT_OUT_WIDTH-1:0] fft_left_re, fft_left_im;
wire [FFT_OUT_WIDTH-1:0] fft_right_re, fft_right_im;
wire fft_left_valid, fft_right_valid;

// 1. FFT处理模块
fft_256 #(
    .DATA_WIDTH(AUDIO_WIDTH),
    .TWID_WIDTH(16),
    .FFT_SIZE(FFT_SIZE),
    .STAGES(8),
    .ADDR_WIDTH(8)
) fft_left (
    .clk        (clk),
    .rst_n      (rst_n),
    .din_re     (mic_left),
    .din_im     (16'h0000),  // 实信号，虚部为0
    .din_valid  (audio_valid),
    .dout_re    (fft_left_re),
    .dout_im    (fft_left_im),
    .dout_valid (fft_left_valid)
);

fft_256 #(
    .DATA_WIDTH(AUDIO_WIDTH),
    .TWID_WIDTH(16), 
    .FFT_SIZE(FFT_SIZE),
    .STAGES(8),
    .ADDR_WIDTH(8)
) fft_right (
    .clk        (clk),
    .rst_n      (rst_n),
    .din_re     (mic_right),
    .din_im     (16'h0000),  // 实信号，虚部为0
    .din_valid  (audio_valid),
    .dout_re    (fft_right_re),
    .dout_im    (fft_right_im),
    .dout_valid (fft_right_valid)
);

// 2. GCC-PHAT计算模块
gcc_phat #(
    .FFT_OUT_WIDTH(FFT_OUT_WIDTH),
    .FREQ_BINS(FREQ_BINS)
) u_gcc_phat (
    .clk            (clk),
    .rst_n          (rst_n),
    .fft_left_re    (fft_left_re),
    .fft_left_im    (fft_left_im),
    .fft_right_re   (fft_right_re), 
    .fft_right_im   (fft_right_im),
    .fft_valid      (fft_left_valid & fft_right_valid),
    .tdoa_estimate  (debug_tdoa),
    .gcc_valid      (loc_valid)
);

// 3. 方向判断逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        direction <= 3'b000;
    end else if (loc_valid) begin
        // 根据TDOA判断方向
        if (debug_tdoa[15] == 1'b1) begin  // 负数，左侧声源
            direction <= 3'b100;
        end else if (debug_tdoa > 16'd10) begin  // 正数且较大，右侧声源
            direction <= 3'b001;
        end else if (debug_tdoa > 16'd2) begin  // 小正数，轻微偏右
            direction <= 3'b011;
        end else if (debug_tdoa < -16'd10) begin  // 大负数，左侧声源
            direction <= 3'b100;
        end else if (debug_tdoa < -16'd2) begin  // 小负数，轻微偏左
            direction <= 3'b110;
        end else begin  // 接近零，中间声源
            direction <= 3'b010;
        end
    end
end

endmodule
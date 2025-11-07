module gcc_phat #(
    parameter FFT_OUT_WIDTH = 24,
    parameter FREQ_BINS = 129
)(
    input                       clk,
    input                       rst_n,
    input  [FFT_OUT_WIDTH-1:0]  fft_left_re,
    input  [FFT_OUT_WIDTH-1:0]  fft_left_im,
    input  [FFT_OUT_WIDTH-1:0]  fft_right_re,
    input  [FFT_OUT_WIDTH-1:0]  fft_right_im,
    input                       fft_valid,
    output reg [15:0]           tdoa_estimate,
    output reg                  gcc_valid
);

// 内部信号声明（全部在模块开始处声明）
reg [FFT_OUT_WIDTH-1:0] left_re_buf [0:FREQ_BINS-1];
reg [FFT_OUT_WIDTH-1:0] left_im_buf [0:FREQ_BINS-1];
reg [FFT_OUT_WIDTH-1:0] right_re_buf [0:FREQ_BINS-1];
reg [FFT_OUT_WIDTH-1:0] right_im_buf [0:FREQ_BINS-1];
reg [8:0] freq_index;
reg buf_full;

// 互功率谱
reg [FFT_OUT_WIDTH*2-1:0] cross_re [0:FREQ_BINS-1];
reg [FFT_OUT_WIDTH*2-1:0] cross_im [0:FREQ_BINS-1];
reg [FFT_OUT_WIDTH*2-1:0] mag [0:FREQ_BINS-1];

// GCC相关
reg [31:0] gcc_corr [0:63];
reg [5:0] delay_idx;
reg [31:0] max_corr_val;
reg [5:0] best_dly;

// GCC计算专用寄存器
reg [31:0] corr_acc;
reg [8:0] f_idx;
reg [15:0] cos_val, sin_val;
reg [31:0] weighted_re, weighted_im;
reg [7:0] phase_idx;

// 状态机
reg [3:0] state;
localparam IDLE = 4'd0;
localparam BUF_FFT = 4'd1;
localparam CALC_CROSS = 4'd2;
localparam CALC_GCC_D = 4'd3;
localparam CALC_GCC_F = 4'd4;
localparam CALC_GCC_ACC = 4'd5;
localparam FIND_PEAK = 4'd6;
localparam OUTPUT = 4'd7;

// 预计算的cos/sin ROM
reg [15:0] cos_rom [0:127];
reg [15:0] sin_rom [0:127];

// 初始化三角函数ROM
integer k;
initial begin
    for (k = 0; k < 128; k = k + 1) begin
        // 简化的cos/sin值（Q1.15格式）
        case (k % 8)
            0: begin cos_rom[k] = 16'h7FFF; sin_rom[k] = 16'h0000; end // 1.0, 0.0
            1: begin cos_rom[k] = 16'h7641; sin_rom[k] = 16'hCF04; end // 0.92, -0.38
            2: begin cos_rom[k] = 16'h5A82; sin_rom[k] = 16'hA57D; end // 0.71, -0.71
            3: begin cos_rom[k] = 16'h30FB; sin_rom[k] = 16'h89BE; end // 0.38, -0.92
            4: begin cos_rom[k] = 16'h0000; sin_rom[k] = 16'h8000; end // 0.0, -1.0
            5: begin cos_rom[k] = 16'hCF04; sin_rom[k] = 16'h89BE; end // -0.38, -0.92
            6: begin cos_rom[k] = 16'hA57D; sin_rom[k] = 16'hA57D; end // -0.71, -0.71
            7: begin cos_rom[k] = 16'h89BE; sin_rom[k] = 16'hCF04; end // -0.92, -0.38
        endcase
    end
end

// 主状态机
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        freq_index <= 0;
        state <= IDLE;
        gcc_valid <= 0;
        tdoa_estimate <= 0;
        delay_idx <= 0;
        f_idx <= 0;
        corr_acc <= 0;
    end else begin
        case (state)
            IDLE: begin
                if (fft_valid) begin
                    state <= BUF_FFT;
                    freq_index <= 0;
                end
                gcc_valid <= 0;
            end
            
            BUF_FFT: begin
                if (freq_index < FREQ_BINS) begin
                    left_re_buf[freq_index] <= fft_left_re;
                    left_im_buf[freq_index] <= fft_left_im;
                    right_re_buf[freq_index] <= fft_right_re;
                    right_im_buf[freq_index] <= fft_right_im;
                    freq_index <= freq_index + 1;
                end else begin
                    state <= CALC_CROSS;
                    freq_index <= 1;  // 跳过DC分量
                end
            end
            
            CALC_CROSS: begin
                if (freq_index < FREQ_BINS) begin
                    // 互功率谱: X_L * conj(X_R)
                    cross_re[freq_index] <= 
                        (fft_left_re * fft_right_re) + (fft_left_im * fft_right_im);
                    cross_im[freq_index] <= 
                        (fft_left_im * fft_right_re) - (fft_left_re * fft_right_im);
                    
                    // 幅度计算
                    mag[freq_index] <= 
                        (fft_left_re * fft_left_re + fft_left_im * fft_left_im +
                         fft_right_re * fft_right_re + fft_right_im * fft_right_im) / 2 + 1;
                    
                    freq_index <= freq_index + 1;
                end else begin
                    state <= CALC_GCC_D;
                    delay_idx <= 0;
                    max_corr_val <= 0;
                    best_dly <= 32;
                end
            end
            
            CALC_GCC_D: begin
                if (delay_idx < 64) begin
                    corr_acc <= 0;
                    f_idx <= 1;  // 从1开始，跳过DC
                    state <= CALC_GCC_F;
                end else begin
                    state <= FIND_PEAK;
                    delay_idx <= 0;
                end
            end
            
            CALC_GCC_F: begin
                if (f_idx < FREQ_BINS) begin
                    // 计算相位索引
                    phase_idx = (f_idx * (delay_idx - 32)) / 8;
                    
                    // 从ROM读取cos/sin值
                    cos_val <= cos_rom[phase_idx % 128];
                    sin_val <= sin_rom[phase_idx % 128];
                    
                    state <= CALC_GCC_ACC;
                end else begin
                    // 完成当前延迟的所有频率计算
                    gcc_corr[delay_idx] <= corr_acc;
                    delay_idx <= delay_idx + 1;
                    state <= CALC_GCC_D;
                end
            end
            
            CALC_GCC_ACC: begin
                // PHAT加权计算
                weighted_re = cross_re[f_idx] / mag[f_idx];
                weighted_im = cross_im[f_idx] / mag[f_idx];
                
                // 累加相关值
                corr_acc <= corr_acc + (weighted_re * cos_val - weighted_im * sin_val);
                
                f_idx <= f_idx + 1;
                state <= CALC_GCC_F;
            end
            
            FIND_PEAK: begin
                if (delay_idx < 64) begin
                    if (gcc_corr[delay_idx] > max_corr_val) begin
                        max_corr_val <= gcc_corr[delay_idx];
                        best_dly <= delay_idx;
                    end
                    delay_idx <= delay_idx + 1;
                end else begin
                    state <= OUTPUT;
                end
            end
            
            OUTPUT: begin
                tdoa_estimate <= {10'b0, best_dly} - 16'd32;
                gcc_valid <= 1'b1;
                state <= IDLE;
            end
            
            default: state <= IDLE;
        endcase
    end
end

endmodule
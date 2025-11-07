// 256点FFT顶层模块（基-2 DIT，兼容Verilog-2001）
module fft_256 #(
    parameter DATA_WIDTH    = 16,    // 输入数据位宽（含符号位）
    parameter TWID_WIDTH    = 16,    // 旋转因子位宽（Q1.15）
    parameter FFT_SIZE      = 256,   // FFT点数
    parameter STAGES        = 8,     // 蝶形级数（log2(256)）
    parameter ADDR_WIDTH    = 8      // 地址位宽
) (
    input                       clk,         // 工作时钟
    input                       rst_n,       // 异步复位（低有效）
    input  [DATA_WIDTH-1:0]     din_re,      // 输入数据实部
    input  [DATA_WIDTH-1:0]     din_im,      // 输入数据虚部
    input                       din_valid,   // 输入有效信号
    output [DATA_WIDTH+STAGES-1:0] dout_re,  // 输出实部（扩展防溢出）
    output [DATA_WIDTH+STAGES-1:0] dout_im,  // 输出虚部
    output                      dout_valid   // 输出有效信号
);

// 内部信号
wire [DATA_WIDTH-1:0]         br_dout_re, br_dout_im;
wire                          br_valid;
wire [DATA_WIDTH+STAGES-1:0]  stage_re[0:STAGES-1];
wire [DATA_WIDTH+STAGES-1:0]  stage_im[0:STAGES-1];
wire [STAGES-1:0]             stage_valid;

// 为每个需要旋转因子的蝶形级创建独立的地址信号
wire [ADDR_WIDTH-2:0]         twid_addr_stage1;  // 第1级地址
wire [ADDR_WIDTH-2:0]         twid_addr_stage[2:STAGES];  // 第2~8级地址

wire [TWID_WIDTH-1:0]         twid_re, twid_im;

// 1. 位反转模块
bit_reverse #(
    .DATA_WIDTH(DATA_WIDTH),
    .ADDR_WIDTH(ADDR_WIDTH)
) u_bit_reverse (
    .clk        (clk),
    .rst_n      (rst_n),
    .din_re     (din_re),
    .din_im     (din_im),
    .din_valid  (din_valid),
    .dout_re    (br_dout_re),
    .dout_im    (br_dout_im),
    .dout_valid (br_valid)
);

// 2. 旋转因子ROM（实部和虚部）
// 注意：这里需要决定使用哪个级的地址，或者创建多个ROM实例
// 方案A：使用第2级的地址（如果各级地址计算逻辑相同）
rom_128x16 u_twid_re_rom (
    .clk    (clk),
    .addr   (twid_addr_stage[2]),  // 使用第2级地址
    .dout   (twid_re)
);

rom_128x16 u_twid_im_rom (
    .clk    (clk),
    .addr   (twid_addr_stage[2]),  // 使用第2级地址
    .dout   (twid_im)
);

// 3. 第1级蝶形（无旋转因子）
butterfly #(
    .DATA_WIDTH   (DATA_WIDTH),
    .TWID_WIDTH   (TWID_WIDTH),
    .STAGE        (1),
    .FFT_SIZE     (FFT_SIZE)
) u_stage1 (
    .clk          (clk),
    .rst_n        (rst_n),
    .din_re       (br_dout_re),
    .din_im       (br_dout_im),
    .din_valid    (br_valid),
    .twid_re      (16'h7FFF),  // cos(0) = 1.0（Q1.15）
    .twid_im      (16'h0000),  // sin(0) = 0
    .twid_addr    (twid_addr_stage1),  // 使用独立的地址信号
    .dout_re      (stage_re[0]),
    .dout_im      (stage_im[0]),
    .dout_valid   (stage_valid[0])
);

// 4. 第2~8级蝶形（带旋转因子）
generate
    genvar s;
    for (s = 2; s <= STAGES; s = s + 1) begin : stage_gen
        butterfly #(
            .DATA_WIDTH   (DATA_WIDTH),
            .TWID_WIDTH   (TWID_WIDTH),
            .STAGE        (s),
            .FFT_SIZE     (FFT_SIZE)
        ) u_stage (
            .clk          (clk),
            .rst_n        (rst_n),
            .din_re       (stage_re[s-2]),
            .din_im       (stage_im[s-2]),
            .din_valid    (stage_valid[s-2]),
            .twid_re      (twid_re),
            .twid_im      (twid_im),
            .twid_addr    (twid_addr_stage[s]),  // 使用独立的地址信号
            .dout_re      (stage_re[s-1]),
            .dout_im      (stage_im[s-1]),
            .dout_valid   (stage_valid[s-1])
        );
    end
endgenerate

// 5. 输出赋值
assign dout_re    = stage_re[STAGES-1];
assign dout_im    = stage_im[STAGES-1];
assign dout_valid = stage_valid[STAGES-1];

endmodule
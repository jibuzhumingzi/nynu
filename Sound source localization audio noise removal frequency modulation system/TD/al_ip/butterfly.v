// 蝶形运算单元（带流水线寄存器，支持多级级联）
module butterfly #(
    parameter DATA_WIDTH = 16,
    parameter TWID_WIDTH = 16,
    parameter STAGE      = 1,
    parameter FFT_SIZE   = 256
) (
    input                       clk,
    input                       rst_n,
    input  [DATA_WIDTH-1:0]     din_re,
    input  [DATA_WIDTH-1:0]     din_im,
    input                       din_valid,
    input  [TWID_WIDTH-1:0]     twid_re,
    input  [TWID_WIDTH-1:0]     twid_im,
    output reg [ADDR_WIDTH-2:0] twid_addr,  // 旋转因子地址输出
    output reg [DATA_WIDTH+STAGES-1:0] dout_re,
    output reg [DATA_WIDTH+STAGES-1:0] dout_im,
    output reg                  dout_valid
);

localparam GROUP_SIZE  = 1 << STAGE;         // 本级分组大小
localparam TWID_STEP   = FFT_SIZE / GROUP_SIZE;  // 旋转因子步长
localparam ADDR_WIDTH  = $clog2(FFT_SIZE);   // 地址位宽
localparam STAGES      = $clog2(FFT_SIZE);   // 总级数

// 内部寄存器（切割关键路径）
reg [DATA_WIDTH-1:0]         din_re_d1, din_im_d1;
reg                          din_valid_d1;
reg [TWID_WIDTH-1:0]         twid_re_d1, twid_im_d1;
reg [DATA_WIDTH-1:0]         a_re, a_im, b_re, b_im;
reg [DATA_WIDTH+STAGES-1:0]  x_re, x_im, y_re, y_im;
reg [ADDR_WIDTH-1:0]         cnt;            // 数据计数器

// 计算旋转因子地址（时序逻辑赋值，解决非网表错误）
wire [ADDR_WIDTH-1:0] group_idx = cnt / (GROUP_SIZE/2);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        twid_addr <= 0;
    end else begin
        twid_addr <= group_idx * TWID_STEP;  // 寄存器赋值，兼容Verilog
    end
end

// 输入寄存（第一级流水线）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        din_re_d1    <= 0;
        din_im_d1    <= 0;
        din_valid_d1 <= 0;
        cnt          <= 0;
    end else begin
        if (din_valid) begin
            din_re_d1    <= din_re;
            din_im_d1    <= din_im;
            din_valid_d1 <= 1'b1;
            cnt          <= cnt + 1'b1;
        end else begin
            din_valid_d1 <= 1'b0;
        end
    end
end

// 存储A/B数据对（第二级流水线）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        a_re <= 0; a_im <= 0;
        b_re <= 0; b_im <= 0;
        twid_re_d1 <= 0;
        twid_im_d1 <= 0;
    end else begin
        if (din_valid_d1) begin
            // 交替存储A（偶数）和B（奇数）
            if (cnt % 2 == 1'b0) begin  // 偶数：A
                a_re <= din_re_d1;
                a_im <= din_im_d1;
            end else begin  // 奇数：B，锁存旋转因子
                b_re <= din_re_d1;
                b_im <= din_im_d1;
                twid_re_d1 <= twid_re;
                twid_im_d1 <= twid_im;
            end
        end
    end
end

// 复数乘法与加减（第三级流水线）
wire signed [DATA_WIDTH-1:0]        a_re_s = a_re;
wire signed [DATA_WIDTH-1:0]        a_im_s = a_im;
wire signed [DATA_WIDTH-1:0]        b_re_s = b_re;
wire signed [DATA_WIDTH-1:0]        b_im_s = b_im;
wire signed [TWID_WIDTH-1:0]        w_re_s = twid_re_d1;
wire signed [TWID_WIDTH-1:0]        w_im_s = twid_im_d1;
wire signed [DATA_WIDTH+TWID_WIDTH-1:0] b_mul_re = b_re_s * w_re_s - b_im_s * w_im_s;
wire signed [DATA_WIDTH+TWID_WIDTH-1:0] b_mul_im = b_re_s * w_im_s + b_im_s * w_re_s;

// 蝶形运算：X = A + B*W，Y = A - B*W（右移15位还原Q1.15）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        x_re <= 0; x_im <= 0;
        y_re <= 0; y_im <= 0;
    end else begin
        if (din_valid_d1 && cnt % 2 == 1'b1) begin  // B数据有效时计算
            x_re <= a_re_s + {{STAGES-1{1'b0}}, b_mul_re[DATA_WIDTH+TWID_WIDTH-1:TWID_WIDTH-1]};
            x_im <= a_im_s + {{STAGES-1{1'b0}}, b_mul_im[DATA_WIDTH+TWID_WIDTH-1:TWID_WIDTH-1]};
            y_re <= a_re_s - {{STAGES-1{1'b0}}, b_mul_re[DATA_WIDTH+TWID_WIDTH-1:TWID_WIDTH-1]};
            y_im <= a_im_s - {{STAGES-1{1'b0}}, b_mul_im[DATA_WIDTH+TWID_WIDTH-1:TWID_WIDTH-1]};
        end
    end
end

// 输出选择（交替输出X和Y）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        dout_re    <= 0;
        dout_im    <= 0;
        dout_valid <= 0;
    end else begin
        if (din_valid_d1) begin
            dout_valid <= 1'b1;
            if (cnt % 2 == 1'b0) begin  // 输出X（A的结果）
                dout_re <= x_re;
                dout_im <= x_im;
            end else begin  // 输出Y（B的结果）
                dout_re <= y_re;
                dout_im <= y_im;
            end
        end else begin
            dout_valid <= 1'b0;
        end
    end
end

endmodule
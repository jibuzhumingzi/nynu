// ============================================================
// Module : gaussian_filter
// Description : 3×3 高斯滤波器（整数近似）
//   核：[1 2 1; 2 4 2; 1 2 1] / 16
//   使用行缓存（2 个 Line Buffer）实现流水线
// ============================================================
module gaussian_filter #(
    parameter IMG_W = 640,
    parameter IMG_H = 480
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire        i_valid,
    input  wire [7:0]  i_pixel,
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,

    output reg         o_valid,
    output reg  [7:0]  o_pixel,
    output reg  [10:0] o_x,
    output reg  [9:0]  o_y
);

// ---- 行缓存（2 行，每行 640 字节）----
reg [7:0] line_buf0 [0:IMG_W-1];
reg [7:0] line_buf1 [0:IMG_W-1];

// 3 行窗口寄存器
reg [7:0] row0_px, row1_px, row2_px;     // 当前列
reg [7:0] r0_prev, r1_prev, r2_prev;     // 前一列（x-1）
reg [7:0] r0_pp,   r1_pp,   r2_pp;      // 前两列（x-2）

reg [10:0] wr_x;
reg [9:0]  wr_y;
reg        wr_en;

// ---- 行缓存写入 ----
always @(posedge clk) begin
    if (i_valid) begin
        wr_x  <= i_x;
        wr_y  <= i_y;
        wr_en <= 1'b1;
        case (i_y[0])   // 奇偶行交替写入
            1'b0: line_buf0[i_x] <= i_pixel;
            1'b1: line_buf1[i_x] <= i_pixel;
        endcase
    end else begin
        wr_en <= 1'b0;
    end
end

// ---- 读取 3 行数据（当前行+上两行）----
reg [7:0] rd0, rd1, rd2;
reg       rd_valid;
reg [10:0] rd_x;
reg [9:0]  rd_y;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        rd_valid <= 1'b0;
    end else begin
        rd_valid <= i_valid;
        rd_x     <= i_x;
        rd_y     <= i_y;
        // 当前行直接用输入
        rd2 <= i_pixel;
        // 上一行：从对面 buf 读
        case (i_y[0])
            1'b0: rd1 <= line_buf1[i_x];  // 当前是偶行，上行是奇行buf1
            1'b1: rd1 <= line_buf0[i_x];
        endcase
        // 上上行：从同奇偶 buf 读（两行前的数据）
        case (i_y[0])
            1'b0: rd0 <= line_buf0[i_x];
            1'b1: rd0 <= line_buf1[i_x];
        endcase
    end
end

// ---- 3×3 窗口移位寄存器 ----
// 每拍把 rd0/rd1/rd2 推入列方向移位
reg [7:0] w[0:2][0:2];  // w[行][列]，列0=最旧
integer i, j;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 3; i = i+1)
            for (j = 0; j < 3; j = j+1)
                w[i][j] <= 8'd0;
        o_valid <= 1'b0;
    end else if (rd_valid) begin
        // 列方向移位
        w[0][0] <= w[0][1]; w[0][1] <= w[0][2]; w[0][2] <= rd0;
        w[1][0] <= w[1][1]; w[1][1] <= w[1][2]; w[1][2] <= rd1;
        w[2][0] <= w[2][1]; w[2][1] <= w[2][2]; w[2][2] <= rd2;

        // 高斯卷积（核 [1 2 1; 2 4 2; 1 2 1] / 16）
        o_pixel <= (
            w[0][0] + 2*w[0][1] +   w[0][2] +
          2*w[1][0] + 4*w[1][1] + 2*w[1][2] +
            w[2][0] + 2*w[2][1] +   w[2][2]
        ) >> 4;

        // 有效区域：行≥2，列≥2（窗口填满后）
        o_valid <= (rd_x >= 2) && (rd_y >= 2);
        o_x     <= rd_x - 11'd2;
        o_y     <= rd_y - 10'd2;
    end else begin
        o_valid <= 1'b0;
    end
end

endmodule

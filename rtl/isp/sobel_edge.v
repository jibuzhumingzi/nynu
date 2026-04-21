// ============================================================
// Module : sobel_edge
// Description : Sobel 边缘检测
//   Gx = [-1 0 1; -2 0 2; -1 0 1]
//   Gy = [-1 -2 -1; 0 0 0; 1 2 1]
//   G  = |Gx| + |Gy|（近似欧氏距离）
//   输出：edge_valid=1 表示边缘像素
// ============================================================
module sobel_edge #(
    parameter IMG_W   = 640,
    parameter THRESHOLD = 50    // 边缘强度阈值
)(
    input  wire        clk,
    input  wire        rst_n,

    input  wire [7:0]  cfg_threshold,  // 运行时可调阈值

    // 输入灰度流（已滤波）
    input  wire        i_valid,
    input  wire [7:0]  i_pixel,
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,

    // 输出
    output reg         o_valid,
    output reg         o_edge,     // 1=边缘
    output reg  [7:0]  o_grad,     // 梯度强度
    output reg  [10:0] o_x,
    output reg  [9:0]  o_y
);

// ---- 两行 Line Buffer ----
reg [7:0] line_buf0 [0:IMG_W-1];
reg [7:0] line_buf1 [0:IMG_W-1];

// ---- 读缓存延迟流水 ----
// Stage 1：写入缓存 + 读出上两行
reg        s1_valid;
reg [10:0] s1_x;
reg [9:0]  s1_y;
reg [7:0]  s1_r0, s1_r1, s1_r2;

always @(posedge clk) begin
    if (i_valid) begin
        // 写当前行到对应 buf
        case (i_y[0])
            1'b0: line_buf0[i_x] <= i_pixel;
            1'b1: line_buf1[i_x] <= i_pixel;
        endcase

        s1_valid <= 1'b1;
        s1_x     <= i_x;
        s1_y     <= i_y;
        s1_r2    <= i_pixel;
        case (i_y[0])
            1'b0: s1_r1 <= line_buf1[i_x];   // 上一行
            1'b1: s1_r1 <= line_buf0[i_x];
        endcase
        case (i_y[0])
            1'b0: s1_r0 <= line_buf0[i_x];   // 上上行（同奇偶）
            1'b1: s1_r0 <= line_buf1[i_x];
        endcase
    end else begin
        s1_valid <= 1'b0;
    end
end

// ---- 3×3 窗口移位（列方向）----
reg [7:0]  w[0:2][0:2];
reg        s2_valid;
reg [10:0] s2_x;
reg [9:0]  s2_y;
integer i, j;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        for (i = 0; i < 3; i = i+1)
            for (j = 0; j < 3; j = j+1)
                w[i][j] <= 8'd0;
        s2_valid <= 1'b0;
    end else if (s1_valid) begin
        w[0][0] <= w[0][1]; w[0][1] <= w[0][2]; w[0][2] <= s1_r0;
        w[1][0] <= w[1][1]; w[1][1] <= w[1][2]; w[1][2] <= s1_r1;
        w[2][0] <= w[2][1]; w[2][1] <= w[2][2]; w[2][2] <= s1_r2;
        s2_valid <= (s1_x >= 2) && (s1_y >= 2);
        s2_x     <= s1_x - 11'd2;
        s2_y     <= s1_y - 10'd2;
    end else begin
        s2_valid <= 1'b0;
    end
end

// ---- Sobel 卷积 ----
reg signed [11:0] Gx, Gy;
reg [11:0]        Gabs;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_valid <= 1'b0;
        o_edge  <= 1'b0;
        o_grad  <= 8'd0;
        o_x     <= 11'd0;
        o_y     <= 10'd0;
    end else if (s2_valid) begin
        Gx = (-$signed({4'b0,w[0][0]}) + $signed({4'b0,w[0][2]})
              - 2*$signed({4'b0,w[1][0]}) + 2*$signed({4'b0,w[1][2]})
              - $signed({4'b0,w[2][0]}) + $signed({4'b0,w[2][2]}));
        Gy = (-$signed({4'b0,w[0][0]}) - 2*$signed({4'b0,w[0][1]}) - $signed({4'b0,w[0][2]})
              + $signed({4'b0,w[2][0]}) + 2*$signed({4'b0,w[2][1]}) + $signed({4'b0,w[2][2]}));
        Gabs = (Gx < 0 ? -Gx : Gx) + (Gy < 0 ? -Gy : Gy);
        o_grad  <= Gabs[9:2];  // 饱和到 8-bit
        o_edge  <= (Gabs[9:2] > cfg_threshold);
        o_valid <= 1'b1;
        o_x     <= s2_x;
        o_y     <= s2_y;
    end else begin
        o_valid <= 1'b0;
    end
end

endmodule

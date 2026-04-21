// ============================================================
// Module : rgb2gray
// Description : RGB565 → 8-bit 灰度（整数近似）
//   Gray = (R*299 + G*587 + B*114) / 1000
//   快速近似：Gray = (R*76 + G*150 + B*29) >> 8
// ============================================================
module rgb2gray (
    input  wire        clk,
    input  wire        rst_n,

    // 输入 RGB565 像素流
    input  wire        i_valid,
    input  wire [15:0] i_pixel,   // RGB565: R[15:11] G[10:5] B[4:0]
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,

    // 输出灰度像素流
    output reg         o_valid,
    output reg  [7:0]  o_gray,
    output reg  [10:0] o_x,
    output reg  [9:0]  o_y
);

// 拆分 RGB565 → R8 G8 B8
wire [4:0] r5 = i_pixel[15:11];
wire [5:0] g6 = i_pixel[10:5];
wire [4:0] b5 = i_pixel[4:0];

// 扩展到 8 bit（左移补位）
wire [7:0] r8 = {r5, r5[4:2]};
wire [7:0] g8 = {g6, g6[5:4]};
wire [7:0] b8 = {b5, b5[4:2]};

// 加权求和（1 拍延迟）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_valid <= 1'b0;
        o_gray  <= 8'd0;
        o_x     <= 11'd0;
        o_y     <= 10'd0;
    end else begin
        o_valid <= i_valid;
        o_x     <= i_x;
        o_y     <= i_y;
        // Gray = (76*R + 150*G + 29*B) >> 8
        o_gray  <= (8'd76  * r8 +
                    8'd150 * g8 +
                    8'd29  * b8) >> 8;
    end
end

endmodule

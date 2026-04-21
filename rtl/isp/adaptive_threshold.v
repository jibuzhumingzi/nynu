// ============================================================
// Module : adaptive_threshold
// Description : 自适应二值化模块
//   方法：局部均值 + 偏置 C 的阈值判断
//   同时提供全局 Otsu 阈值结果（软件计算后从寄存器写入）
//   output: 1 = 前景（目标），0 = 背景
// ============================================================
module adaptive_threshold #(
    parameter BLOCK_SIZE = 15,   // 局部窗口大小（奇数）
    parameter IMG_W      = 640
)(
    input  wire        clk,
    input  wire        rst_n,

    // 可由软核/MCU 在运行时修改的参数
    input  wire [7:0]  cfg_bias,        // 偏置 C（默认 10）
    input  wire        cfg_use_global,  // 1=使用全局阈值
    input  wire [7:0]  cfg_global_thr,  // 全局阈值（Otsu 计算结果）

    // 输入灰度像素流
    input  wire        i_valid,
    input  wire [7:0]  i_pixel,
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,

    // 输出二值像素流
    output reg         o_valid,
    output reg         o_bin,    // 1=前景
    output reg  [10:0] o_x,
    output reg  [9:0]  o_y
);

// ---- 简化：用 7×1 行均值近似局部均值 ----
// 实际工程中可换成积分图方案
reg [9:0]  sum;        // 最多 7 个 8-bit 值
reg [2:0]  cnt;
reg [7:0]  shift_reg [0:6];
reg [7:0]  local_mean;
integer i;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum       <= 10'd0;
        cnt       <= 3'd0;
        local_mean<= 8'd128;
        o_valid   <= 1'b0;
        o_bin     <= 1'b0;
        o_x       <= 11'd0;
        o_y       <= 10'd0;
        for (i = 0; i < 7; i = i+1) shift_reg[i] <= 8'd0;
    end else begin
        o_valid <= 1'b0;

        if (i_valid) begin
            // 滑动窗口 sum
            sum <= sum + i_pixel - shift_reg[6];
            // 移位
            for (i = 6; i > 0; i = i-1)
                shift_reg[i] <= shift_reg[i-1];
            shift_reg[0] <= i_pixel;

            if (cnt < 7) cnt <= cnt + 3'd1;

            // sum 为 7 个像素之和（10 bit），除以 7 近似为 sum*37>>8（误差<2%）
            local_mean <= (cnt == 7) ? (sum * 10'd37) >> 8 : i_pixel;

            if (cfg_use_global) begin
                o_bin <= (i_pixel > cfg_global_thr) ? 1'b1 : 1'b0;
            end else begin
                // 局部：pixel > mean - bias → 前景
                o_bin <= (i_pixel + cfg_bias > local_mean) ? 1'b1 : 1'b0;
            end

            o_valid <= 1'b1;
            o_x     <= i_x;
            o_y     <= i_y;
        end
    end
end

endmodule

// ============================================================
// Module : ov5640_capture
// Description : OV5640 DVP 接口采集模块
//   - 接收 PCLK / HREF / VSYNC / D[7:0]
//   - 输出行场同步 + RGB565 像素流
//   - 支持 640×480 @ 30fps（可通过参数调整）
// Board  : PGL50H6FB484
// ============================================================
module ov5640_capture #(
    parameter H_ACTIVE = 640,
    parameter V_ACTIVE = 480
)(
    // ---- DVP 物理接口 ----
    input  wire        pclk,        // 像素时钟（来自 OV5640）
    input  wire        href,        // 行有效
    input  wire        vsync,       // 帧同步（高有效）
    input  wire [7:0]  din,         // 8-bit DVP 数据

    // ---- 系统侧复位 ----
    input  wire        rst_n,

    // ---- 像素输出流 ----
    output reg         pixel_valid, // 像素有效
    output reg  [15:0] pixel_data,  // RGB565 数据（MSB first）
    output reg  [10:0] pixel_x,     // 列坐标 0~(H_ACTIVE-1)
    output reg  [9:0]  pixel_y,     // 行坐标 0~(V_ACTIVE-1)
    output reg         frame_start, // 帧开始脉冲（1 pclk）
    output reg         frame_done   // 帧结束脉冲（1 pclk）
);

// ---- 内部信号 ----
reg        vsync_d1, vsync_d2;
reg        href_d1;
reg [7:0]  din_d1;
reg        byte_sel;    // 0=高字节, 1=低字节
reg [7:0]  byte_hi;
reg [10:0] col_cnt;
reg [9:0]  row_cnt;

// ---- 帧同步上升沿检测 ----
always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        vsync_d1 <= 1'b0;
        vsync_d2 <= 1'b0;
    end else begin
        vsync_d1 <= vsync;
        vsync_d2 <= vsync_d1;
    end
end

wire vsync_rise = vsync_d1 & ~vsync_d2;
wire vsync_fall = ~vsync_d1 & vsync_d2;

// ---- 坐标计数 ----
always @(posedge pclk or negedge rst_n) begin
    if (!rst_n) begin
        col_cnt    <= 11'd0;
        row_cnt    <= 10'd0;
        byte_sel   <= 1'b0;
        byte_hi    <= 8'd0;
        pixel_valid<= 1'b0;
        pixel_data <= 16'd0;
        pixel_x    <= 11'd0;
        pixel_y    <= 10'd0;
        frame_start<= 1'b0;
        frame_done <= 1'b0;
        href_d1    <= 1'b0;
        din_d1     <= 8'd0;
    end else begin
        href_d1   <= href;
        din_d1    <= din;
        frame_start <= vsync_rise;
        frame_done  <= vsync_fall;

        if (vsync_rise) begin
            row_cnt  <= 10'd0;
            col_cnt  <= 11'd0;
            byte_sel <= 1'b0;
        end

        pixel_valid <= 1'b0;

        // href 下降沿：行结束，行计数+1
        if (href_d1 & ~href) begin
            row_cnt <= row_cnt + 10'd1;
            col_cnt <= 11'd0;
            byte_sel<= 1'b0;
        end

        if (href) begin
            if (!byte_sel) begin
                // 第一个字节（高字节）
                byte_hi  <= din_d1;
                byte_sel <= 1'b1;
            end else begin
                // 第二个字节（低字节）：凑成 RGB565 输出
                pixel_data  <= {byte_hi, din_d1};
                pixel_x     <= col_cnt;
                pixel_y     <= row_cnt;
                pixel_valid <= (col_cnt < H_ACTIVE) && (row_cnt < V_ACTIVE);
                col_cnt     <= col_cnt + 11'd1;
                byte_sel    <= 1'b0;
            end
        end
    end
end

endmodule

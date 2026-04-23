// ============================================================
// Module : top
// Description : PGL50H6FB484 + OV5640 视觉处理顶层模块
//
//   完整处理链路：
//   OV5640 → ov5640_capture → [color_detect / (rgb2gray →
//     gaussian_filter → adaptive_threshold / sobel_edge)] →
//     target_detect → vision_ctrl → uart_frame_tx → 串口
//
//   两条可选通路（由 cfg_mode 切换）：
//     mode=0: 颜色检测模式（HSV 色块跟踪）
//     mode=1: 边缘/形状检测模式（Sobel + 二值化）
// ============================================================
module top (
    // ---- 系统时钟与复位 ----
    input  wire        sys_clk,      // 50 MHz
    input  wire        sys_rst_n,

    // ---- OV5640 DVP 接口 ----
    input  wire        cam_pclk,
    input  wire        cam_href,
    input  wire        cam_vsync,
    input  wire [7:0]  cam_data,

    // ---- OV5640 控制（SCCB 由上层控制器或 I2C IP 接管）----
    output wire        cam_pwdn,    // 置 0 正常工作
    output wire        cam_rst_n,   // 低有效复位

    // ---- 串口输出 ----
    output wire        uart_tx,

    // ---- 状态指示 ----
    output wire [2:0]  led,

    // ---- 运行时配置（可由软核/拨码开关驱动）----
    input  wire        cfg_mode,       // 0=颜色 1=边缘
    input  wire [7:0]  cfg_h_min,
    input  wire [7:0]  cfg_h_max,
    input  wire [7:0]  cfg_s_min,
    input  wire [7:0]  cfg_s_max,
    input  wire [7:0]  cfg_v_min,
    input  wire [7:0]  cfg_v_max,
    input  wire [7:0]  cfg_bias,       // 二值化偏置
    input  wire [7:0]  cfg_edge_thr,   // 边缘阈值
    input  wire [7:0]  cfg_global_thr, // 全局阈值
    input  wire [15:0] cfg_min_area,
    input  wire [15:0] cfg_max_area
);

// ---- 摄像头固定控制 ----
assign cam_pwdn  = 1'b0;
assign cam_rst_n = sys_rst_n;

// ================================================================
// 1. 摄像头采集
// ================================================================
wire        cap_valid;
wire [15:0] cap_pixel;
wire [10:0] cap_x;
wire [9:0]  cap_y;
wire        cap_frame_start;
wire        cap_frame_done;

ov5640_capture #(
    .H_ACTIVE(640),
    .V_ACTIVE(480)
) u_capture (
    .pclk        (cam_pclk),
    .href        (cam_href),
    .vsync       (cam_vsync),
    .din         (cam_data),
    .rst_n       (sys_rst_n),
    .pixel_valid (cap_valid),
    .pixel_data  (cap_pixel),
    .pixel_x     (cap_x),
    .pixel_y     (cap_y),
    .frame_start (cap_frame_start),
    .frame_done  (cap_frame_done)
);

// ================================================================
// 2a. 颜色检测通路
// ================================================================
wire        col_valid, col_match;
wire [10:0] col_x;
wire [9:0]  col_y;

color_detect u_color (
    .clk        (cam_pclk),
    .rst_n      (sys_rst_n),
    .cfg_h_min  (cfg_h_min),
    .cfg_h_max  (cfg_h_max),
    .cfg_s_min  (cfg_s_min),
    .cfg_s_max  (cfg_s_max),
    .cfg_v_min  (cfg_v_min),
    .cfg_v_max  (cfg_v_max),
    .i_valid    (cap_valid),
    .i_pixel    (cap_pixel),
    .i_x        (cap_x),
    .i_y        (cap_y),
    .o_valid    (col_valid),
    .o_match    (col_match),
    .o_x        (col_x),
    .o_y        (col_y)
);

// ================================================================
// 2b. 边缘/形状检测通路
// ================================================================
// rgb2gray
wire        gray_valid;
wire [7:0]  gray_pixel;
wire [10:0] gray_x;
wire [9:0]  gray_y;

rgb2gray u_rgb2gray (
    .clk    (cam_pclk),
    .rst_n  (sys_rst_n),
    .i_valid(cap_valid),
    .i_pixel(cap_pixel),
    .i_x    (cap_x),
    .i_y    (cap_y),
    .o_valid(gray_valid),
    .o_gray (gray_pixel),
    .o_x    (gray_x),
    .o_y    (gray_y)
);

// gaussian_filter
wire        gf_valid;
wire [7:0]  gf_pixel;
wire [10:0] gf_x;
wire [9:0]  gf_y;

gaussian_filter #(.IMG_W(640), .IMG_H(480)) u_gaussian (
    .clk    (cam_pclk),
    .rst_n  (sys_rst_n),
    .i_valid(gray_valid),
    .i_pixel(gray_pixel),
    .i_x    (gray_x),
    .i_y    (gray_y),
    .o_valid(gf_valid),
    .o_pixel(gf_pixel),
    .o_x    (gf_x),
    .o_y    (gf_y)
);

// sobel_edge
wire        edge_valid, edge_flag;
wire [7:0]  edge_grad;
wire [10:0] edge_x;
wire [9:0]  edge_y;

sobel_edge #(.IMG_W(640)) u_sobel (
    .clk          (cam_pclk),
    .rst_n        (sys_rst_n),
    .cfg_threshold(cfg_edge_thr),
    .i_valid      (gf_valid),
    .i_pixel      (gf_pixel),
    .i_x          (gf_x),
    .i_y          (gf_y),
    .o_valid      (edge_valid),
    .o_edge       (edge_flag),
    .o_grad       (edge_grad),
    .o_x          (edge_x),
    .o_y          (edge_y)
);

// adaptive_threshold（用于形状填充检测）
wire        bin_valid, bin_pixel;
wire [10:0] bin_x;
wire [9:0]  bin_y;

adaptive_threshold #(.IMG_W(640)) u_threshold (
    .clk            (cam_pclk),
    .rst_n          (sys_rst_n),
    .cfg_bias       (cfg_bias),
    .cfg_use_global (1'b0),
    .cfg_global_thr (cfg_global_thr),
    .i_valid        (gf_valid),
    .i_pixel        (gf_pixel),
    .i_x            (gf_x),
    .i_y            (gf_y),
    .o_valid        (bin_valid),
    .o_bin          (bin_pixel),
    .o_x            (bin_x),
    .o_y            (bin_y)
);

// ================================================================
// 3. 通路选择 → target_detect
// ================================================================
wire        sel_valid, sel_fg;
wire [10:0] sel_x;
wire [9:0]  sel_y;

assign sel_valid = cfg_mode ? bin_valid  : col_valid;
assign sel_fg    = cfg_mode ? bin_pixel  : col_match;
assign sel_x     = cfg_mode ? bin_x      : col_x;
assign sel_y     = cfg_mode ? bin_y      : col_y;

wire        tgt_valid;
wire [10:0] tgt_cx;
wire [9:0]  tgt_cy;
wire [16:0] tgt_area;
wire [10:0] tgt_xmin, tgt_xmax;
wire [9:0]  tgt_ymin, tgt_ymax;

target_detect #(
    .IMG_W(640),
    .IMG_H(480)
) u_target (
    .clk            (cam_pclk),
    .rst_n          (sys_rst_n),
    .cfg_min_area   (cfg_min_area),
    .cfg_max_area   (cfg_max_area),
    .i_valid        (sel_valid),
    .i_fg           (sel_fg),
    .i_x            (sel_x),
    .i_y            (sel_y),
    .i_frame_done   (cap_frame_done),
    .o_target_valid (tgt_valid),
    .o_cx           (tgt_cx),
    .o_cy           (tgt_cy),
    .o_area         (tgt_area),
    .o_x_min        (tgt_xmin),
    .o_x_max        (tgt_xmax),
    .o_y_min        (tgt_ymin),
    .o_y_max        (tgt_ymax)
);

// ================================================================
// 4. 视觉控制状态机
// ================================================================
wire        report_en;
wire [10:0] pan_ref;
wire [9:0]  tilt_ref;
wire [3:0]  ctrl_state;
wire [2:0]  ctrl_led;

vision_ctrl #(
    .IMG_W(640),
    .IMG_H(480)
) u_ctrl (
    .clk           (cam_pclk),
    .rst_n         (sys_rst_n),
    .i_frame_done  (cap_frame_done),
    .i_target_valid(tgt_valid),
    .i_cx          (tgt_cx),
    .i_cy          (tgt_cy),
    .i_area        (tgt_area),
    .o_pan_ref     (pan_ref),
    .o_tilt_ref    (tilt_ref),
    .o_report_en   (report_en),
    .o_state       (ctrl_state),
    .o_led         (ctrl_led)
);

assign led = ctrl_led;

// ================================================================
// 5. 串口上报
// ================================================================
uart_frame_tx #(
    .CLK_FREQ (50_000_000),
    .BAUD_RATE(115200)
) u_uart (
    .clk          (cam_pclk),
    .rst_n        (sys_rst_n),
    .i_send       (report_en),
    .i_target_valid(tgt_valid),
    .i_cx         (tgt_cx),
    .i_cy         (tgt_cy),
    .i_area       (tgt_area[15:0]),
    .i_flags      ({4'b0, ctrl_state}),
    .uart_txd     (uart_tx),
    .frame_sent   ()
);

endmodule

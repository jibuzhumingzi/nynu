// ============================================================
// Testbench : tb_image_proc
// Description : 端到端图像处理流水线仿真
//   - 生成模拟 OV5640 DVP 时序（VSYNC/HREF/PCLK/D）
//   - 注入测试图像（纯色块 + 噪声）
//   - 验证颜色检测、质心计算、UART 输出
// ============================================================
`timescale 1ns/1ps

module tb_image_proc;

// ================================================================
// 参数
// ================================================================
parameter IMG_W    = 64;   // 仿真用小分辨率加速
parameter IMG_H    = 48;
parameter PCLK_PERIOD = 20; // 50MHz pclk

// ================================================================
// DUT 接口信号
// ================================================================
reg        cam_pclk;
reg        cam_href;
reg        cam_vsync;
reg [7:0]  cam_data;
reg        sys_rst_n;

wire       uart_txd;
wire [2:0] led;

// 配置信号（检测橙色 H:10~25, S:150~255, V:100~255）
reg [7:0] cfg_h_min = 8'd10;
reg [7:0] cfg_h_max = 8'd25;
reg [7:0] cfg_s_min = 8'd150;
reg [7:0] cfg_s_max = 8'd255;
reg [7:0] cfg_v_min = 8'd100;
reg [7:0] cfg_v_max = 8'd255;
reg [7:0] cfg_bias       = 8'd10;
reg [7:0] cfg_edge_thr   = 8'd50;
reg [7:0] cfg_global_thr = 8'd128;
reg [15:0] cfg_min_area  = 16'd20;
reg [15:0] cfg_max_area  = 16'd5000;
reg        cfg_mode      = 1'b0; // 颜色模式

// ================================================================
// 时钟
// ================================================================
initial cam_pclk = 0;
always #(PCLK_PERIOD/2) cam_pclk = ~cam_pclk;

// ================================================================
// DUT 实例化（使用缩小参数版本测试各子模块）
// ================================================================
// -- 摄像头采集
wire        cap_valid;
wire [15:0] cap_pixel;
wire [10:0] cap_x;
wire [9:0]  cap_y;
wire        cap_frame_done;
wire        cap_frame_start;

ov5640_capture #(.H_ACTIVE(IMG_W), .V_ACTIVE(IMG_H)) u_cap (
    .pclk       (cam_pclk),
    .href       (cam_href),
    .vsync      (cam_vsync),
    .din        (cam_data),
    .rst_n      (sys_rst_n),
    .pixel_valid(cap_valid),
    .pixel_data (cap_pixel),
    .pixel_x    (cap_x),
    .pixel_y    (cap_y),
    .frame_start(cap_frame_start),
    .frame_done (cap_frame_done)
);

// -- 颜色检测
wire        col_valid, col_match;
wire [10:0] col_x;
wire [9:0]  col_y;

color_detect u_color (
    .clk      (cam_pclk), .rst_n(sys_rst_n),
    .cfg_h_min(cfg_h_min), .cfg_h_max(cfg_h_max),
    .cfg_s_min(cfg_s_min), .cfg_s_max(cfg_s_max),
    .cfg_v_min(cfg_v_min), .cfg_v_max(cfg_v_max),
    .i_valid  (cap_valid), .i_pixel(cap_pixel),
    .i_x(cap_x), .i_y(cap_y),
    .o_valid(col_valid), .o_match(col_match),
    .o_x(col_x), .o_y(col_y)
);

// -- 目标检测
wire        tgt_valid;
wire [10:0] tgt_cx;
wire [9:0]  tgt_cy;
wire [16:0] tgt_area;

target_detect #(.IMG_W(IMG_W), .IMG_H(IMG_H)) u_tgt (
    .clk(cam_pclk), .rst_n(sys_rst_n),
    .cfg_min_area(cfg_min_area), .cfg_max_area(cfg_max_area),
    .i_valid(col_valid), .i_fg(col_match),
    .i_x(col_x), .i_y(col_y),
    .i_frame_done(cap_frame_done),
    .o_target_valid(tgt_valid),
    .o_cx(tgt_cx), .o_cy(tgt_cy), .o_area(tgt_area),
    .o_x_min(), .o_x_max(), .o_y_min(), .o_y_max()
);

// ================================================================
// 测试图像生成：DVP 时序驱动
//   前 1/4 行：注入橙色（R=255, G=128, B=0）→ RGB565=0xFD00
//   后 3/4 行：背景（蓝色 RGB565=0x001F）
// ================================================================
integer frame, row, col;
reg [15:0] pix;
integer byte_n; // 0=高字节, 1=低字节

task send_frame;
    integer r, c;
    begin
        // --- 帧同步 ---
        @(posedge cam_pclk); cam_vsync = 1; cam_href = 0;
        repeat(4) @(posedge cam_pclk);
        cam_vsync = 0;
        repeat(2) @(posedge cam_pclk);

        for (r = 0; r < IMG_H; r = r+1) begin
            cam_href = 1;
            for (c = 0; c < IMG_W; c = c+1) begin
                // 中间 1/4 区域注入橙色目标
                if (r >= IMG_H/4 && r < IMG_H*3/4 &&
                    c >= IMG_W/4 && c < IMG_W*3/4) begin
                    pix = 16'hFD00;  // 橙色近似 RGB565
                end else begin
                    pix = 16'h001F;  // 蓝色背景
                end
                // 高字节
                @(posedge cam_pclk); cam_data = pix[15:8];
                // 低字节
                @(posedge cam_pclk); cam_data = pix[7:0];
            end
            cam_href = 0;
            repeat(2) @(posedge cam_pclk);
        end
    end
endtask

// ================================================================
// 仿真主流程
// ================================================================
integer pass_cnt = 0;
integer fail_cnt = 0;

initial begin
    sys_rst_n  = 0;
    cam_href   = 0;
    cam_vsync  = 0;
    cam_data   = 0;
    repeat(20) @(posedge cam_pclk);
    sys_rst_n  = 1;
    repeat(10) @(posedge cam_pclk);

    $display("=== 开始仿真：发送 3 帧测试图像 ===");

    for (frame = 0; frame < 3; frame = frame+1) begin
        $display("[Frame %0d] 发送中...", frame);
        send_frame();
        repeat(50) @(posedge cam_pclk);
        if (tgt_valid) begin
            $display("[Frame %0d] 检测到目标: cx=%0d cy=%0d area=%0d",
                     frame, tgt_cx, tgt_cy, tgt_area);
            if (tgt_cx > IMG_W/4 && tgt_cx < IMG_W*3/4 &&
                tgt_cy > IMG_H/4 && tgt_cy < IMG_H*3/4) begin
                $display("  [PASS] 质心在预期区域内");
                pass_cnt = pass_cnt + 1;
            end else begin
                $display("  [FAIL] 质心超出预期区域");
                fail_cnt = fail_cnt + 1;
            end
        end else begin
            $display("[Frame %0d] 未检测到目标", frame);
            fail_cnt = fail_cnt + 1;
        end
    end

    $display("=== 仿真结束: %0d PASS / %0d FAIL ===", pass_cnt, fail_cnt);
    if (fail_cnt == 0)
        $display("所有测试通过 ✓");
    else
        $display("存在失败用例，请检查 ISP 参数配置");

    #1000;
    $finish;
end

// ================================================================
// 超时保护
// ================================================================
initial begin
    #5_000_000;
    $display("[TIMEOUT] 仿真超时，强制结束");
    $finish;
end

// ================================================================
// 波形导出（ModelSim/GHDL/Icarus）
// ================================================================
initial begin
    $dumpfile("tb_image_proc.vcd");
    $dumpvars(0, tb_image_proc);
end

endmodule

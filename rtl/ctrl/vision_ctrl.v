// ============================================================
// Module : vision_ctrl
// Description : 视觉→控制闭环状态机
//   状态：
//     IDLE      - 等待摄像头帧就绪
//     WAIT_CAM  - 等待摄像头初始化完成
//     DETECTING - 持续检测目标
//     TRACKING  - 目标锁定，输出控制量
//     LOST      - 目标丢失，触发搜索策略
//     FAULT     - 连续多帧异常，进入保护
//   控制输出：
//     - 舵机/电机 PWM 参考量 (pan/tilt)
//     - 串口结果上报使能
//     - 状态指示灯
// ============================================================
module vision_ctrl #(
    parameter LOST_TIMEOUT   = 30,   // 丢失超时帧数
    parameter FAULT_THRESHOLD= 100,  // 连续异常帧数触发 FAULT
    parameter IMG_W          = 640,
    parameter IMG_H          = 480
)(
    input  wire        clk,
    input  wire        rst_n,

    // 目标检测结果（每帧结束后输入）
    input  wire        i_frame_done,
    input  wire        i_target_valid,
    input  wire [10:0] i_cx,
    input  wire [9:0]  i_cy,
    input  wire [16:0] i_area,

    // 控制输出
    output reg  [10:0] o_pan_ref,    // 水平方向控制参考（单位：像素误差）
    output reg  [9:0]  o_tilt_ref,   // 垂直方向控制参考
    output reg         o_report_en,  // 1=本帧结果上报使能
    output reg  [3:0]  o_state,      // 当前状态（调试用）
    output reg  [2:0]  o_led         // 状态指示灯
);

// ---- 状态定义 ----
localparam S_IDLE      = 4'd0;
localparam S_WAIT_CAM  = 4'd1;
localparam S_DETECTING = 4'd2;
localparam S_TRACKING  = 4'd3;
localparam S_LOST      = 4'd4;
localparam S_FAULT     = 4'd5;

// ---- 图像中心 ----
localparam CX_CENTER = IMG_W / 2;
localparam CY_CENTER = IMG_H / 2;

// ---- 内部计数器 ----
reg [3:0]  state, next_state;
reg [7:0]  lost_cnt;     // 丢失帧计数
reg [7:0]  fault_cnt;    // 连续异常帧计数
reg [7:0]  cam_init_cnt; // 摄像头初始化等待帧数

// ---- 状态寄存器 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n)
        state <= S_WAIT_CAM;
    else
        state <= next_state;
end

// ---- 状态转移逻辑 ----
always @(*) begin
    next_state = state;
    case (state)
        S_WAIT_CAM:  if (cam_init_cnt >= 30)     next_state = S_DETECTING;
        S_DETECTING: if (i_frame_done) begin
                         if (i_target_valid)      next_state = S_TRACKING;
                     end
        S_TRACKING:  if (i_frame_done) begin
                         if (!i_target_valid)     next_state = S_LOST;
                     end
        S_LOST:      if (i_frame_done) begin
                         if (i_target_valid)      next_state = S_TRACKING;
                         else if (lost_cnt >= LOST_TIMEOUT) next_state = S_DETECTING;
                     end
                     if (fault_cnt >= FAULT_THRESHOLD)    next_state = S_FAULT;
        S_FAULT:     if (i_frame_done && i_target_valid) next_state = S_DETECTING;
        default:     next_state = S_DETECTING;
    endcase
end

// ---- 输出逻辑 ----
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_pan_ref    <= 11'd0;
        o_tilt_ref   <= 10'd0;
        o_report_en  <= 1'b0;
        o_led        <= 3'b000;
        lost_cnt     <= 8'd0;
        fault_cnt    <= 8'd0;
        cam_init_cnt <= 8'd0;
        o_state      <= S_WAIT_CAM;
    end else begin
        o_report_en <= 1'b0;
        o_state     <= state;

        // 摄像头初始化等待
        if (state == S_WAIT_CAM && i_frame_done)
            cam_init_cnt <= cam_init_cnt + 8'd1;

        case (state)
            S_DETECTING: begin
                o_led       <= 3'b001;  // 蓝灯：检测中
                lost_cnt    <= 8'd0;
                fault_cnt   <= 8'd0;
                o_pan_ref   <= CX_CENTER;
                o_tilt_ref  <= CY_CENTER;
            end

            S_TRACKING: begin
                o_led <= 3'b010;  // 绿灯：锁定目标
                if (i_frame_done && i_target_valid) begin
                    // 误差 = 目标质心 - 图像中心
                    o_pan_ref   <= i_cx;
                    o_tilt_ref  <= i_cy;
                    o_report_en <= 1'b1;
                    lost_cnt    <= 8'd0;
                end
            end

            S_LOST: begin
                o_led <= 3'b100;  // 红灯：目标丢失
                if (i_frame_done) begin
                    if (!i_target_valid) begin
                        lost_cnt  <= lost_cnt + 8'd1;
                        fault_cnt <= fault_cnt + 8'd1;
                    end else begin
                        lost_cnt  <= 8'd0;
                        fault_cnt <= 8'd0;
                    end
                end
                // 搜索策略：保持上次控制量（可扩展为扫描模式）
            end

            S_FAULT: begin
                o_led      <= 3'b110;  // LED[2:1] 亮（红+绿=黄色，代表故障保护）
                o_pan_ref  <= CX_CENTER;
                o_tilt_ref <= CY_CENTER;
                fault_cnt  <= 8'd0;
            end

            default: o_led <= 3'b000;
        endcase
    end
end

endmodule

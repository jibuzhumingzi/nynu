// ============================================================
// Module : target_detect
// Description : 目标检测与质心计算
//   逐帧累积前景像素坐标，帧结束时输出：
//     - 质心坐标 (cx, cy)
//     - 像素计数（面积）
//     - 有效目标标志
//   支持最小/最大面积过滤
//   质心除法采用移位迭代方式，避免单周期大位宽组合除法关键路径
// ============================================================
module target_detect #(
    parameter IMG_W     = 640,
    parameter IMG_H     = 480,
    parameter MIN_AREA  = 200,   // 最小前景像素数（去噪）
    parameter MAX_AREA  = 60000  // 最大前景像素数
)(
    input  wire        clk,
    input  wire        rst_n,

    // 运行时配置
    input  wire [15:0] cfg_min_area,
    input  wire [15:0] cfg_max_area,

    // 输入前景二值流（来自 color_detect 或 adaptive_threshold）
    input  wire        i_valid,
    input  wire        i_fg,      // 1=前景
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,
    input  wire        i_frame_done,  // 帧结束脉冲

    // 输出（帧结束后若干拍，div_done=1 时有效）
    output reg         o_target_valid,  // 1=检测到有效目标
    output reg  [10:0] o_cx,            // 质心 X
    output reg  [9:0]  o_cy,            // 质心 Y
    output reg  [16:0] o_area,          // 前景像素数
    output reg  [10:0] o_x_min,         // 包围盒
    output reg  [10:0] o_x_max,
    output reg  [9:0]  o_y_min,
    output reg  [9:0]  o_y_max
);

// 帧内累积寄存器
reg [23:0] sum_x;   // 最多 640*480 = 307200，需 19 bit；用 24 保余量
reg [23:0] sum_y;
reg [16:0] pixel_cnt;
reg [10:0] bbox_x_min, bbox_x_max;
reg [9:0]  bbox_y_min, bbox_y_max;

// ---- 迭代除法状态机 ----
// 使用非恢复余数算法，对 24-bit 分子 / 17-bit 分母，24 步完成
localparam DIV_IDLE  = 2'd0;
localparam DIV_CX    = 2'd1;  // 计算 cx = sum_x / pixel_cnt
localparam DIV_CY    = 2'd2;  // 计算 cy = sum_y / pixel_cnt
localparam DIV_DONE  = 2'd3;

reg [1:0]  div_state;
reg [4:0]  div_step;       // 最多 24 步
reg [23:0] div_a;          // 被除数
reg [16:0] div_b;          // 除数
reg [23:0] div_q;          // 商
reg [23:0] div_r;          // 余数
reg        div_area_valid; // 面积在有效范围

// 存储帧结束时的快照，供除法使用
reg [23:0] snap_sum_x, snap_sum_y;
reg [16:0] snap_cnt;
reg [10:0] snap_xmin, snap_xmax;
reg [9:0]  snap_ymin, snap_ymax;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sum_x          <= 24'd0;
        sum_y          <= 24'd0;
        pixel_cnt      <= 17'd0;
        bbox_x_min     <= {11{1'b1}};
        bbox_x_max     <= 11'd0;
        bbox_y_min     <= {10{1'b1}};
        bbox_y_max     <= 10'd0;
        o_target_valid <= 1'b0;
        o_cx           <= 11'd0;
        o_cy           <= 10'd0;
        o_area         <= 17'd0;
        o_x_min        <= 11'd0;
        o_x_max        <= 11'd0;
        o_y_min        <= 10'd0;
        o_y_max        <= 10'd0;
        div_state      <= DIV_IDLE;
        div_step       <= 5'd0;
        div_area_valid <= 1'b0;
    end else begin
        o_target_valid <= 1'b0;

        // ---- 帧像素累积 ----
        if (i_frame_done) begin
            // 保存快照，开始除法
            snap_sum_x <= sum_x;
            snap_sum_y <= sum_y;
            snap_cnt   <= pixel_cnt;
            snap_xmin  <= bbox_x_min;
            snap_xmax  <= bbox_x_max;
            snap_ymin  <= bbox_y_min;
            snap_ymax  <= bbox_y_max;
            div_area_valid <= (pixel_cnt >= cfg_min_area &&
                               pixel_cnt <= cfg_max_area &&
                               pixel_cnt != 0);
            // 清零累积器
            sum_x      <= 24'd0;
            sum_y      <= 24'd0;
            pixel_cnt  <= 17'd0;
            bbox_x_min <= {11{1'b1}};
            bbox_x_max <= 11'd0;
            bbox_y_min <= {10{1'b1}};
            bbox_y_max <= 10'd0;

            if (div_area_valid)
                div_state <= DIV_CX;

        end else if (i_valid && i_fg) begin
            sum_x     <= sum_x + {13'b0, i_x};
            sum_y     <= sum_y + {14'b0, i_y};
            pixel_cnt <= pixel_cnt + 17'd1;
            if (i_x < bbox_x_min) bbox_x_min <= i_x;
            if (i_x > bbox_x_max) bbox_x_max <= i_x;
            if (i_y < bbox_y_min) bbox_y_min <= i_y;
            if (i_y > bbox_y_max) bbox_y_max <= i_y;
        end

        // ---- 迭代除法 ----
        case (div_state)
            DIV_IDLE: ; // 等待

            DIV_CX: begin
                if (div_step == 0) begin
                    // 初始化
                    div_a   <= snap_sum_x;
                    div_b   <= snap_cnt;
                    div_q   <= 24'd0;
                    div_r   <= 24'd0;
                    div_step<= 5'd1;
                end else if (div_step <= 24) begin
                    // 非恢复余数法（简化为恢复法）
                    div_r <= {div_r[22:0], div_a[23]};
                    div_a <= {div_a[22:0], 1'b0};
                    if ({div_r[22:0], div_a[23]} >= {7'b0, div_b}) begin
                        div_q <= {div_q[22:0], 1'b1};
                        div_r <= {div_r[22:0], div_a[23]} - {7'b0, div_b};
                    end else begin
                        div_q <= {div_q[22:0], 1'b0};
                    end
                    div_step <= div_step + 5'd1;
                end else begin
                    o_cx     <= div_q[10:0];
                    // 切换到 CY 除法
                    div_a    <= snap_sum_y;
                    div_b    <= snap_cnt;
                    div_q    <= 24'd0;
                    div_r    <= 24'd0;
                    div_step <= 5'd1;
                    div_state<= DIV_CY;
                end
            end

            DIV_CY: begin
                if (div_step <= 24) begin
                    div_r <= {div_r[22:0], div_a[23]};
                    div_a <= {div_a[22:0], 1'b0};
                    if ({div_r[22:0], div_a[23]} >= {7'b0, div_b}) begin
                        div_q <= {div_q[22:0], 1'b1};
                        div_r <= {div_r[22:0], div_a[23]} - {7'b0, div_b};
                    end else begin
                        div_q <= {div_q[22:0], 1'b0};
                    end
                    div_step <= div_step + 5'd1;
                end else begin
                    div_state <= DIV_DONE;
                end
            end

            DIV_DONE: begin
                o_cy           <= div_q[9:0];
                o_area         <= snap_cnt;
                o_x_min        <= snap_xmin;
                o_x_max        <= snap_xmax;
                o_y_min        <= snap_ymin;
                o_y_max        <= snap_ymax;
                o_target_valid <= div_area_valid;
                div_state      <= DIV_IDLE;
                div_step       <= 5'd0;
            end

            default: div_state <= DIV_IDLE;
        endcase
    end
end

endmodule

// ============================================================
// Module : color_detect
// Description : HSV 颜色空间色块检测
//   输入：RGB565 像素
//   输出：色块标志（前景/背景）
//
//   颜色判定：先 RGB→HSV，再对 H/S/V 范围判定
//   支持通过寄存器动态配置目标颜色范围
// ============================================================
module color_detect (
    input  wire        clk,
    input  wire        rst_n,

    // 目标颜色 HSV 范围（运行时可配置）
    input  wire [7:0]  cfg_h_min,   // H 最小值 (0~179)
    input  wire [7:0]  cfg_h_max,   // H 最大值
    input  wire [7:0]  cfg_s_min,   // S 最小值 (0~255)
    input  wire [7:0]  cfg_s_max,
    input  wire [7:0]  cfg_v_min,   // V 最小值 (0~255)
    input  wire [7:0]  cfg_v_max,

    // 输入 RGB565 像素流
    input  wire        i_valid,
    input  wire [15:0] i_pixel,
    input  wire [10:0] i_x,
    input  wire [9:0]  i_y,

    // 输出
    output reg         o_valid,
    output reg         o_match,   // 1=颜色匹配（前景）
    output reg  [10:0] o_x,
    output reg  [9:0]  o_y
);

// ---- Stage 1: RGB565 → RGB888 ----
reg [7:0] r8, g8, b8;
reg       s1_valid;
reg [10:0] s1_x;
reg [9:0]  s1_y;

always @(posedge clk) begin
    s1_valid <= i_valid;
    s1_x     <= i_x;
    s1_y     <= i_y;
    r8 <= {i_pixel[15:11], i_pixel[15:13]};
    g8 <= {i_pixel[10:5],  i_pixel[10:9]};
    b8 <= {i_pixel[4:0],   i_pixel[4:2]};
end

// ---- Stage 2: 求 max/min/delta ----
reg [7:0] cmax, cmin;
reg [8:0] cdelta;   // max-min，可能需要 9 bit（防溢出）
reg [7:0] s2_r, s2_g, s2_b;
reg       s2_valid;
reg [10:0] s2_x;
reg [9:0]  s2_y;

always @(posedge clk) begin
    s2_valid <= s1_valid;
    s2_x     <= s1_x;
    s2_y     <= s1_y;
    s2_r <= r8; s2_g <= g8; s2_b <= b8;

    if (r8 >= g8 && r8 >= b8)      cmax <= r8;
    else if (g8 >= r8 && g8 >= b8) cmax <= g8;
    else                            cmax <= b8;

    if (r8 <= g8 && r8 <= b8)      cmin <= r8;
    else if (g8 <= r8 && g8 <= b8) cmin <= g8;
    else                            cmin <= b8;

    cdelta <= (r8 >= g8 && r8 >= b8) ? r8 - ((g8 <= b8) ? g8 : b8) :
              (g8 >= r8 && g8 >= b8) ? g8 - ((r8 <= b8) ? r8 : b8) :
                                       b8 - ((r8 <= g8) ? r8 : g8);
end

// ---- Stage 3: 计算 V 和 S ----
reg [7:0] hsv_v, hsv_s;
reg [7:0] s3_r, s3_g, s3_b, s3_cmax, s3_cmin;
reg [8:0] s3_delta;
reg       s3_valid;
reg [10:0] s3_x;
reg [9:0]  s3_y;

always @(posedge clk) begin
    s3_valid  <= s2_valid;
    s3_x      <= s2_x;
    s3_y      <= s2_y;
    s3_r      <= s2_r;
    s3_g      <= s2_g;
    s3_b      <= s2_b;
    s3_cmax   <= cmax;
    s3_cmin   <= cmin;
    s3_delta  <= cdelta;

    hsv_v  <= cmax;
    // S = delta/max * 255；用 16-bit 中间值避免溢出，结果饱和到 8-bit
    hsv_s  <= (cmax == 0) ? 8'd0 :
              (({8'b0, cdelta[7:0]} * 9'd255) / {1'b0, cmax} > 16'd255) ? 8'd255 :
              ({8'b0, cdelta[7:0]} * 9'd255) / {1'b0, cmax};
end

// ---- Stage 4: 计算 H（整数近似，0~179）----
// H 计算需除法，这里用查找近似法：
//   若 max=R: H = 43*(G-B)/delta         (mod 256 near 0 or 170)
//   若 max=G: H = 85 + 43*(B-R)/delta
//   若 max=B: H = 171 + 43*(R-G)/delta
reg [7:0] hsv_h;
reg       s4_valid;
reg [10:0] s4_x;
reg [9:0]  s4_y;
reg [7:0]  s4_v, s4_s;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        s4_valid <= 1'b0;
        hsv_h    <= 8'd0;
    end else begin
        s4_valid <= s3_valid;
        s4_x     <= s3_x;
        s4_y     <= s3_y;
        s4_v     <= hsv_v;
        s4_s     <= hsv_s;

        if (s3_delta == 0) begin
            hsv_h <= 8'd0;
        end else if (s3_cmax == s3_r) begin
            // H = 43*(G-B)/delta（有符号差值，结果模 170 归一化）
            if (s3_g >= s3_b)
                hsv_h <= (8'd43 * (s3_g - s3_b)) / s3_delta[7:0];
            else begin
                // G < B：差值为负，加 170 保证 H 在 [0,170) 内
                // 用 (43*(G-B+255))/delta 可能超出 170，改为直接减
                hsv_h <= 8'd170 - (8'd43 * (s3_b - s3_g)) / s3_delta[7:0];
            end
        end else if (s3_cmax == s3_g) begin
            // H = 85 + 43*(B-R)/delta
            if (s3_b >= s3_r)
                hsv_h <= 8'd85 + (8'd43 * (s3_b - s3_r)) / s3_delta[7:0];
            else
                hsv_h <= 8'd85 - (8'd43 * (s3_r - s3_b)) / s3_delta[7:0];
        end else begin
            // H = 171 + 43*(R-G)/delta
            if (s3_r >= s3_g)
                hsv_h <= 8'd171 + (8'd43 * (s3_r - s3_g)) / s3_delta[7:0];
            else
                hsv_h <= 8'd171 - (8'd43 * (s3_g - s3_r)) / s3_delta[7:0];
        end
    end
end

// ---- Stage 5: 范围判定 ----
// 注意：H 可跨越 0（如红色 H ≈ 0/170），需特殊处理
wire h_wrap = (cfg_h_min > cfg_h_max);  // 红色类颜色会出现

// 提前用组合逻辑计算匹配标志
wire h_ok_nowrap = (hsv_h >= cfg_h_min && hsv_h <= cfg_h_max);
wire h_ok_wrap   = (hsv_h >= cfg_h_min || hsv_h <= cfg_h_max);
wire h_ok        = h_wrap ? h_ok_wrap : h_ok_nowrap;
wire s_ok        = (s4_s >= cfg_s_min && s4_s <= cfg_s_max);
wire v_ok        = (s4_v >= cfg_v_min && s4_v <= cfg_v_max);

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        o_valid <= 1'b0;
        o_match <= 1'b0;
    end else begin
        o_valid <= s4_valid;
        o_x     <= s4_x;
        o_y     <= s4_y;

        if (s4_valid) begin
            o_match <= h_ok && s_ok && v_ok;
        end else begin
            o_match <= 1'b0;
        end
    end
end

endmodule

module improved_sound_localization (
    input wire clk,
    input wire rst_n,
    input wire mic_left,
    input wire mic_right,
    output reg [2:0] leds
);

// 添加数字滤波器
reg [15:0] left_filter = 0;
reg [15:0] right_filter = 0;

// 移动平均滤波
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        left_filter <= 0;
        right_filter <= 0;
    end else begin
        left_filter <= (left_filter * 15 + {8'b0, mic_left}) / 16;
        right_filter <= (right_filter * 15 + {8'b0, mic_right}) / 16;
    end
end

// 自适应阈值
reg [15:0] noise_floor = 100;
wire left_signal = (left_filter > noise_floor);
wire right_signal = (right_filter > noise_floor);

// 相位差检测
reg [7:0] phase_diff_counter = 0;
reg waiting_first = 0;
reg first_channel = 0;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        phase_diff_counter <= 0;
        waiting_first <= 0;
    end else begin
        if (left_signal && !waiting_first) begin
            waiting_first <= 1;
            first_channel <= 0; // 左侧先检测到
            phase_diff_counter <= 0;
        end else if (right_signal && !waiting_first) begin
            waiting_first <= 1;
            first_channel <= 1; // 右侧先检测到
            phase_diff_counter <= 0;
        end else if (waiting_first) begin
            if (phase_diff_counter < 200) begin
                phase_diff_counter <= phase_diff_counter + 1;
                
                // 检测另一个通道的信号
                if ((!first_channel && right_signal) || (first_channel && left_signal)) begin
                    // 计算时间差并判断方向
                    if (phase_diff_counter < 80) begin
                        leds <= first_channel ? 3'b001 : 3'b100;
                    end else begin
                        leds <= 3'b010; // 中间
                    end
                    waiting_first <= 0;
                end
            end else begin
                // 超时，只有一个通道有信号
                leds <= first_channel ? 3'b001 : 3'b100;
                waiting_first <= 0;
            end
        end
    end
end

endmodule
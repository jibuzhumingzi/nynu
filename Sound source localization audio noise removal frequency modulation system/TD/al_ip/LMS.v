// 24位一阶LMS自适应滤波器，适用于与ES8388等高精度音频芯片对接
module LMS #(
    parameter DATA_WIDTH = 24,
    parameter COEF_WIDTH = 24,
    parameter MU = 50 // 步长因子，越大收敛越快但易震荡
)(
    input clk,
    input rst_n,
    input signed [DATA_WIDTH-1:0] din,      // 输入信号
    input signed [DATA_WIDTH-1:0] dref,     // 期望信号
    output signed [DATA_WIDTH-1:0] dout     // 滤波输出
);

reg signed [COEF_WIDTH-1:0] coef;           // 滤波系数
reg signed [DATA_WIDTH-1:0] din_r;
reg signed [DATA_WIDTH*2-1:0] y;            // 滤波器输出（宽度扩展防止溢出）
reg signed [DATA_WIDTH-1:0] e;              // 误差信号

always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        coef <= 0;
        din_r <= 0;
        y <= 0;
    end else begin
        din_r <= din;
        y <= coef * din_r;
        e <= dref - y[DATA_WIDTH+COEF_WIDTH-2:COEF_WIDTH-1]; // 截取高位做输出
        coef <= coef + ((MU * e * din_r) >>> (COEF_WIDTH-1));
    end
end

assign dout = y[DATA_WIDTH+COEF_WIDTH-2:COEF_WIDTH-1]; // 截取高位输出

endmodule
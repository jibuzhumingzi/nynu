module noise_suppress #(
    parameter DATA_WIDTH = 32,
    parameter THRESHOLD = 32'h0000_00FF  // 降低噪声阈值
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] audio_in,
    output reg [DATA_WIDTH-1:0] audio_out
);

reg [DATA_WIDTH-1:0] last_sample;
// 计算绝对值（考虑有符号数）
wire [DATA_WIDTH-2:0] abs_audio = (audio_in[DATA_WIDTH-1]) ? 
                                 (~audio_in[DATA_WIDTH-2:0] + 1) : 
                                 audio_in[DATA_WIDTH-2:0];

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        audio_out <= 0;
        last_sample <= 0;
    end else begin
        // 简单的噪声门限 - 只处理小幅值信号
        if (abs_audio < THRESHOLD) begin
            audio_out <= 0;  // 低于阈值，认为是噪声
        end else begin
            audio_out <= audio_in;  // 高于阈值，保留信号
        end
        last_sample <= audio_in;
    end
end

endmodule
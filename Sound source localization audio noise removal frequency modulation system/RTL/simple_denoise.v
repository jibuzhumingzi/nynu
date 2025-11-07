module simple_denoise(
    input               clk,
    input               rst_n,
    input      [31:0]   primary_data,    // 主麦克风（语音+噪声）
    input      [31:0]   reference_data,  // 参考麦克风（环境噪声）
    output reg [31:0]   denoised_data    // 去噪后的语音
);

// 简单的谱减法去噪
reg [31:0] noise_estimate;
reg [31:0] primary_delayed;

// 延迟主信号以对齐处理
always @(posedge clk) begin
    primary_delayed <= primary_data;
end

// 估计噪声（使用参考麦克风）
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        noise_estimate <= 32'd0;
    end else begin
        // 简单的移动平均来估计噪声
        noise_estimate <= (noise_estimate * 7 + reference_data) / 16;
    end
end

// 谱减法：主信号 - 噪声估计
always @(posedge clk) begin
    // 简单的减法，确保不会下溢
    if (primary_delayed > noise_estimate) begin
        denoised_data <= primary_delayed - (noise_estimate >> 2);  // 只减去部分噪声
    end else begin
        denoised_data <= 32'd0;  // 避免负值
    end
end

endmodule
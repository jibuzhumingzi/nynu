// 旋转因子ROM（128x16位，存储cos/sin值，Q1.15格式）
module rom_128x16 (
    input        clk,
    input  [6:0] addr,  // 地址：0~127
    output reg [15:0] dout  // 数据输出
);

// 初始化旋转因子（使用前文gen_twiddle256.py生成的数值）
// 以下为示例值，需替换为实际计算的128个数据
always @(posedge clk) begin
    case (addr)
        7'd0:  dout <= 16'h7FFF;  // cos(0) = 1.0
        7'd1:  dout <= 16'h7FFC;  // cos(-2π*1/256) ≈ 0.9999
        7'd2:  dout <= 16'h7FF3;  // cos(-2π*2/256) ≈ 0.9997
        7'd3:  dout <= 16'h7FE8;  // 示例值，需补充完整
        // ... 其余124个值（省略）
        default: dout <= 16'h0000;
    endcase
end

endmodule
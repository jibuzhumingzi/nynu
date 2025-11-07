module pitch_shift #(
    parameter DATA_WIDTH = 32
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] audio_in,
    input wire [2:0] pitch_factor,  // 变调因子：000=0.5x, 001=0.75x, 010=1x, 011=1.25x, 100=1.5x
    output reg [DATA_WIDTH-1:0] audio_out,
    output reg data_valid
);

// 双缓冲区用于音频数据存储
reg [DATA_WIDTH-1:0] buffer [0:255];  // 减小缓冲区大小
reg [7:0] write_ptr;
reg [7:0] read_ptr;
reg [15:0] phase_accum;
reg data_ready;

// 重采样实现变调
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        write_ptr <= 0;
        read_ptr <= 0;
        phase_accum <= 0;
        audio_out <= 0;
        data_valid <= 0;
        data_ready <= 0;
        
        // 初始化缓冲区
        for (integer i = 0; i < 256; i = i + 1) begin
            buffer[i] <= 0;
        end
    end else begin
        // 写入新数据到缓冲区
        buffer[write_ptr] <= audio_in;
        write_ptr <= write_ptr + 1;
        
        // 当缓冲区有足够数据时开始处理
        if (write_ptr > 8'd128) begin
            data_ready <= 1'b1;
        end
        
        if (data_ready) begin
            // 根据变调因子计算读取步长
            case(pitch_factor)
                3'b000: phase_accum <= phase_accum + 8'd64;   // 0.5x 慢速
                3'b001: phase_accum <= phase_accum + 8'd85;   // 0.75x  
                3'b010: phase_accum <= phase_accum + 8'd128;  // 1x 正常
                3'b011: phase_accum <= phase_accum + 8'd160;  // 1.25x
                3'b100: phase_accum <= phase_accum + 8'd192;  // 1.5x 快速
                default: phase_accum <= phase_accum + 8'd128; // 默认正常速度
            endcase
            
            read_ptr <= phase_accum[15:8]; // 取高位作为读取地址
            
            // 简单的数据输出（后续可添加插值）
            audio_out <= buffer[read_ptr];
            data_valid <= 1'b1;
        end else begin
            data_valid <= 1'b0;
        end
    end
end

endmodule
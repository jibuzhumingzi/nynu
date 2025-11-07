// 位反转模块：将输入数据按位反转顺序重排
module bit_reverse #(
    parameter DATA_WIDTH = 16,
    parameter ADDR_WIDTH = 8
) (
    input                       clk,
    input                       rst_n,
    input  [DATA_WIDTH-1:0]     din_re,
    input  [DATA_WIDTH-1:0]     din_im,
    input                       din_valid,
    output reg [DATA_WIDTH-1:0] dout_re,
    output reg [DATA_WIDTH-1:0] dout_im,
    output reg                  dout_valid
);

// 内部存储器
reg [DATA_WIDTH-1:0] mem_re [0:(1<<ADDR_WIDTH)-1];
reg [DATA_WIDTH-1:0] mem_im [0:(1<<ADDR_WIDTH)-1];
reg [ADDR_WIDTH-1:0] wr_addr, rd_addr;
reg [1:0] state;

localparam IDLE  = 2'b00;
localparam WRITE = 2'b01;
localparam READ  = 2'b10;

// 位反转地址计算函数
function [ADDR_WIDTH-1:0] reverse;
    input [ADDR_WIDTH-1:0] addr;
    integer i;
    begin
        reverse = 0;
        for (i = 0; i < ADDR_WIDTH; i = i + 1) begin
            reverse[ADDR_WIDTH-1-i] = addr[i];
        end
    end
endfunction

// 状态机控制：写入→位反转读出
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        wr_addr    <= 0;
        rd_addr    <= 0;
        state      <= IDLE;
        dout_valid <= 0;
        dout_re    <= 0;
        dout_im    <= 0;
    end else begin
        case (state)
            IDLE: begin
                if (din_valid) begin
                    state <= WRITE;
                    wr_addr <= 0;
                end
            end
            WRITE: begin  // 写入256个输入数据
                mem_re[wr_addr] <= din_re;
                mem_im[wr_addr] <= din_im;
                if (wr_addr == (1<<ADDR_WIDTH)-1) begin
                    state <= READ;
                    rd_addr <= 0;
                end else begin
                    wr_addr <= wr_addr + 1;
                end
            end
            READ: begin  // 按位反转顺序读出
                dout_re    <= mem_re[reverse(rd_addr)];
                dout_im    <= mem_im[reverse(rd_addr)];
                dout_valid <= 1'b1;
                if (rd_addr == (1<<ADDR_WIDTH)-1) begin
                    state <= IDLE;
                    dout_valid <= 1'b0;
                end else begin
                    rd_addr <= rd_addr + 1;
                end
            end
        endcase
    end
end

endmodule
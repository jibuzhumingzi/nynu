// ============================================================
// Module : uart_tx
// Description : UART 发送模块
//   - 支持 8N1 格式
//   - 波特率由参数 CLK_FREQ / BAUD_RATE 自动计算
//   - 提供帧打包发送接口：一次发送 N 字节协议帧
// ============================================================
module uart_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input  wire       clk,
    input  wire       rst_n,

    // 字节发送接口
    input  wire       tx_start,  // 上升沿触发发送
    input  wire [7:0] tx_data,   // 待发送字节
    output reg        tx_busy,   // 1=正在发送
    output reg        tx_done,   // 1=发送完成（1 clk 脉冲）

    // 串口物理信号
    output reg        uart_txd
);

localparam BIT_PERIOD = CLK_FREQ / BAUD_RATE;

reg [$clog2(BIT_PERIOD)-1:0] baud_cnt;
reg [3:0]  bit_idx;   // 0=start, 1~8=data, 9=stop
reg [9:0]  shift_reg; // {stop, data[7:0], start}
reg        tx_en;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        uart_txd <= 1'b1;
        tx_busy  <= 1'b0;
        tx_done  <= 1'b0;
        baud_cnt <= 0;
        bit_idx  <= 4'd0;
        shift_reg<= 10'h3FF;
        tx_en    <= 1'b0;
    end else begin
        tx_done <= 1'b0;

        if (tx_start && !tx_busy) begin
            shift_reg <= {1'b1, tx_data, 1'b0};  // stop + data + start
            tx_busy   <= 1'b1;
            baud_cnt  <= 0;
            bit_idx   <= 4'd0;
            tx_en     <= 1'b1;
        end

        if (tx_en) begin
            if (baud_cnt < BIT_PERIOD - 1) begin
                baud_cnt <= baud_cnt + 1;
            end else begin
                baud_cnt  <= 0;
                uart_txd  <= shift_reg[0];
                shift_reg <= {1'b1, shift_reg[9:1]};
                bit_idx   <= bit_idx + 4'd1;

                if (bit_idx == 4'd9) begin
                    tx_busy <= 1'b0;
                    tx_done <= 1'b1;
                    tx_en   <= 1'b0;
                    uart_txd<= 1'b1;
                end
            end
        end
    end
end

endmodule

// ============================================================
// Module : uart_frame_tx
// Description : 封装帧格式发送
//   帧格式：0xAA 0x55 [TYPE] [CX_H] [CX_L] [CY_H] [CY_L]
//           [AREA_H] [AREA_L] [FLAGS] [CHK]
//   CHK = XOR of TYPE..FLAGS
// ============================================================
module uart_frame_tx #(
    parameter CLK_FREQ  = 50_000_000,
    parameter BAUD_RATE = 115200
)(
    input  wire        clk,
    input  wire        rst_n,

    // 结果输入
    input  wire        i_send,         // 发送触发（帧结束后）
    input  wire        i_target_valid,
    input  wire [10:0] i_cx,
    input  wire [9:0]  i_cy,
    input  wire [15:0] i_area,
    input  wire [7:0]  i_flags,        // 扩展标志位

    output wire        uart_txd,
    output reg         frame_sent
);

// 字节 FIFO（11 字节帧）
reg [7:0]  frame [0:10];
reg [3:0]  byte_idx;
reg        sending;
wire       byte_done;
wire       byte_busy;
reg        byte_start;
reg [7:0]  byte_data;

uart_tx #(
    .CLK_FREQ (CLK_FREQ),
    .BAUD_RATE(BAUD_RATE)
) u_uart_tx (
    .clk     (clk),
    .rst_n   (rst_n),
    .tx_start(byte_start),
    .tx_data (byte_data),
    .tx_busy (byte_busy),
    .tx_done (byte_done),
    .uart_txd(uart_txd)
);

// 帧构建
wire [7:0] chk = 8'h01 ^ i_cx[10:8] ^ i_cx[7:0] ^ {6'b0, i_cy[9:8]} ^ i_cy[7:0]
               ^ i_area[15:8] ^ i_area[7:0] ^ i_flags;

always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        sending    <= 1'b0;
        byte_idx   <= 4'd0;
        byte_start <= 1'b0;
        frame_sent <= 1'b0;
    end else begin
        byte_start <= 1'b0;
        frame_sent <= 1'b0;

        if (i_send && !sending) begin
            frame[0]  <= 8'hAA;
            frame[1]  <= 8'h55;
            frame[2]  <= i_target_valid ? 8'h01 : 8'h00;
            frame[3]  <= {5'b0, i_cx[10:8]};
            frame[4]  <= i_cx[7:0];
            frame[5]  <= {6'b0, i_cy[9:8]};
            frame[6]  <= i_cy[7:0];
            frame[7]  <= i_area[15:8];
            frame[8]  <= i_area[7:0];
            frame[9]  <= i_flags;
            frame[10] <= chk;
            byte_idx  <= 4'd0;
            sending   <= 1'b1;
        end

        if (sending && !byte_busy && !byte_start) begin
            if (byte_idx <= 4'd10) begin
                byte_data  <= frame[byte_idx];
                byte_start <= 1'b1;
                byte_idx   <= byte_idx + 4'd1;
            end else begin
                sending    <= 1'b0;
                frame_sent <= 1'b1;
            end
        end
    end
end

endmodule

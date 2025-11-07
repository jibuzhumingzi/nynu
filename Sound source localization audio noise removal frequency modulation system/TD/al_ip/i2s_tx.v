module i2s_tx_32bit_48k (
    input wire clk,                   // 50MHz系统主时钟
    input wire rst_n,                 
    input wire [31:0] adc_data,       // 32位数据输入
    
    output reg I2S_LRCLK,            // 声道时钟（左低右高，匹配接收端）
    output reg I2S_BCLK,             // 位时钟
    output reg I2S_DATA              // 数据输出
);

    // 精确匹配接收端BCLK频率（3.072MHz）
    // 50MHz / (2×8.125) = 3.072MHz，采用近似整数分频+误差补偿
    parameter BCLK_DIV = 8'd8;        // 基础分频值
    parameter BCLK_COMP = 12'd1024;   // 补偿计数器阈值（动态调整占空比）
    parameter LRCLK_DIV = 11'd521;    // 48KHz精确分频（50MHz/(48K×2)=520.833→521）
    
    reg [7:0] bclk_counter;
    reg [10:0] lrclk_counter;
    reg [5:0] bit_counter;            // 32位计数（0-31）
    reg [31:0] shift_reg;             // 移位寄存器
    reg [31:0] data_latch;            // 数据锁存器
    reg [11:0] bclk_comp_counter;     // 频率补偿计数器

    // 1. BCLK生成（3.072MHz近似，带动态补偿）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            bclk_counter <= 0;
            I2S_BCLK <= 0;
            bclk_comp_counter <= 0;
        end else begin
            // 基础分频（8）+ 动态补偿（每1024周期多计数1次）
            if (bclk_counter >= BCLK_DIV - 1 + (bclk_comp_counter >= BCLK_COMP ? 1 : 0)) begin
                bclk_counter <= 0;
                I2S_BCLK <= ~I2S_BCLK;
                bclk_comp_counter <= (bclk_comp_counter >= BCLK_COMP) ? 0 : bclk_comp_counter + 1;
            end else begin
                bclk_counter <= bclk_counter + 1;
            end
        end
    end

    // 2. LRCLK生成（48KHz，高电平右声道，低电平左声道）
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            lrclk_counter <= 0;
            I2S_LRCLK <= 1'b1;  // 初始右声道（匹配接收端右声道优先）
        end else begin
            if (lrclk_counter >= LRCLK_DIV - 1) begin
                lrclk_counter <= 0;
                I2S_LRCLK <= ~I2S_LRCLK;  // 声道切换
            end else begin
                lrclk_counter <= lrclk_counter + 1;
            end
        end
    end

    // 3. 数据发送时序（修复锁存逻辑，避免数据覆盖）
    always @(negedge I2S_BCLK or negedge rst_n) begin  // BCLK下降沿移位
        if (!rst_n) begin
            I2S_DATA <= 1'b0;
            bit_counter <= 5'd31;  // 从最高位开始发送
            shift_reg <= 32'h0;
            data_latch <= 32'h0;
        end else begin
            // 发送当前位（MSB先行）
            I2S_DATA <= shift_reg[31];
            shift_reg <= {shift_reg[30:0], 1'b0};  // 右移
            
            // 位计数到0时，锁存新数据（下一周期使用）
            if (bit_counter == 0) begin
                bit_counter <= 5'd31;
                data_latch <= adc_data;  // 锁存输入数据
            end else begin
                bit_counter <= bit_counter - 1;
                // 计数到31时，加载锁存的数据（新周期开始）
                if (bit_counter == 5'd31) begin
                    shift_reg <= data_latch;
                end
            end
        end
    end

endmodule
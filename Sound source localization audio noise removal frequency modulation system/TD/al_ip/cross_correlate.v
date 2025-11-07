module cross_correlate(
    input clk,
    input rst_n,
    input [31:0] mic1_data,     // 32位输入数据（保持位宽一致）
    input [31:0] mic2_data,
    input data_valid,           // 数据有效标志
    output reg [63:0] max_corr, // 扩大位宽至64位，避免溢出
    output reg [7:0] tdoa_cnt   // 最大值对应的时延（0~127）
);

// 数据缓存：存储mic2的历史数据（深度128，位宽与输入一致为32位）
reg [31:0] mic2_buf [0:127];
reg [7:0] buf_addr;            // 缓存地址指针（循环写入）

// 互相关计算变量
reg [7:0] corr_cnt;            // 时延索引（0~127）
reg [63:0] corr_sum [0:127];   // 每个时延对应的累加和（64位防溢出）
reg calculating;               // 计算状态标志（1:正在计算所有时延）
reg update_flag;               // 计算完成标志（用于更新最大值）

// 初始化缓存和累加器
integer init_idx;
initial begin
    for(init_idx = 0; init_idx < 128; init_idx = init_idx + 1) begin
        mic2_buf[init_idx] = 32'd0;
        corr_sum[init_idx] = 64'd0;
    end
end

// 1. 缓存mic2数据（循环覆盖旧数据）
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        buf_addr <= 8'd0;
    end else if(data_valid) begin
        mic2_buf[buf_addr] <= mic2_data;  // 32位数据完整存储
        buf_addr <= (buf_addr == 8'd127) ? 8'd0 : buf_addr + 8'd1;
    end
end

// 2. 控制互相关计算流程：数据有效时启动，遍历所有时延
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        calculating <= 1'b0;
        corr_cnt <= 8'd0;
        update_flag <= 1'b0;
    end else if(data_valid) begin
        // 新数据到来，启动计算，从τ=0开始
        calculating <= 1'b1;
        corr_cnt <= 8'd0;
        update_flag <= 1'b0;
    end else if(calculating) begin
        // 遍历所有时延（0~127）
        if(corr_cnt == 8'd127) begin
            calculating <= 1'b0;  // 所有时延计算完成
            update_flag <= 1'b1;  // 触发最大值更新
        end else begin
            corr_cnt <= corr_cnt + 8'd1;
            update_flag <= 1'b0;
        end
    end else begin
        update_flag <= 1'b0;
    end
end

// 3. 计算每个时延的互相关累加和：corr_sum[τ] += mic1(t) × mic2(t-τ)
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        for(init_idx = 0; init_idx < 128; init_idx = init_idx + 1) begin
            corr_sum[init_idx] <= 64'd0;
        end
    end else if(calculating) begin
        // 对当前时延τ=corr_cnt，累加乘积（32位×32位=64位）
        corr_sum[corr_cnt] <= corr_sum[corr_cnt] + 
                            ($signed(mic1_data) * $signed(mic2_buf[corr_cnt]));
    end else if(update_flag) begin
        // 一次完整计算后清零累加器，准备下一轮（可选，根据需求保留）
        for(init_idx = 0; init_idx < 128; init_idx = init_idx + 1) begin
            corr_sum[init_idx] <= 64'd0;
        end
    end
end

// 4. 计算完成后更新最大值和对应的时延
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        max_corr <= 64'd0;
        tdoa_cnt <= 8'd0;
    end else if(update_flag) begin
        // 遍历所有时延的互相关结果，找到最大值
        for(init_idx = 0; init_idx < 128; init_idx = init_idx + 1) begin
            if(corr_sum[init_idx] > max_corr) begin
                max_corr <= corr_sum[init_idx];
                tdoa_cnt <= init_idx[7:0];  // 记录对应时延
            end
        end
    end
end

endmodule
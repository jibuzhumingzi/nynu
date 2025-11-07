module audio_speak(
    input           sys_clk      ,    // 共用系统时钟（如50MHz）
    
    input           sys_rst_n    ,    // 复位（低电平有效）
    input           sys_rst_n_1  ,    // 复位（低电平有效，独立控制）
    input           sys_rst_n_2  ,    // 复位（低电平有效，独立控制）
    input           sys_rst_n_3  ,    // 复位（低电平有效，独立控制）
    

    input   [1:0]   volume       ,    // 音量配置（可共用/volume_1）
    
    //  ES8388 接口
    input           aud_bclk     ,    // I2S位时钟
    input           aud_lrc      ,    // I2S声道对齐信号
    input           aud_adcdat   ,    // ADC音频输入
    output          aud_mclk     ,    // ES8388主时钟（PLL生成）
    output          aud_dacdat   ,    // DAC音频输出
    output          aud_scl      ,    // I2C时钟（配置ES8388）
    inout           aud_sda      ,    // I2C数据（配置ES8388）
    output          ilaclk       ,    // 内部音频时钟（PLL生成）
    
    //  ES8388 接口（共用sys_clk）
    input           aud_bclk_1   ,    // I2S位时钟
    input           aud_lrc_1    ,    // I2S声道对齐信号
    input           aud_adcdat_1 ,    // ADC音频输入
    output          aud_mclk_1   ,    // ES8388主时钟（PLL生成）
    output          aud_dacdat_1 ,    // DAC音频输出
    output          aud_scl_1    ,    // I2C时钟（配置ES8388）
    inout           aud_sda_1    ,    // I2C数据（配置ES8388）
    output          ilaclk_1     ,   // 内部音频时钟（PLL生成）
    
    input           aud_bclk_2   ,    
    input           aud_lrc_2    ,    
    input           aud_adcdat_2 ,    
    output          aud_mclk_2   ,    
    output          aud_dacdat_2 ,    
    output          aud_scl_2    ,    
    inout           aud_sda_2    ,    
    output          ilaclk_2     ,    
    
    input           aud_bclk_3   ,   
    input           aud_lrc_3    ,   
    input           aud_adcdat_3 ,   
    output          aud_mclk_3   ,   
    output          aud_dacdat_3 ,   
    output          aud_scl_3    ,   
    inout           aud_sda_3    ,   
    output          ilaclk_3     ,   
    
    output          led_one	     ,    
    output          led_two      ,    
    output          led_tree     ,
    output          led_four     ,
    output          led_five     ,    
    output          led_six      ,
    output          led_seven	 ,
    output          led_eight	 ,
    
    
    input   [2:0]   pitch_control,   
    input           noise_suppress_en,
    input  			denoise_en,
    input   [1:0]		pitch_en
);

// 信号声明（双路独立，避免冲突）
wire [31:0] adc_data, adc_data_1,adc_data_2,adc_data_3;    // 双路ADC并行数据
wire locked, locked_1,locked_2,locked_3;               // 双路PLL锁定信号
wire chipwatcherclk, chipwatcherclk_1,chipwatcherclk_2,chipwatcherclk_3;// 双路PLL辅助输出时钟
reg [7:0] rst_cnt=0, rst_cnt_1=0,rst_cnt_2=0,rst_cnt_3=0;    // 双路复位延时计数器（避免上电不稳定）


wire [31:0] processed_audio;
wire [31:0] processed_audio_1;
wire [31:0] noise_filtered_audio;
wire [31:0] pitch_shifted_audio;
wire process_data_valid;

wire [31:0] denoised_audio;

//*****************************************************
//** 共用sys_clk：双路复位延时计数（确保上电时序稳定）
//*****************************************************
// 第一路复位延时：sys_clk上升沿计数，8位计数器满（256个时钟周期）后释放复位
always @(posedge sys_clk) begin
    if (!sys_rst_n) begin  // 系统复位时清零
        rst_cnt <= 8'd0;
    end else if (rst_cnt[7]) begin  // 最高位为1表示计数满（256周期），保持
        rst_cnt <= rst_cnt;
    end else begin  // 未计数满则自增
        rst_cnt <= rst_cnt + 1'b1;
    end
end

// 第二路复位延时：与第一路逻辑一致，独立计数（适配第二路复位需求）
always @(posedge sys_clk) begin
    if (!sys_rst_n_1) begin  
        rst_cnt_1 <= 8'd0;
    end else if (rst_cnt_1[7]) begin 
        rst_cnt_1 <= rst_cnt_1;
    end else begin  
        rst_cnt_1 <= rst_cnt_1 + 1'b1;
    end
end


always @(posedge sys_clk) begin
    if (!sys_rst_n_2) begin 
        rst_cnt_2 <= 8'd0;
    end else if (rst_cnt_2[7]) begin  
        rst_cnt_2 <= rst_cnt_2;
    end else begin 
        rst_cnt_2 <= rst_cnt_2 + 1'b1;
    end
end

always @(posedge sys_clk) begin
    if (!sys_rst_n_3) begin  
        rst_cnt_3 <= 8'd0;
    end else if (rst_cnt_3[7]) begin  
        rst_cnt_3 <= rst_cnt_3;
    end else begin  
        rst_cnt_3 <= rst_cnt_3 + 1'b1;
    end
end


//*****************************************************
//** 共用sys_clk：双路PLL实例化（生成ES8388所需时钟）
//*****************************************************
// 第一路PLL：输入sys_clk，输出aud_mclk（ES8388主时钟）和ilaclk（内部音频时钟）
clk_wiz_0 u_pll_clk (
    .refclk     (sys_clk),         
    .reset      (!rst_cnt[7]),     
    .stdby      (1'b0),            
    .extlock    (locked),          
    .clk0_out   (chipwatcherclk),  
    .clk1_out   (aud_mclk)         
);
assign ilaclk = chipwatcherclk;  
// 第二路PLL：同样输入sys_clk，独立生成第二路时钟（避免与第一路干扰）
clk_wiz_0 u1_pll_clk (
    .refclk     (sys_clk),          
    .reset      (!rst_cnt_1[7]),    
    .stdby      (1'b0),             
    .extlock    (locked_1),         
    .clk0_out   (chipwatcherclk_1), 
    .clk1_out   (aud_mclk_1)        
);
assign ilaclk_1 = chipwatcherclk_1;  


clk_wiz_0 u2_pll_clk (
    .refclk     (sys_clk),          
    .reset      (!rst_cnt_2[7]),    
    .stdby      (1'b0),             
    .extlock    (locked_2),         
    .clk0_out   (chipwatcherclk_2), 
    .clk1_out   (aud_mclk_2)        
);
assign ilaclk_2 = chipwatcherclk_2;  

clk_wiz_0 u3_pll_clk (
    .refclk     (sys_clk),         
    .reset      (!rst_cnt_3[7]),   
    .stdby      (1'b0),            
    .extlock    (locked_3),        
    .clk0_out   (chipwatcherclk_3),
    .clk1_out   (aud_mclk_3)       
);
assign ilaclk_3 = chipwatcherclk_3;  

//*****************************************************
//** 共用sys_clk：双路ES8388控制（配置+音频收发）
//*****************************************************
// 第一路ES8388控制：配置、ADC接收、DAC发送
es8388_ctrl u_es8388_ctrl(
    .clk        (sys_clk),        
    .rst_n      (locked),         
    .aud_bclk   (aud_bclk),       
    .aud_lrc    (aud_lrc),        
    .aud_adcdat (aud_adcdat),     
    .aud_dacdat (aud_dacdat),     
    .aud_scl    (aud_scl),        
    .aud_sda    (aud_sda),        
    .volume     (2'b11),          
    .adc_data   (adc_data),       
    .dac_data   (processed_audio),        
    .rx_done    (),                
    .tx_done    ()                 
);

// 第二路ES8388控制：与第一路逻辑一致，独立控制
es8388_ctrl u1_es8388_ctrl(
    .clk        (sys_clk),          
    .rst_n      (locked_1),         
    .aud_bclk   (aud_bclk_1),       
    .aud_lrc    (aud_lrc_1),        
    .aud_adcdat (aud_adcdat_1),     
    .aud_dacdat (aud_dacdat_1),     
    .aud_scl    (aud_scl_1),        
    .aud_sda    (aud_sda_1),        
    .volume     (2'b11),           
    .adc_data   (adc_data_1),       
    .dac_data   (processed_audio_1),      
    .rx_done    (),                
    .tx_done    ()                 
);


es8388_ctrl u2_es8388_ctrl(
    .clk        (sys_clk),         
    .rst_n      (locked_2),        
    .aud_bclk   (aud_bclk_2),      
    .aud_lrc    (aud_lrc_2),       
    .aud_adcdat (aud_adcdat_2),    
    .aud_dacdat (aud_dacdat_2),    
    .aud_scl    (aud_scl_2),       
    .aud_sda    (aud_sda_2),       
    .volume     (2'b11),           
    .adc_data   (adc_data_2),       
    .dac_data   (processed_audio_1),       
    .rx_done    (),                 
    .tx_done    ()                  
);

es8388_ctrl u3_es8388_ctrl(
    .clk        (sys_clk),          
    .rst_n      (locked_3),         
    .aud_bclk   (aud_bclk_3),       
    .aud_lrc    (aud_lrc_3),        
    .aud_adcdat (aud_adcdat_3),     
    .aud_dacdat (aud_dacdat_3),     
    .aud_scl    (aud_scl_3),        
    .aud_sda    (aud_sda_3),        
    .volume     (2'b11),           
    .adc_data   (adc_data_3),       
    .dac_data   (adc_data_3),       
    .rx_done    (),                 
    .tx_done    ()                  
);

spar_4mic_localization u_spar_4mic_localization (
    .clk			(sys_clk),            // 系统时钟（如50MHz）
    .rst_n			(sys_rst_n),          // 复位信号（低有效）
    .mic1_data		(adc_data),      // 麦克风1的24位数字信号
    .mic2_data		(adc_data_1),      // 麦克风2的24位数字信号
    .mic3_data		(adc_data_2),      // 麦克风3的24位数字信号
    .mic4_data		(adc_data_3),
    .data_valid		(1'b1),     // 数据有效信号
    .led1			(led_one),           // 对应麦克风1
    .led2			(led_two),           // 对应麦克风2
    .led3           (led_tree),// 对应麦克风3
    .led4           (led_four),// 对应麦克风4
    
    .led12			(led_five),
	.led23			(led_six),
	.led34			(led_seven),
	.led41			(led_eight)
    			
);

simple_denoise u_simple_denoise(
    .clk            (sys_clk),
    .rst_n          (locked & locked_1),
    .primary_data   (adc_data),        // 第一路作为主麦克风
    .reference_data (adc_data_1),      // 第二路作为参考麦克风  
    .denoised_data  (denoised_audio)   // 去噪后的输出
);


// 噪声抑制模块
noise_suppress #(
    .DATA_WIDTH(32),
    .THRESHOLD(32'h0000_0FFF)  // 噪声阈值
) u_noise_suppress(
    .clk        (sys_clk),
    .rst_n      (locked),
    .audio_in   (adc_data),
    .audio_out  (noise_filtered_audio)
);

// 变调模块
pitch_shift #(
    .DATA_WIDTH(32)
) u_pitch_shift(
    .clk          (sys_clk),
    .rst_n        (locked),
    .audio_in     (noise_filtered_audio ),
    .pitch_factor (pitch_control),
    .audio_out    (pitch_shifted_audio),
    .data_valid   (process_data_valid)
);

assign processed_audio = (pitch_en == 2'b10) ? pitch_shifted_audio :  
                         (pitch_en == 2'b11) ?  adc_data :  32'b0; 
                        
assign processed_audio_1 = (denoise_en == 1'b1) ? denoised_audio : 32'b0;
                       
endmodule
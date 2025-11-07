module sound_localization_comparison(
    input               clk,
    input               rst_n,
    input [31:0]        adc_data1,
    input [31:0]        adc_data2,
    output reg [2:0]    led
);

reg [10:0] clk_div_cnt;
reg sample_clk;
reg [31:0] peak1 = 0, peak2 = 0;
reg [15:0] sample_cnt = 0;

parameter MIN_PEAK_THRESH = 32'd16500000; 
 
always @(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        clk_div_cnt <= 0;
        sample_clk <= 0;
    end else begin
        if(clk_div_cnt >= 11'd1000) begin
            clk_div_cnt <= 0;
            sample_clk <= ~sample_clk;
        end else begin
            clk_div_cnt <= clk_div_cnt + 1;
        end
    end
end

always @(posedge sample_clk or negedge rst_n) begin
    if(!rst_n) begin
        peak1 <= 0;
        peak2 <= 0;
        sample_cnt <= 0;
        led <= 3'b010;  
    end else begin
        sample_cnt <= sample_cnt + 1;
        
        if(adc_data1 > peak1) peak1 <= adc_data1;
        if(adc_data2 > peak2) peak2 <= adc_data2;
        
        if(sample_cnt == 16'd20000) begin
            if(peak1 > MIN_PEAK_THRESH || peak2 > MIN_PEAK_THRESH) begin
                if(peak1 > peak2) begin
                    led <= 3'b001;  
                end else if(peak2 > peak1) begin
                    led <= 3'b100;  
                end else begin
                    led <= 3'b010;  
                end
            end else begin
                led <= 3'b010;  
            end
            
            peak1 <= 0;
            peak2 <= 0;
            sample_cnt <= 0;
        end
    end
end

endmodule
# PGL50H6FB484 约束文件
# OV5640 + 视觉处理系统
# 请根据实际板子原理图核对引脚编号

# ============================================================
# 系统时钟（50MHz 晶振）
# ============================================================
create_clock -period 20.000 [get_ports sys_clk]
set_input_delay  2.0 -clock sys_clk [get_ports sys_rst_n]

# ============================================================
# OV5640 DVP 接口（pclk 最高约 96MHz @ 1080p, 640x480时约42MHz）
# ============================================================
create_clock -period 23.810 [get_ports cam_pclk]  ;# ~42 MHz

set_input_delay  3.0 -clock cam_pclk [get_ports cam_href]
set_input_delay  3.0 -clock cam_pclk [get_ports cam_vsync]
set_input_delay  3.0 -clock cam_pclk [get_ports {cam_data[*]}]

# ============================================================
# UART 输出（异步输出，放宽约束）
# ============================================================
set_output_delay 5.0 -clock sys_clk [get_ports uart_tx]

# ============================================================
# LED 输出
# ============================================================
set_output_delay 5.0 -clock sys_clk [get_ports {led[*]}]

# ============================================================
# OV5640 控制信号
# ============================================================
set_output_delay 5.0 -clock sys_clk [get_ports cam_pwdn]
set_output_delay 5.0 -clock sys_clk [get_ports cam_rst_n]

# ============================================================
# 跨时钟域声明（sys_clk ↔ cam_pclk）
# ============================================================
set_clock_groups -asynchronous \
    -group [get_clocks sys_clk] \
    -group [get_clocks cam_pclk]

# ============================================================
# 物理引脚分配（请根据原理图修改！下面仅为示例占位）
# ============================================================
# set_location_assignment PIN_E1  -to sys_clk
# set_location_assignment PIN_M1  -to sys_rst_n
# set_location_assignment PIN_F14 -to cam_pclk
# set_location_assignment PIN_G14 -to cam_href
# set_location_assignment PIN_H14 -to cam_vsync
# set_location_assignment PIN_A14 -to cam_data[0]
# set_location_assignment PIN_B14 -to cam_data[1]
# set_location_assignment PIN_C14 -to cam_data[2]
# set_location_assignment PIN_D14 -to cam_data[3]
# set_location_assignment PIN_A15 -to cam_data[4]
# set_location_assignment PIN_B15 -to cam_data[5]
# set_location_assignment PIN_C15 -to cam_data[6]
# set_location_assignment PIN_D15 -to cam_data[7]
# set_location_assignment PIN_K2  -to uart_tx
# set_location_assignment PIN_L1  -to led[0]
# set_location_assignment PIN_L2  -to led[1]
# set_location_assignment PIN_M2  -to led[2]
# set_location_assignment PIN_N1  -to cam_pwdn
# set_location_assignment PIN_N2  -to cam_rst_n

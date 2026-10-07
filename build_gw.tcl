#=============================================================================
# build_gw.tcl —— Gowin 命令行构建脚本（综合 + 布局布线 + 生成码流）
# 用法：在本目录下执行
#   "E:\Gowin\Gowin_V1.9.10.02_x64\IDE\bin\gw_sh.exe" build_gw.tcl
#
# 说明 1：本工程必须放在纯 ASCII 路径下（当前 E:\fpga_pd\）。
#         Gowin 1.9.10.02 在中文路径下综合会报 "ERROR (SP0002) Corrupted project file"：
#         它生成的 impl\gwsynthesis\pendulum_fpga.prj 里写的是中文路径，
#         GowinSynthesis 自己再读就失败。原位置 E:\嵌赛\fpga版 现已改为
#         指向 E:\fpga_pd 的目录联接（实体是同一份文件）。
#
# 说明 2：LED 只用 4 个板载脚（R9/R7/N6/P7），不占用 SSPI 专用脚（C10/T10），
#         因此不需要 -use_sspi_as_gpio，**保留外部 Flash(MSPI) 上电自启动能力**。
#         若以后想用满 6 个板载 LED，需要加下面这行，代价是不能再用 Flash 启动：
#             set_option -use_sspi_as_gpio 1
#
set_device -name GW2A-18C GW2A-LV18PG256C8/I7

set_option -top_module top
set_option -output_base_name pendulum_fpga

add_file src/clk_tick.v
add_file src/adc_if.v
add_file src/encoder_if.v
add_file src/key_ctrl.v
add_file src/pid_angle.v
add_file src/pid_pos.v
add_file src/ctrl_fsm.v
add_file src/motor_pwm.v
add_file src/uart_dbg.v
add_file src/led_ind.v
add_file src/oled_ssd1306.v
add_file src/top.v

add_file src/pendulum_fpga.cst
add_file src/pendulum_fpga.sdc

run all

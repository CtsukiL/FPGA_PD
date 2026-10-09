#=============================================================================
# run_angle.do —— 角度环开环测试（命令行）
# 用法：
#   cd /d E:\fpga_pd\sim
#   "E:\modelsim\win64\vsim.exe" -c -do run_angle.do
#=============================================================================

if {[file exists work]} { vdel -all -lib work }
vlib work
vmap work work

vlog -work work +incdir+.. ../src/clk_tick.v
vlog -work work +incdir+.. ../src/adc_if.v
vlog -work work +incdir+.. ../src/encoder_if.v
vlog -work work +incdir+.. ../src/key_ctrl.v
vlog -work work +incdir+.. ../src/pid_angle.v
vlog -work work +incdir+.. ../src/pid_pos.v
vlog -work work +incdir+.. ../src/ctrl_fsm.v
vlog -work work +incdir+.. ../src/motor_pwm.v
vlog -work work +incdir+.. ../src/uart_dbg.v
vlog -work work +incdir+.. ../src/led_ind.v
vlog -work work +incdir+.. ../src/oled_ssd1306.v
vlog -work work +incdir+.. ../src/top.v

vlog -work work +incdir+.. -sv tb_angle_map.v

vsim -c -voptargs="+acc" -do "run -all; quit -f" work.tb_angle_map

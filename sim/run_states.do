#=============================================================================
# run_states.do —— 各状态电机输出模拟（命令行）
# 用法：
#   cd /d E:\fpga_pd\sim
#   "E:\modelsim\win64\vsim.exe" -c -do run_states.do
#=============================================================================

if {[file exists work]} { vdel -all -lib work }
vlib work
vmap work work

vlog -work work ../src/clk_tick.v
vlog -work work ../src/adc_if.v
vlog -work work ../src/encoder_if.v
vlog -work work ../src/key_ctrl.v
vlog -work work ../src/pid_angle.v
vlog -work work ../src/pid_pos.v
vlog -work work ../src/ctrl_fsm.v
vlog -work work ../src/motor_pwm.v
vlog -work work ../src/uart_dbg.v
vlog -work work ../src/led_ind.v
vlog -work work ../src/oled_ssd1306.v
vlog -work work ../src/top.v

vlog -work work -sv tb_states.v

vsim -c -voptargs="+acc" -do "run -all; quit -f" work.tb_states

#=============================================================================
# run_states_gui.do —— 各状态电机输出的波形（GUI）
# 用法：
#   cd /d E:\fpga_pd\sim
#   "E:\modelsim\win64\vsim.exe" -gui -do run_states_gui.do
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
vlog -work work +incdir+.. -sv tb_states.v

vsim -gui -onfinish stop -voptargs="+acc" work.tb_states

add wave -divider "PHYSICAL TIME (ms)  <- 波形上的 1ms 是它 +1"
add wave -radix unsigned /tb_states/phys_ms

add wave -divider "SENSOR / SETPOINT"
add wave -radix hex     /tb_states/adc_d
add wave -radix unsigned /tb_states/u_top/angle
add wave -radix unsigned /tb_states/u_top/center_angle
add wave -radix decimal /tb_states/u_top/location

add wave -divider "FSM"
add wave -radix unsigned /tb_states/u_top/run_state
add wave -radix decimal /tb_states/u_top/u_ctrl_fsm/ang_target

add wave -divider "PID"
add wave -radix decimal /tb_states/u_top/angle_out
add wave -radix decimal /tb_states/u_top/pos_out

add wave -divider "MOTOR (what the driver gets)"
add wave -radix decimal /tb_states/u_top/motor_cmd
add wave /tb_states/motor_in1
add wave /tb_states/motor_in2
add wave /tb_states/motor_pwm

add wave -divider "KEY / LED"
add wave -radix binary /tb_states/key
add wave -radix binary /tb_states/led

configure wave -namecolwidth 210
configure wave -valuecolwidth 90
configure wave -timelineunits ms
wave zoom full

run -all

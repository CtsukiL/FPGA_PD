#=============================================================================
# run_gui.do —— ModelSim GUI 仿真脚本（看波形用）
# 用法：
#   cd /d E:\fpga_pd\sim
#   "E:\modelsim\win64\vsim.exe" -gui -do run_gui.do
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
vlog -work work +incdir+.. -sv tb_top.v

vsim -gui -onfinish stop -voptargs="+acc" work.tb_top

# ---- 波形分组 ----
add wave -divider "PHYSICS (virtual plant)"
add wave -radix decimal /tb_top/alpha
add wave -radix decimal /tb_top/alpha_d
add wave -radix decimal /tb_top/theta
add wave -radix decimal /tb_top/theta_d

add wave -divider "SENSORS"
add wave -radix unsigned /tb_top/u_top/angle
add wave -radix unsigned /tb_top/u_top/center_angle
add wave -radix hex /tb_top/adc_d
add wave -radix decimal /tb_top/u_top/location
add wave -radix binary /tb_top/enc_a
add wave -radix binary /tb_top/enc_b

add wave -divider "CONTROL"
add wave -radix unsigned /tb_top/u_top/run_state
add wave -radix decimal /tb_top/u_top/angle_out
add wave -radix decimal /tb_top/u_top/pos_out
add wave -radix decimal /tb_top/u_top/pos_target
add wave -radix decimal /tb_top/u_top/motor_cmd

add wave -divider "ACTUATOR / UI"
add wave /tb_top/motor_in1
add wave /tb_top/motor_in2
add wave /tb_top/motor_pwm
add wave -radix binary /tb_top/key
add wave -radix binary /tb_top/led
add wave /tb_top/uart_tx

# 波形窗口整理
configure wave -namecolwidth 200
configure wave -valuecolwidth 90
configure wave -timelineunits ms
wave zoom full

run -all

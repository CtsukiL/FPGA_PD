#=============================================================================
# run_sim.do —— ModelSim 命令行仿真脚本（快速迭代用）
# 用法：
#   cd /d E:\fpga_pd\sim
#   "E:\modelsim\win64\vsim.exe" -c -do run_sim.do
# 说明：在 sim 目录下运行，源文件用相对路径 ../src/...
#=============================================================================

# 清掉旧库
if {[file exists work]} { vdel -all -lib work }
vlib work
vmap work work

# 编译 RTL（纯 Verilog）
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

# 编译测试平台（-sv 以支持 $sin/$cos 等数学函数）
vlog -work work +incdir+.. -sv tb_top.v

# 运行（+acc 保留内部信号可见性，方便调试）
vsim -c -voptargs="+acc" -do "run -all; quit -f" work.tb_top

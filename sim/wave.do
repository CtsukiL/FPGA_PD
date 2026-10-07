onerror {resume}
quietly WaveActivateNextPane {} 0
add wave -noupdate -divider {PHYSICAL TIME (ms)  <- 波形上的 1ms 是它 +1}
add wave -noupdate -radix unsigned /tb_states/phys_ms
add wave -noupdate -divider {SENSOR / SETPOINT}
add wave -noupdate -radix hexadecimal /tb_states/adc_d
add wave -noupdate -radix unsigned /tb_states/u_top/angle
add wave -noupdate -radix unsigned /tb_states/u_top/center_angle
add wave -noupdate -radix decimal /tb_states/u_top/location
add wave -noupdate -divider FSM
add wave -noupdate -radix unsigned /tb_states/u_top/run_state
add wave -noupdate -radix decimal /tb_states/u_top/u_ctrl_fsm/ang_target
add wave -noupdate -divider PID
add wave -noupdate -radix decimal /tb_states/u_top/angle_out
add wave -noupdate -radix decimal /tb_states/u_top/pos_out
add wave -noupdate -divider {MOTOR (what the driver gets)}
add wave -noupdate -radix decimal /tb_states/u_top/motor_cmd
add wave -noupdate /tb_states/motor_in1
add wave -noupdate /tb_states/motor_in2
add wave -noupdate /tb_states/motor_pwm
add wave -noupdate -divider {KEY / LED}
add wave -noupdate -radix binary /tb_states/key
add wave -noupdate -radix binary /tb_states/led
TreeUpdate [SetDefaultTree]
WaveRestoreCursors {{Cursor 1} {0 ps} 0}
quietly wave cursor active 0
configure wave -namecolwidth 210
configure wave -valuecolwidth 90
configure wave -justifyvalue left
configure wave -signalnamewidth 0
configure wave -snapdistance 10
configure wave -datasetprefix 0
configure wave -rowmargin 4
configure wave -childrowmargin 2
configure wave -gridoffset 0
configure wave -gridperiod 1
configure wave -griddelta 40
configure wave -timeline 0
configure wave -timelineunits ms
update
WaveRestoreZoom {0 ps} {127238406144 ps}

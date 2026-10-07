//=============================================================================
// pendulum_fpga.sdc —— 时序约束
// 系统只有一个 50MHz 时钟源（板载晶振 T7）：
//   50MHz -> 周期 20ns，占空比 50%（0~10ns 高）
// 加上这条约束后，PnR 的时序分析才有比较基准，也消除
// "sys_clk was determined to be a clock but was not created" 告警。
//=============================================================================

create_clock -name sys_clk -period 20 -waveform {0 10} [get_ports {sys_clk}]

//============================================================================
// led_ind.v —— 板载 LED 状态指示（4 个，全部低电平点亮）
// 对应 STM32 版：LED.c（PC13 指示运行状态）
// 说明：G1 板载 LED 有 6 个，但 led 的 C10、T10 两个脚是 SSPI 专用引脚，
//       占用后就不能再用外部 Flash（MSPI）配置启动，所以本工程只用另外 4 个：
//   led[0] (R9) 运行中常亮（对应 STM32 的 PC13）
//   led[1] (R7) 平衡（PID，状态 4）常亮；起摆阶段（21~34）慢闪
//   led[2] (N6) 完成一次标定后亮 0.5s
//   led[3] (P7) 心跳（约 1.34s 周期）；ADC 超量程 OTR 时加快到约 0.34s
//============================================================================
module led_ind(
    input  wire       clk,           // 50MHz 系统时钟
    input  wire       rst_n,         // 低电平复位
    input  wire [5:0] run_state,     // 运行状态
    input  wire       calib_pulse,   // 标定脉冲（单周期）
    input  wire       adc_otr,       // ADC 超量程
    output reg  [3:0] led            // 低电平点亮
);

    //----------------- 心跳基准 ----------------
    reg [24:0] hb_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) hb_cnt <= 25'd0;
        else        hb_cnt <= hb_cnt + 25'd1;
    end

    //----------------- 标定指示灯（亮 0.5s）-----------------
    reg [24:0] calib_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)               calib_cnt <= 25'd0;
        else if (calib_pulse)     calib_cnt <= 25'd25_000_000;
        else if (calib_cnt != 0)  calib_cnt <= calib_cnt - 25'd1;
    end

    wire running  = (run_state != 6'd0);
    wire swinging = (run_state >= 6'd21) && (run_state <= 6'd34);
    wire balance  = (run_state == 6'd4);
    wire calib_on = (calib_cnt != 25'd0);

    // 平衡常亮 / 起摆慢闪（约 0.67s 半周期）
    wire bal_or_swing = balance || (swinging && hb_cnt[23]);
    // 心跳：正常约 0.67s 翻转一次；OTR 时加快到约 0.17s，兼作超量程告警
    wire hb = adc_otr ? hb_cnt[22] : hb_cnt[24];

    always @(*) begin
        led[0] = ~running;        // 运行中常亮
        led[1] = ~bal_or_swing;   // 平衡常亮 / 起摆慢闪
        led[2] = ~calib_on;       // 标定完成亮 0.5s
        led[3] = ~hb;             // 心跳（OTR 时加快）
    end

endmodule

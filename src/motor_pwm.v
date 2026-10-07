//============================================================================
// motor_pwm.v —— 电机 PWM + 方向（TB6612FNG）
// 对应 STM32 版：Motor.c 的 Motor_SetPWM() + PWM.c（TIM2_CH1 20kHz）
//   - PWM 频率 20kHz（50MHz / 2500 = 20kHz，与 STM32 版一致）
//   - 输入指令 -100 ~ +100，对应 0~100% 占空比
//   - 小信号死区 2【2026-10-06 由 5 降到 2：5 会吃掉小修正，摆杆小幅倾斜/过冲后恢复很慢】
//   - 方向：正转 IN1=0/IN2=1，反转 IN1=1/IN2=0（与 Motor.c 一致）
// 注意：TB6612 的 STBY 必须由硬件拉高，否则电机不动（与 3PA1030 的 STBY 逻辑相反）
//============================================================================
module motor_pwm(
    input  wire               clk,          // 50MHz 系统时钟
    input  wire               rst_n,        // 低电平复位
    input  wire signed [15:0] cmd,          // 指令 -100 ~ +100
    output reg                in1,          // TB6612 IN1
    output reg                in2,          // TB6612 IN2
    output wire               pwm,          // TB6612 PWMA
    output wire               smp_pulse     // PWM 周期末尾脉冲 -> adc_if（让 AD 采样与 PWM 同相）
);

    localparam [11:0] PWM_PERIOD = 12'd2500;    // 20kHz
    localparam signed [15:0] DEADZONE = 16'sd2; // 【2026-10-06】5 -> 2：小误差时位置环的小输出被死区 5 吃掉 -> 摆杆小幅倾斜要调很久、过冲后恢复慢

    //----------------- PWM 载波 ----------------
    reg [11:0] pwm_cnt;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)                        pwm_cnt <= 12'd0;
        else if (pwm_cnt >= PWM_PERIOD - 1) pwm_cnt <= 12'd0;
        else                               pwm_cnt <= pwm_cnt + 12'd1;
    end

    //----------------- PWM 周期末尾脉冲（每 50us 一个 clk）-----------------
    // 给 adc_if 做同步采样：把 3PA1030 的采样点固定到 PWM 周期的同一相位。
    // 【2026-10-02 起用】原来 ADC 自己分频 125us 采样，与 50us 的 PWM 周期之比是 2.5
    // （非整数），采样点在高/低电平期间之间交替，读数跟着电机转 ±10 count 抖
    // （实测：静止 1~3、电机一转 10、拔掉电机供电恢复 1~3）。角度环 Kd=0.4 会把这种
    // 抖动放大成输出抖动，表现为电机"咔咔"抽搐。
    assign smp_pulse = (pwm_cnt == PWM_PERIOD - 12'd1);

    //----------------- 方向 + 占空比 ----------------
    reg [15:0] duty;    // 0 ~ 2500

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            in1  <= 1'b0;
            in2  <= 1'b0;
            duty <= 16'd0;
        end else if (cmd > -DEADZONE && cmd < DEADZONE) begin
            // 死区：指令太小推不动电机，直接当停止
            in1  <= 1'b0;
            in2  <= 1'b0;
            duty <= 16'd0;
        end else if (cmd >= DEADZONE) begin
            // 正转（与 STM32 Motor.c 的 PB12=0 / PB13=1 对应）
            // 【2026-10-02 极性实验：已回退】曾把 IN1/IN2 对调试过一版，
            //   上板后系统"疯狂抽搐"，不是想要的解法，这里改回原方向。
            in1  <= 1'b0;
            in2  <= 1'b1;
            duty <= cmd * 16'd25;           // 100 * 25 = 2500 = 满占空比
        end else begin
            // 反转
            in1  <= 1'b1;
            in2  <= 1'b0;
            duty <= (-cmd) * 16'd25;
        end
    end

    assign pwm = (pwm_cnt < duty) ? 1'b1 : 1'b0;

endmodule

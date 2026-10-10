//============================================================================
// top.v —— 顶层（逻辑派 FPGA-G1，GW2A-LV18PG256C8/I7，50MHz）
// 对应 STM32 版：main.c 的 main()（模块初始化 + 主循环）
// 模块划分与 STM32 版一一对应：
//   clk_tick   <- Timer.c（1ms 中断）+ main.c 的计次分频（5ms/40ms/50ms）
//   adc_if     <- AD.c（ADC1_IN8，0~4095）+ 4 点滑动平均
//   encoder_if <- Encoder.c（TIM3 编码器模式）+ Location 累加
//   key_ctrl   <- Key.c（20ms 扫描）+ main.c 的 K1~K4 功能
//   ctrl_fsm   <- main.c 的 TIM1_UP_IRQHandler（状态机 + 双环 PID）
//   motor_pwm  <- Motor.c（Motor_SetPWM）+ PWM.c（20kHz）
//   uart_dbg   <- Serial.c（原程序未使用，这里用于调试上报）
//   led_ind    <- LED.c（PC13）
//   oled_ssd1306 <- 江协 OLED 驱动（SSD1306 4 针 I2C）：四行演示显示
//                   CNT/ST、L/V（横杆位置/速度）、DEG/POS（位移角度/目标位置）、ANG/CT
// 复位：G1 板载没有复位键，这里用上电计数器产生内部复位；
//       4 个按键全部外接（key[3:0]），板载 key0/key1 不再使用。
//============================================================================
module top(
    input  wire       sys_clk,      // T7   50MHz 系统时钟
    input  wire [3:0] key,          // 外接 4 个按键（低有效，按下接 GND）：
                                    //   [0]=K1 启停
                                    //   [1]=K2 短按目标 +360 度 / 长按手动点动 +
                                    //   [2]=K3 短按目标 -360 度 / 长按手动点动 -
                                    //   [3]=K4 标定平衡点
    output wire [3:0] led,          // R9/R7/N6/P7 四个板载 LED（低电平点亮）
    output wire       uart_tx,      // F12  串口发送（接板载 USB 串口）
    // input wire     uart_rx,      // F13  串口接收预留：当前不接（无接收功能，接了会报 CV0016）
    output wire       adc_clk,      // C11  -> 3PA1030 CLK（5MHz）
    input  wire [9:0] adc_d,        // 3PA1030 D0~D9（D0=R14 ... D9=D10）
    input  wire       adc_otr,      // C9   3PA1030 超量程指示
    input  wire       enc_a,        // K16  编码器 A 相（H5-1）
    input  wire       enc_b,        // J15  编码器 B 相（H5-2）
    output wire       motor_pwm,    // G15  -> TB6612 PWMA（H6-7）
    output wire       motor_in1,    // G14  -> TB6612 IN1（H6-8）
    output wire       motor_in2,    // G16  -> TB6612 IN2（H6-9）
    output wire       oled_scl,     // J14  -> OLED SCL（H5-3）
    inout  wire       oled_sda      // J16  -> OLED SDA（H5-4，开漏）
);

    //----------------- 上电复位（约 0.65ms 后释放）-----------------
    reg [15:0] rst_cnt = 16'd0;

    always @(posedge sys_clk) begin
        if (!rst_cnt[15]) rst_cnt <= rst_cnt + 16'd1;
    end

    wire rst_n = rst_cnt[15];

    //----------------- 节拍 ----------------
    wire tick_1ms, tick_5ms, tick_40ms, tick_50ms;

    clk_tick u_clk_tick(
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .tick_5ms  (tick_5ms),
        .tick_40ms (tick_40ms),
        .tick_50ms (tick_50ms)
    );

    //----------------- 角度采集（3PA1030 并行 ADC）-----------------
    wire [11:0] angle;
    wire        angle_vld;
    wire [9:0]  raw_code;           // 【调试】ADC 原码 0~1023
    wire [11:0] avg4_dbg;           // 【调试】滑窗均值后的取反码 0~4092
    wire        smp_pulse;          // motor_pwm 的 PWM 周期末尾脉冲（AD 采样同步用）

    adc_if u_adc(
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .smp_pulse (smp_pulse),
        .adc_clk   (adc_clk),
        .adc_d     (adc_d),
        .angle     (angle),
        .angle_vld (angle_vld),
        .raw_code  (raw_code),
        .avg4_out  (avg4_dbg)
    );

    //----------------- 编码器 ----------------
    wire signed [31:0] location;
    wire               enc_zero;        // 来自 ctrl_fsm 的标定脉冲

    encoder_if u_encoder(
        .clk     (sys_clk),
        .rst_n   (rst_n),
        // 编码器 A/B 方向
        //   【2026-10-03 旧电机】曾软件对调（.enc_a(enc_b)）：当时电机转向与角度环都对、
        //     只有编码器计数方向不匹配，位置环成了正反馈（横杆越转越快）。
        //   【2026-10-10 换 JGA25-370】新电机编码器相序与旧电机相反，那处对调要撤掉
        //     （再反一次才是对的），否则 L 的增减方向与 10-03 之前相反。
        //   !! 这一处与下面 u_motor 的 IN1/IN2 是"一对"：必须同时改。
        //      只改一个（例如只反电机方向）会让位置环变成正反馈 -> 横杆飞转。
        .enc_a   (enc_a),
        .enc_b   (enc_b),
        .pos_clr (enc_zero),
        .pos     (location)
    );

    //----------------- 按键（4 个全部外接，一键一功能）-----------------
    // K2/K3 有两路输出：短按（松手脉冲）= 目标 ±360 度走曲线；长按（按住电平）= 手动点动慢移
    wire k_start, k_plus, k_minus, k_calib;
    wire k_jog_plus, k_jog_minus;

    key_ctrl #(.DB_MS (20)) u_key0 (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .key_in    (key[0]),            // K1：启停
        .ev_single (k_start),
        .ev_hold   ()
    );

    key_ctrl #(.DB_MS (20), .HOLD_MS (400)) u_key1 (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .key_in    (key[1]),            // K2：短按目标 +360 度 / 长按点动 +
        .ev_single (k_plus),
        .ev_hold   (k_jog_plus)
    );

    key_ctrl #(.DB_MS (20), .HOLD_MS (400)) u_key2 (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .key_in    (key[2]),            // K3：短按目标 -360 度 / 长按点动 -
        .ev_single (k_minus),
        .ev_hold   (k_jog_minus)
    );

    key_ctrl #(.DB_MS (20)) u_key3 (
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .tick_1ms  (tick_1ms),
        .key_in    (key[3]),            // K4：标定平衡点
        .ev_single (k_calib),
        .ev_hold   ()
    );

    //----------------- 控制状态机 + 双环 PID ----------------
    wire [5:0]         run_state;
    wire signed [15:0] motor_cmd;
    wire signed [15:0] angle_out;
    wire signed [15:0] pos_out;
    wire [11:0]        center_angle;
    wire signed [31:0] pos_target;
    wire signed [31:0] pos_set_dbg;     // 目标位置（给 OLED）
    wire signed [31:0] bar_vel_dbg;     // 横杆速度（给 OLED）
    wire signed [31:0] target_vel_dbg;  // 指令速度 count/s，给串口
    wire               mov_active;      // 【2026-10-04】运动标志（给 motor_pwm 选死区档）

    ctrl_fsm u_ctrl_fsm(
        .clk           (sys_clk),
        .rst_n         (rst_n),
        .tick_1ms      (tick_1ms),
        .tick_5ms      (tick_5ms),
        .tick_40ms     (tick_40ms),
        .tick_50ms     (tick_50ms),
        .angle         (angle),
        .location      (location),
        .ev_start_stop (k_start),       // K1：启停
        .ev_step_plus  (k_plus),        // K2 短按：+360 度
        .ev_step_minus (k_minus),       // K3 短按：-360 度
        .ev_jog_plus   (k_jog_plus),    // K2 长按：点动 +
        .ev_jog_minus  (k_jog_minus),   // K3 长按：点动 -
        .ev_calib      (k_calib),       // K4：标定 + 记录初始位置
        .run_state     (run_state),
        .motor_cmd     (motor_cmd),
        .angle_out     (angle_out),
        .pos_out       (pos_out),
        .center_angle  (center_angle),
        .pos_target    (pos_target),
        .pos_set_out   (pos_set_dbg),   // 给 OLED 显示
        .bar_vel       (bar_vel_dbg),   // 给 OLED 和串口显示
        .target_vel    (target_vel_dbg),
        .mov_active    (mov_active),    // 给 motor_pwm 选死区档
        .enc_zero      (enc_zero)
    );

    //----------------- 电机驱动（TB6612FNG）-----------------
    motor_pwm u_motor(
        .clk       (sys_clk),
        .rst_n     (rst_n),
        .cmd       (motor_cmd),
        // 【2026-10-10 换 JGA25-370】21.3:1 减速箱的输出转向与旧电机相反，这里对调 IN1/IN2
        //   （引脚定义不动，只换内部连接）。与上面 u_encoder 的 A/B 是一对，必须同时改。
        .in1       (motor_in2),
        .in2       (motor_in1),
        .pwm       (motor_pwm),
        .smp_pulse (smp_pulse)      // 给 adc_if 做同步采样
    );

    //----------------- OLED（SSD1306，4 针 I2C）-----------------
    oled_ssd1306 #(
        .CLK_HZ (50000000),
        .SCL_HZ (50000)             // 【示波器观察用】原 400000(约396.8kHz)，现 50kHz，验完改回
    ) u_oled (
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .location     (location),
        .angle        (angle),
        .center_angle (center_angle),
        .pos_set      (pos_set_dbg),    // 目标位置（page4 的 POS）
        .bar_vel      (bar_vel_dbg),    // 横杆速度（page2 的 V）
        .raw_code     (raw_code),       // 【保留未用】AD 原码
        .avg4         (avg4_dbg),       // 【保留未用】滑窗均值码
        .run_state    (run_state),      // 诊断显示
        .scl          (oled_scl),
        .sda          (oled_sda)
    );

    //----------------- 串口调试上报 ----------------
    uart_dbg u_uart_dbg(
        .clk          (sys_clk),
        .rst_n        (rst_n),
        .angle        (angle),
        .location     (location),
        .angle_out    (angle_out),
        .pos_out      (pos_out),
        .run_state    (run_state),
        .center_angle (center_angle),
        .pos_target   (pos_target),
        .pos_set      (pos_set_dbg),
        .bar_vel      (bar_vel_dbg),
        .target_vel   (target_vel_dbg),
        .motor_cmd    (motor_cmd),
        .raw_code     (raw_code),
        .avg4         (avg4_dbg),
        .mov_active   (mov_active),
        .tx           (uart_tx)
    );

    //----------------- LED 指示（4 个板载 LED）-----------------
    led_ind u_led(
        .clk         (sys_clk),
        .rst_n       (rst_n),
        .run_state   (run_state),
        .calib_pulse (k_calib),
        .adc_otr     (adc_otr),
        .led         (led)
    );

    // 说明：
    //   uart_rx 已注释掉（预留不接，避免 CV0016 unused 告警）；
    //   要用时把 top.v 的端口和 cst 里 F13 那两行一起取消注释。
    //   angle_vld / pos_target 只作内部状态观察，未引出。

endmodule

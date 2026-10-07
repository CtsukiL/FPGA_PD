`timescale 1ns/1ps
//============================================================================
// tb_top.v —— 倒立摆 FPGA 工程闭环仿真平台（ModelSim）
//
// 【仿真加速】
//   clk_tick 与 adc_if 的 DIV 都设成 50：
//     1ms 物理时间 = 1000 个 50MHz 周期 = 20us 仿真时间
//   两个模块必须用同一个 DIV，否则"角度更新率(3ms)"与"控制节拍(5ms/50ms)"
//   在仿真时间轴上的比例会错，物理时间轴仍然是真实的 ms。
//
// 【虚拟模型】
//   摆杆   : 简化 Furuta 摆  alpha_dd = (G/L)sin(a) - (Rm/L)*theta_dd*cos(a)
//            摆杆有效长度 L = 10cm（按用户机构）
//   电位器 : adc = 2010 - alpha_deg*11.5，钳位到 [0,4095]
//            11.5 码/度 等价于"安装角度让摆杆全行程(±170°)都落在有效范围内"
//   编码器 : 臂角 theta 的变化 -> AB 正交脉冲（四倍频），细步拍逐步发出
//
// 【电机参数怎么定的】
//   角度环 Kp=0.3（STM32 原值）在 ADC 域给出 Out = 0.3*(659*alpha) ≈ 198*alpha (PWM)
//   要让摆回中，需要臂角加速度 theta_dd = (g/Rm)*alpha = 98*alpha
//   => 满 PWM 的角加速度 KM = 98*100/198 ≈ 50 rad/s^2
//   取 KM_PHY = 50，这样 Kp 既不过校正、也能把摆拉回来
//
// 【激励顺序】
//   上电(摆竖直) -> 按 key[3] 1.1s 标定 -> 摆杆置于下垂 -> 按 key[0] 起摆
//============================================================================
module tb_top;

    //----------------- 仿真参数 ----------------
    localparam integer DIV     = 50;                  // 加速比（与 defparam 一致）
    localparam real    PI      = 3.141592653589793;
    localparam real    RAD2DEG = 180.0 / PI;
    localparam real    CNT_DEG  = 10.0;               // 电位器每度对应的 ADC 码（±180° 全行程都有读数）
    localparam real    AMAX_DEG = 180.0;              // 摆杆机械行程 ±180 度（可真正垂到底）
    localparam real    AMAX_RAD = 180.0 * PI / 180.0;
    localparam real    CNT_RAD  = 408.0 / (2.0 * PI); // 编码器：每弧度 64.9 count

    // ---- 物理参数（按实物估算）----
    //   摆杆 10cm / 旋转臂 10cm，均为 5~6mm 杆（各约 20g）
    //   总转动惯量 J = 臂 mL^2/3 + 摆杆点质量 m*Rm^2 + 轮毂 ≈ 3.7e-4 kg*m^2
    localparam real G_PHY    = 9.8;    // 重力加速度
    localparam real L_PHY    = 0.10;   // 摆杆有效长度 10cm
    localparam real RM_PHY   = 0.10;   // 旋转臂长度 10cm
    localparam real KM_PHY   = 50.0;   // PWM(±1) -> 臂角加速度 rad/s^2（按 Kp=0.3 增益匹配）
    localparam real WMAX_PHY = 65.0;   // 臂空载最高角速度 rad/s（620RPM，与 STM32 同款电机）
    localparam real B_PHY    = 3.0;    // 臂粘性阻尼
    localparam real BPEND_PHY = 0.05;  // 摆杆铰链粘性阻尼
    // ---- 完整 Furuta 模型参数（双向耦合）----
    localparam real MP_PHY  = 0.020;   // 摆杆质量 kg（10cm 杆 + 铰链）
    localparam real LC_PHY  = 0.050;   // 摆杆质心距铰点 m
    localparam real IP_PHY  = 1.67e-5; // 摆杆绕质心转动惯量 kg*m^2
    localparam real JA_PHY  = 8.0e-4;  // 臂+轮毂+减速箱等效惯量 kg*m^2
    localparam real TAU_MAX = 0.15;    // 25GA370 12V 有效力矩 N*m（小一点，让摆幅缓慢增长）
    localparam real BF_PHY  = 0.002;   // 臂粘性摩擦
    localparam real DT_PHY   = 0.001;  // 1ms 物理步长

    localparam real CENTER = 2010.0;   // 平衡点 ADC 值（与 ctrl_fsm 的初值一致）

    // 时间换算：1ms 物理 = 20us 仿真 = 20000 ns
    localparam integer NS_PER_MS = 20000;

    //----------------- 时钟 ----------------
    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;      // 50MHz

    //----------------- DUT ----------------
    reg  [9:0] adc_d   = 10'd0;
    reg        adc_otr = 1'b0;
    reg        enc_a   = 1'b0;
    reg        enc_b   = 1'b0;
    reg  [3:0] key     = 4'b1111;       // 4 个外接按键，低有效
    wire [3:0] led;
    wire       uart_tx;
    wire       adc_clk;
    wire       adc_oe;
    wire       oled_scl;
    wire       oled_sda;
    wire       motor_pwm, motor_in1, motor_in2;

    top u_top(
        .sys_clk   (sys_clk),
        .key       (key),
        .led       (led),
        .uart_tx   (uart_tx),
        .uart_rx   (1'b1),
        .adc_clk   (adc_clk),
        .adc_oe    (adc_oe),
        .adc_d     (adc_d),
        .adc_otr   (adc_otr),
        .enc_a     (enc_a),
        .enc_b     (enc_b),
        .motor_pwm (motor_pwm),
        .motor_in1 (motor_in1),
        .motor_in2 (motor_in2),
        .oled_scl  (oled_scl),
        .oled_sda  (oled_sda)
    );

    defparam u_top.u_clk_tick.DIV = DIV;
    defparam u_top.u_adc.DIV      = DIV;

    //----------------- 内部观察信号 ----------------
    wire [5:0]         run_state  = u_top.run_state;
    wire               tick_1ms   = u_top.tick_1ms;
    wire               tick_50ms  = u_top.tick_50ms;
    wire signed [15:0] motor_cmd  = u_top.motor_cmd;
    wire signed [15:0] angle_out  = u_top.angle_out;
    wire signed [15:0] pos_out    = u_top.pos_out;
    wire [11:0]        angle_dut  = u_top.angle;
    wire [11:0]        center_dut = u_top.center_angle;
    wire signed [31:0] location   = u_top.location;

    //----------------- 物理模型状态 ----------------
    real alpha   = 0.0;     // 摆角(rad)，0 = 竖直向上，正方向与臂旋转正方向一致
    real alpha_d = 0.0;
    real theta   = 0.0;     // 臂角(rad)
    real theta_d = 0.0;

    real    alpha_deg;
    real    adc_f;
    real    u_in;
    real    theta_dd;
    real    alpha_dd;
    real    td_abs;
    real    tau, csa, sna, a11, a12, a22, b1, b2, det;
    integer adc_i;

    real      enc_pos_f  = 0.0;    // 编码器应处位置(count，浮点)
    integer   enc_sent   = 0;      // 已发出的步数
    real      theta_prev = 0.0;
    reg [1:0] ab         = 2'b00;  // {enc_a, enc_b}
    integer   enc_div    = 0;
    reg       hold_rod   = 1'b0;   // 1 = 人扶着摆杆（按 K1 后松手）

    integer phys_ms   = 0;         // 物理时间(ms)
    integer k;                     // 起摆重试循环
    integer tx_starts = 0;         // 串口起始位计数
    integer state_cnt = 0;         // 状态变化次数

    always @(negedge uart_tx) tx_starts = tx_starts + 1;

    //----------------- 摆 + 电位器 + 编码器目标（每虚拟 1ms 更新）-----------------
    always @(posedge tick_1ms) begin
        phys_ms = phys_ms + 1;

        // ---- 完整 Furuta 摆动力学：2x2 联立求解（含摆对臂的反作用力矩）----
        //   [a11 a12][alpha_dd]   [b1]
        //   [a12 a22][theta_dd] = [b2]
        u_in = motor_cmd / 100.0;
        tau  = TAU_MAX * u_in - BF_PHY * theta_d;
        csa  = $cos(alpha);
        sna  = $sin(alpha);
        a11  = MP_PHY*LC_PHY*LC_PHY + IP_PHY;
        a12  = MP_PHY*RM_PHY*LC_PHY*csa;
        a22  = JA_PHY + MP_PHY*RM_PHY*RM_PHY;
        b1   = MP_PHY*G_PHY*LC_PHY*sna - BPEND_PHY*alpha_d*a11;
        b2   = tau + MP_PHY*RM_PHY*LC_PHY*sna*alpha_d*alpha_d;
        det  = a11*a22 - a12*a12;
        alpha_dd = (b1*a22 - a12*b2)/det;
        theta_dd = (a11*b2 - a12*b1)/det;
        theta_d  = theta_d + theta_dd * DT_PHY;
        theta    = theta + theta_d * DT_PHY;
        alpha_d  = alpha_d + alpha_dd * DT_PHY;
        alpha    = alpha + alpha_d * DT_PHY;

        // 角度回绕：摆杆是自由转动的，跨过 ±180° 要绕回来而不是"撞限位"
        // （之前写成限位并把角速度清零，等于每次垂到底都把能量丢掉，摆永远起不来）
        if (alpha >  PI) alpha = alpha - 2.0*PI;
        if (alpha < -PI) alpha = alpha + 2.0*PI;

        // 电位器 -> 12bit ADC 值
        alpha_deg = alpha * RAD2DEG;
        adc_f     = CENTER - alpha_deg * CNT_DEG;
        adc_i     = $rtoi(adc_f + 0.5);
        if (adc_i < 0)    adc_i = 0;
        if (adc_i > 4095) adc_i = 4095;
        adc_d     <= adc_i[11:2];          // 12bit -> 10bit
        adc_otr   <= (adc_f < 0.0) || (adc_f > 4095.0);

        // 编码器目标位置（count）
        enc_pos_f  = enc_pos_f + (theta - theta_prev) * CNT_RAD;
        theta_prev = theta;

        // 扶住阶段：模拟人把摆杆扶在偏离下垂的位置（真实操作就是这样，
        // 按下 K1 后才松手）。否则精确 180° + 上下对称的推力净冲量为零，
        // 摆永远不会动。
        if (hold_rod) begin
            alpha   = 175.0 * PI / 180.0;
            alpha_d = 0.0;
        end
    end

    //----------------- 编码器 AB 正交脉冲（每 100 个时钟周期发一步）-----------------
    always @(posedge sys_clk) begin
        enc_div = enc_div + 1;
        if (enc_div >= 100) begin
            enc_div = 0;
            if (enc_sent < $rtoi(enc_pos_f)) begin
                // 正向一步：00->10->11->01->00
                case (ab)
                    2'b00:   ab = 2'b10;
                    2'b10:   ab = 2'b11;
                    2'b11:   ab = 2'b01;
                    default: ab = 2'b00;
                endcase
                enc_sent = enc_sent + 1;
            end else if (enc_sent > $rtoi(enc_pos_f)) begin
                // 反向一步：00->01->11->10->00
                case (ab)
                    2'b00:   ab = 2'b01;
                    2'b01:   ab = 2'b11;
                    2'b11:   ab = 2'b10;
                    default: ab = 2'b00;
                endcase
                enc_sent = enc_sent - 1;
            end
            enc_a = ab[1];
            enc_b = ab[0];
        end
    end

    //----------------- 观察打印 ----------------
    always @(run_state) begin
        state_cnt = state_cnt + 1;
        $display("[%0d ms] >>> RunState = %0d", phys_ms, run_state);
    end

    // ---- 密集打印：每 20ms 物理一次，用于符号/相位诊断 ----
    integer dbg_cnt = 0;
    wire signed [15:0] ang_target_dut = u_top.u_ctrl_fsm.ang_target;

    always @(posedge tick_1ms) begin
        dbg_cnt = dbg_cnt + 1;
        if (dbg_cnt >= 20) begin
            dbg_cnt = 0;
            if (phys_ms >= 1400)
                $display("[%5d ms] st=%0d ang=%4d tgt=%5d aOut=%5d pOut=%5d cmd=%5d | alpha=%9.4f deg  ad=%9.4f  th=%7.3f thd=%7.3f",
                         phys_ms, run_state, angle_dut, ang_target_dut, angle_out, pos_out, motor_cmd,
                         alpha*RAD2DEG, alpha_d, theta, theta_d);
        end
    end

    always @(posedge tick_50ms) begin
        $display("[%5d ms] st=%2d ang=%4d ctr=%4d loc=%7d pOut=%4d aOut=%4d cmd=%4d | a=%8.2f deg  th=%7.3f rad",
                 phys_ms, run_state, angle_dut, center_dut, location,
                 pos_out, angle_out, motor_cmd, alpha * RAD2DEG, theta);
    end

    //----------------- 激励 ----------------
    initial begin
        $display("==== tb_top start (DIV=%0d, rod %0.0f cm, arm %0.0f cm) ====",
                 DIV, L_PHY*100.0, RM_PHY*100.0);

        alpha = 0.0; alpha_d = 0.0; theta = 0.0; theta_d = 0.0;

        // 等上电复位 + 稳定（50ms 物理）
        #(NS_PER_MS * 50);

        // ---- 1) 按 key[3]（K4）1.1s：标定平衡点 + 记录初始位置 ----
        $display("[%0d ms] >> KEY[3] press: calibrate (rod upright)", phys_ms);
        key[3] = 1'b0;
        #(NS_PER_MS * 1100);
        key[3] = 1'b1;
        #(NS_PER_MS * 100);
        $display("[%0d ms] after calib: center=%0d loc=%0d", phys_ms, center_dut, location);

        // ---- 2) 把摆杆扶到接近下垂（模拟手动摆下；此时标定已完成）----
        hold_rod = 1'b1;
        alpha    = 175.0 * PI / 180.0;
        alpha_d  = 0.0;
        $display("[%0d ms] >> rod held at %0.1f deg", phys_ms, alpha*RAD2DEG);
        #(NS_PER_MS * 300);

        // ---- 2.5) 松手：放开摆杆，准备起摆 ----
        hold_rod = 1'b0;

        // ---- 3) 按 key[0]（K1）：开始自动起摆 ----
        $display("[%0d ms] >> KEY[0] click: start swing-up", phys_ms);
        key[0] = 1'b0;
        #(NS_PER_MS * 40);          // 按下 40ms（>20ms 消抖）
        key[0] = 1'b1;

        // ---- 4) 反复尝试起摆（模拟人工多次按键；只在状态 0 时按）----
        for (k = 0; k < 8; k = k + 1) begin
            if (run_state == 6'd0) begin
                $display("[%0d ms] >> KEY[0] click (attempt %0d)", phys_ms, k+1);
                key[0] = 1'b0;
                #(NS_PER_MS * 40);
                key[0] = 1'b1;
            end
            #(NS_PER_MS * 2500);
        end
        #(NS_PER_MS * 2000);

        $display("==== sim end ====");
        $display("final: state=%0d  alpha=%0.2f deg  theta=%0.2f rad  loc=%0d",
                 run_state, alpha*RAD2DEG, theta, location);
        $display("state changes=%0d  uart start bits=%0d", state_cnt, tx_starts);
        $finish;
    end

    // 超时保护（45 秒物理时间）
    initial begin
        #(NS_PER_MS * 45000);
        $display("!! timeout (45s physical)");
        $finish;
    end

endmodule

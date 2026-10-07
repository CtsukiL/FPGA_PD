//============================================================================
// adc_if.v —— 3PA1030 并行 ADC 接口（角度传感器 SV01A103AEA01R00）
// 对应 STM32 版：AD.c（ADC1_IN8 单次转换，0~4095）+ main.c 的 4 点滑动平均
//   采样：adc_clk = 5MHz（50MHz 十分频；原为 25MHz 二分频。3PA1030 上限 50MSPS，
//         降频只为便于示波器观察/排查）
//   采样节拍与电机 PWM 同步：每 2 个 PWM 周期（100us）采一次、10 次共 1ms
//         （2026-10-02 改；原来是自由分频 125us，与 50us 的 PWM 周期之比 2.5 非整数，
//          采样点相位每次漂移 180 度，电机一转读数就抖 ±10 —— 详见下面"采样节拍"段）
//   再对这 1ms 采样做 4 点滑动平均后输出 —— 与 main.c 的
//     AngleBuf[4]（1ms 一拍、4 点滑动平均）行为一致
//   angle_vld：每 1ms 一个脉冲（对应 main.c 里每 1ms 刷新一次 Angle）
//   数据位宽 {~D[9:0], 2'b0} 扩到 12bit，与 STM32 的 12bit ADC 数值域一致
// 2026-09-21 换新 AD 板（3PA1030 + TPH2501 单 +3V 反相前端，1V 量程）：
//   新板 AIN = 0.5113 - 0.1*Vin、码 = AIN*1024，Vin 升高使码下降（反相）；
//   旧板是同相前端，码随 Vin 上升。用两步把尺度对齐 STM32：
//     ① 原始码按位取反（1023-码）—— 把反相的符号翻回来
//     ② angle = (取反码x4 - OFFSET) x 3 —— 归一化到 0~4095（12.2 count/度，
//        STM32 是 12.3 count/度），使 PID 参数与 CENTER_RANGE 含义一致
//   OFFSET：设计值 1998（对应前级 +IN 偏置 0.465V）。2026-09 实测本板 +IN 约 0.90V，
//           零点整体抬高约 0.48V，按实测改为 28 —— 详见下面归一化段的标定记录
//   归一化后：传感器 0 度 -> angle 0，333.3 度 -> angle ~4050
// OE/STBY 极性：3PA1030 为"低=正常工作"（自制板上直接接地，此处不再输出）
// 参数 DIV：仿真加速比（保留给 sim/tb 的 defparam 用）。注意：改同步采样后
//           DIV 对采样节拍不再生效（采样跟着 motor_pwm 的 PWM 周期走，
//           仿真里 motor_pwm 同样按 2500 拍/周期跑，自动等比缩放）。
//============================================================================
module adc_if #(
    parameter integer DIV = 1           // 仿真加速比，综合时用 1
)(
    input  wire        clk,         // 50MHz 系统时钟
    input  wire        rst_n,       // 低电平复位
    input  wire        smp_pulse,   // 来自 motor_pwm：PWM 周期末尾脉冲（每 50us）
    output reg         adc_clk,     // 5MHz 采样时钟 -> 3PA1030 CLK
    input  wire [9:0]  adc_d,       // 3PA1030 D9~D0 并行数据
    output reg  [11:0] angle,       // 角度值（12bit，0~4095 数值域）
    output reg         angle_vld,   // 角度更新脉冲（每 1ms 一次）
    output reg  [9:0]  raw_code,    // 【调试】本次采样的 ADC 原码（0~1023，未取反）
    output reg  [11:0] avg4_out     // 【调试】8 次平均 + 4 点滑窗后的取反码（0~4092）
);

    //----------------- ADC 采样时钟：50MHz / 10 = 5MHz（占空比 50%）-----------------
    // 每 5 拍翻转一次：半周期 100ns，周期 200ns = 5MHz。
    // （原来是 50MHz 二分频 = 25MHz，要改回就是每拍翻转。）
    reg [2:0] cnt_adc;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_adc <= 3'd0;
            adc_clk <= 1'b0;
        end else if (cnt_adc == 3'd4) begin
            cnt_adc <= 3'd0;
            adc_clk <= ~adc_clk;
        end else begin
            cnt_adc <= cnt_adc + 3'd1;
        end
    end

    //----------------- 采样节拍：与电机 PWM 同步，每 2 个 PWM 周期（100us）一次 ----------------
    // 【2026-10-02 起用】原来是自由分频 125us 一次（8 次 = 1ms）；但 125us / 50us(PWM 周期)
    //   = 2.5 不是整数，采样点相对 PWM 波形每次漂移 180 度，在"高电平期间/低电平期间"之间
    //   交替采样，于是电机一转读数就抖 ±10 count（实测：静止 1~3、按 K1 后 10、断电机恢复 1~3）。
    //   现在改用 motor_pwm 的"周期末尾脉冲"做基准、再 2 分频 → 100us（正好 2 个 PWM 周期），
    //   采样点固定在 PWM 周期的同一相位；10 次正好 1ms，总节拍与原来一致。
    //   注：DIV（仿真加速比）对这个节拍不再生效；仿真里 motor_pwm 同样按 2500 拍/周期跑，
    //       采样间隔会自动等比缩放，行为与硬件一致。
    reg smp_ph;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)         smp_ph <= 1'b0;
        else if (smp_pulse) smp_ph <= ~smp_ph;
    end

    wire smp_en = smp_pulse & smp_ph;           // 每 100us 一个 clk 的采样脉冲

    //----------------- 12bit 扩展 + 1ms 内 10 次平均 ----------------
    // 新板前级反相（Vin 升 -> 码降），这里取反把方向翻回来
    wire [11:0] sample = {~adc_d, 2'b0};        // 10bit 反码 -> 12bit（对齐 STM32 的 12bit 数值域）
    reg  [15:0] sum;
    reg  [3:0]  cnt_add;                        // 0 ~ 9

    //----------------- 4 点滑动平均（对应 main.c 的 AngleBuf[4]）-----------------
    reg  [11:0] b0, b1, b2;                     // 前三次的 1ms 采样（最新值直接参与求和）

    wire [15:0] sum_next = sum + sample;        // 10 次 x 最大 4092 = 40920，16bit 足够
    // 除以 10：移位近似 (x × 3277) >> 15（3277/32768 = 0.100006，误差 0.006%）
    wire [31:0] avg_div10 = (sum_next * 32'd3277) >> 15;
    wire [11:0] avg_new   = avg_div10[11:0];    // 本 1ms 的采样值（除以 10）：最大 4092，正好占 12bit
    // 4 点之和最大 4 x 4092 = 16368，必须显式扩到 14bit 再相加
    // （Verilog 的加法表达式位宽取操作数最大位宽，不扩位会被截断成 12bit）
    wire [13:0] sum4     = {2'b00, avg_new} + {2'b00, b0} + {2'b00, b1} + {2'b00, b2};
    wire [11:0] avg4     = sum4[13:2];          // 4 点滑动平均

    //----------------- 归一化到 0~4095（与 STM32 的角度尺度一致）-----------------
    // 2026-09 实测标定（DA 扫 0~3.3V，33 点；只取两端占空比 0%/100% 的纯直流点）：
    //   输入 0.000V → 原码 1014 / 取反码   9 → avg4 =   36
    //   输入 3.300V → 原码  665 / 取反码 358 → avg4 = 1432
    //   33 点平均值的线性度极好，斜率 = 105.8 码/V（理论 102.4，+3.3%）——
    //   增益与跨度都正常，分辨率不损失；
    //   零点比理论高 490 码（≈0.48V）：前级 +IN 偏置实测约 0.90V（设计 0.465V），
    //   属硬件异常，这里按实测把零点偏移 1998 改成 36 做补偿。
    //   ⚠ 中间点的 RAW 会在 1015 / 665 两簇之间跳，那是 DA 输出未滤波的 PWM 方波
    //     本身（不是 AD 故障），avg 才是占空比加权后的正确值。
    //   x3 不动 ⇒ angle 尺度仍是 0~4095 <-> 0~333.3 度（0.98 度/码），与 STM32 一致，
    //   PID 参数与 CENTER_RANGE 的含义不变。
    //   注：Vout > 3.22V 后 ang_x3 会超 4095 而饱和在 4095（全行程末端约 2.5%），
    //       倒立摆工作区在中心 ±41 度，不受影响。
    //   ⚠ 若以后把 +IN 偏置修回 0.465V，这个常数要改回 1998。
    // 【2026-10-01 改回设计值零点】本板前级偏置已回到设计值（+IN 0.465V）：
    //   实测原码 ≈ 523、AIN ≈ 0.51V（取反码 ≈ 500、avg4 ≈ 2000），
    //   正是本段备注里说的"若把 +IN 修回 0.465V，这个常数要改回 1998"那种情况。
    //   继续用 36 会算出 (2000-36)×2.9375 = 5769，被 clamp 成 4095 —— 屏上 CNT 永远 1023。
    wire signed [15:0] ang_off  = $signed({4'b0000, avg4}) - 16'sd1998;
    // 斜率：实测 1274 count/V，STM32 是 4095/3.3 = 1241 count/V（差 2.7%）。
    // 用 x2.9375（= 3 - 1/16）替掉 x3，把尺度差压到 0.5%，其余参数含义不变。
    wire signed [15:0] ang_x3   = ang_off + (ang_off <<< 1) - (ang_off >>> 4);   // x2.9375        // x3
    wire [11:0]        ang_norm = (ang_x3 <= 16'sd0)    ? 12'd0
                                : (ang_x3 >= 16'sd4095) ? 12'd4095
                                : ang_x3[11:0];

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sum       <= 16'd0;
            cnt_add   <= 4'd0;
            b0        <= 12'd0;
            b1        <= 12'd0;
            b2        <= 12'd0;
            angle     <= 12'd0;
            angle_vld <= 1'b0;
            raw_code  <= 10'd0;
            avg4_out  <= 12'd0;
        end else begin
            angle_vld <= 1'b0;
            if (smp_en) begin
                raw_code <= adc_d;              // 【调试】原码直通（与 avg4 同一拍的数据源）
                if (cnt_add == 4'd9) begin
                    // 第 10 次：本 1ms 采样到手，写滑动窗口并输出 4 点平均
                    sum       <= 16'd0;
                    cnt_add   <= 4'd0;
                    b0        <= avg_new;
                    b1        <= b0;
                    b2        <= b1;
                    angle     <= ang_norm;
                    avg4_out  <= avg4;          // 【调试】滑窗均值（OLED 的 AVG 行）
                    angle_vld <= 1'b1;
                end else begin
                    sum     <= sum_next;
                    cnt_add <= cnt_add + 4'd1;
                end
            end
        end
    end

endmodule

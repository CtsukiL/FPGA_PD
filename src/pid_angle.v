//============================================================================
// pid_angle.v —— 内环角度环（5ms）
// 对应 STM32 版：User/PID.c 的 PID_Update(&AnglePID)
//   Kp = 0.3, Ki = 0.01, Kd = 0.4，输出限幅 ±100
//   PID.c 的积分限幅 I_OUT_MAX = 30（积分项对输出的最大贡献）
// 定点化：Q16（×65536）
//   原先用 Q8 时 0.01 只能取整成 3/256 = 0.0117（偏大 17%）；Q16 下三个系数
//   误差都 <0.1%，与 STM32 的浮点参数真正等效
//   0.3  -> 19661    0.01 -> 655     0.4 -> 26214
//   积分限幅：使 KI*ErrorInt/65536 ≈ ±30 → ErrorInt 限幅 ±3001
//   运算：Out = (KP*e + KI*Σe + KD*(e-e_prev)) >>> 16，再饱和限幅 ±100
//   右移用算术右移，限幅用饱和（不回绕）
//============================================================================
module pid_angle(
    input  wire               clk,          // 50MHz 系统时钟
    input  wire               rst_n,        // 低电平复位
    input  wire               calc_en,      // 5ms 计算脉冲
    input  wire signed [15:0] target,       // 目标角度（= CENTER_ANGLE - 位置环输出）
    input  wire [11:0]        angle,        // 实测角度（12bit，0~4095）
    input  wire               clr,          // 清零积分与误差（起摆入区时）
    output reg  signed [15:0] out           // 输出 -100 ~ +100
);

    //----------------- Q16 参数 ----------------
    localparam signed [15:0] KP = 16'sd19661;   // 0.3  * 65536
    localparam signed [15:0] KI = 16'sd655;     // 0.01 * 65536
    localparam signed [15:0] KD = 16'sd26214;   // 0.4  * 65536

    //----------------- 积分限幅（对应 PID.c 的 I_OUT_MAX = 30）-----------------
    localparam signed [31:0] INT_MAX = 32'sd3001;

    //----------------- 误差计算 ----------------
    wire signed [15:0] angle_s = $signed({4'b0000, angle});
    wire signed [15:0] err_raw = target - angle_s;
    // 【2026-10-03】误差死区（±6 count ≈ 0.8 度）：
    //   FPGA 角度量化粗（1 个 ADC 码 ≈ 0.99 度 = 12 个 angle count，是 STM32 的 12 倍），
    //   静止时读数本就有 ±4~8 count 的抖动；经 Kp=0.3 与 Kd=0.4 放大后约 5~9，
    //   正好压在电机死区 5 上 —— 表现就是"没人碰也会时不时晃一下调整"。
    //   这里把小误差直接吃掉（连同它的微分一起消失，因为 d_err = err - err0），
    //   真正的偏差（远大于 ±6）照常响应，Kd 的阻尼也完整保留（比整体降 Kd 安全，
    //   降 Kd 到 0.2 已实测会失稳）。
    //   若发现摆杆变成"小幅度持续摆"（在死区边缘来回蹭），把阈值调小（如 4）或去掉这层。
    wire signed [15:0] err     = (err_raw > 16'sd6 || err_raw < -16'sd6) ? err_raw : 16'sd0;

    reg  signed [15:0] err0, err1;
    reg  signed [31:0] err_int;

    wire signed [15:0] d_err  = err - err0;              // 本次误差 - 上次误差
    wire signed [31:0] i_next = err_int + err;           // 本次积分累加
    wire signed [31:0] i_cl   = (i_next >  INT_MAX) ?  INT_MAX :
                                (i_next < -INT_MAX) ? -INT_MAX : i_next;

    //----------------- PID 计算（32bit 域，避免中间截断）-----------------
    wire signed [31:0] KP32 = {{16{KP[15]}}, KP};
    wire signed [31:0] KI32 = {{16{KI[15]}}, KI};
    wire signed [31:0] KD32 = {{16{KD[15]}}, KD};

    wire signed [31:0] err32   = err;
    wire signed [31:0] derr32  = d_err;
    wire signed [31:0] icl32   = i_cl;

    wire signed [39:0] acc     = KP32 * err32
                               + KI32 * icl32
                               + KD32 * derr32;
    wire signed [39:0] acc_s   = acc >>> 16;             // 算术右移，还原 Q16

    // 饱和限幅 ±100（先限幅再截位，避免回绕）
    wire signed [39:0] o_lim   = (acc_s >  40'sd100) ?  40'sd100 :
                                 (acc_s < -40'sd100) ? -40'sd100 : acc_s;

    //----------------- 每 5ms 更新一次 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err0    <= 16'sd0;
            err1    <= 16'sd0;
            err_int <= 32'sd0;
            out     <= 16'sd0;
        end else if (clr) begin
            // 与 STM32 入区时清零 ErrorInt / Error0 / Error1 对应
            err0    <= 16'sd0;
            err1    <= 16'sd0;
            err_int <= 32'sd0;
            out     <= 16'sd0;
        end else if (calc_en) begin
            err1    <= err0;
            err0    <= err;
            err_int <= i_cl;
            out     <= o_lim[15:0];
        end
    end

endmodule

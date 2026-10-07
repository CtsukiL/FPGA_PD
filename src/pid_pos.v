//============================================================================
// pid_pos.v —— 外环位置环（50ms）
// 对应 STM32 版 (2) 工程 main.c 的 LocationPID（走 PID.c 的 PID_Update）：
//   Kp = 0.4, Kd = 4，输出限幅 ±40【2026-10-06 由 ±100 收到 ±40：治运动结束后来回晃】
//   【2026-10-06】加回积分（KI 0.10 / I_MAX 48 / 泄漏 1/32，只在运动态参与）：
//     纯 PD 要出力就必须有误差，于是"小误差没劲（恢复慢）、攒够才动（步长大）"；
//     积分让小误差也能持续出力。带 active 端口：只在运动态（走轨迹 / 手动移动）参与，静止态清零。
//   main.c 在 PID_Update 之后加了输出死区 |Out| < 3 -> 0，这里等价实现
//   公式与 PID.c 的 Out = Kp*e + Ki*Σe + Kd*(e-e1) 一致（Ki 项为 0）
// 定点化：Q16（×65536，与 pid_angle 统一）
//   0.4 -> 26214     4.0 -> 262144
//   运算：Out = (KP*e + KD*(e-e_prev)) >>> 16，再饱和限幅 ±100，最后过输出死区
// 说明：Ki 固定为 0（避免低频振荡），与 STM32 版一致
//============================================================================
module pid_pos(
    input  wire               clk,          // 50MHz 系统时钟
    input  wire               rst_n,        // 低电平复位
    input  wire               calc_en,      // 50ms 计算脉冲
    input  wire signed [31:0] target,       // 位置目标（count，1 圈 = 408）
    input  wire signed [31:0] location,     // 实测位置（count）
    input  wire               clr,          // 刷新（起摆入区：把上次误差置为当前误差，D 项首拍为 0）
    input  wire               active,       // 【2026-10-06】1 = 运动中（走轨迹 / 手动移动）：积分只在这时参与
    output reg  signed [15:0] out           // 输出 -40 ~ +40（限幅 OUT_MAX/OUT_MIN）
);

    //----------------- Q16 参数 ----------------
    localparam signed [31:0] KP = 32'sd26214;   // 0.4  * 65536
    localparam signed [31:0] KD = 32'sd262144;  // 4.0  * 65536

    localparam signed [39:0] OUT_MAX  = 40'sd40;    // 【2026-10-06】100 -> 40：±100 允许摆杆被撑到 8 度去换横杆回中，运动结束后来回晃、恢复慢
    localparam signed [39:0] OUT_MIN  = -40'sd40;   // 【2026-10-06】-100 -> -40（同上）
    localparam signed [39:0] DEADZONE = 40'sd1; // 【2026-10-06】3 -> 1：死区 3 对应误差 7.5 count，小修正被自己截掉 -> 静止态稳定慢（电机死区已降到 2，瓶颈转到这一环）

    //----------------- 积分项（2026-10-06 加回，只用在运动态）-----------------
    // 纯 PD 的固有毛病：要出力就必须有误差 -> 小误差没劲（恢复慢），攒够才动（步长大）。
    // 积分让小误差也能持续出力；`active` = 运动态才参与，静止态清零（静止行为仍是纯 PD）。
    // 泄漏 1/32（时间常数 32 拍 = 1.6s）：平衡点 = 32×跟踪误差，避免一路爬到限幅顶死。
    localparam signed [31:0] KI    = 32'sd6554;   // 0.10 * 65536
    localparam signed [31:0] I_MAX = 32'sd0;      // 【2026-10-06】48 -> 0：**本版关掉位置环积分**（代码保留，随时可开回）。
                                                  //   原因：位置环 + 角度环两个积分器 = 相位裕度不够 -> 低频摆，
                                                  //   表现就是长按"走一段、回移一段、反复"。恒定推力改由固定前馈 POS_FF 提供（不占相位）。

    //----------------- 误差计算 ----------------
    wire signed [31:0] err   = target - location;
    reg  signed [31:0] err1;
    wire signed [31:0] d_err = err - err1;      // 50ms 内的位置变化

    // 积分：累加 -> 限幅 -> 弱泄漏（对绝对值做泄漏，保证正负对称）
    // 【2026-10-06】抗饱和（条件积分）：已经顶到限幅、且误差还在同方向推时，本拍不再累加。
    //   目的是不让"顶满后继续加"变成过量偏置（顶满正是"走一段回移一段"的来源）。
    //   饱和时保持 err_int 不动，泄漏仍会正常抽它，所以不会锁死。
    reg  signed [31:0] err_int;
    wire signed [31:0] i_raw  = err_int + err;
    wire               i_sat_u = (i_raw >=  I_MAX) && (err > 32'sd0);
    wire               i_sat_d = (i_raw <= -I_MAX) && (err < 32'sd0);
    wire signed [31:0] i_next = (i_sat_u || i_sat_d) ? err_int : i_raw;
    wire signed [31:0] i_cl   = (i_next >  I_MAX) ?  I_MAX :
                                (i_next < -I_MAX) ? -I_MAX : i_next;
    wire signed [31:0] i_abs  = (i_cl < 0) ? -i_cl : i_cl;
    wire signed [31:0] i_mag  = i_abs - (i_abs >>> 5);   // 【2026-10-06】泄漏 1/64 -> 1/32：平衡点从 64×误差 降到 32×误差，不再顶到 I_MAX（顶满就"走一段回移一段"）
    wire signed [31:0] i_leak = (i_cl < 0) ? -i_mag : i_mag;

    //----------------- PID 计算 ----------------
    wire signed [31:0] ki_term = active ? (KI * i_cl) : 32'sd0;   // 【2026-10-06】积分只在运动态参与
    wire signed [39:0] acc   = KP * err + ki_term + KD * d_err;
    wire signed [39:0] acc_s = acc >>> 16;

    // 先饱和限幅
    wire signed [39:0] o_lim = (acc_s > OUT_MAX) ? OUT_MAX :
                               (acc_s < OUT_MIN) ? OUT_MIN : acc_s;
    // 再过输出死区
    wire signed [39:0] o_dz  = (o_lim > -DEADZONE && o_lim < DEADZONE) ? 40'sd0 : o_lim;

    //----------------- 每 50ms 更新一次 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            err1    <= 32'sd0;
            err_int <= 32'sd0;
            out     <= 16'sd0;
        end else if (clr) begin
            // 起摆入区 / 标定：位置零点与位置目标都会归零，误差历史与积分也清零
            // （对应 STM32 的 Location = 0、Target = 0、Error0 = Error1 = 0）
            // 注意不能写成"err1 <= err"：enc_zero 要下一拍才把位置清零，
            // 这一拍 err 还是旧的大偏差，会变成一次微分冲击。
            err1    <= 32'sd0;
            err_int <= 32'sd0;
            out     <= 16'sd0;
        end else if (calc_en) begin
            err1    <= err;
            err_int <= active ? i_leak : 32'sd0;   // 【2026-10-06】静止态立刻清零（避免残留偏置带跑横杆）
            out     <= o_dz[15:0];
        end
    end

endmodule

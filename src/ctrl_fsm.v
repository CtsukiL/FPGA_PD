//============================================================================
// ctrl_fsm.v —— 控制状态机 + 双环串级 PID 调度
// 对应 STM32 版 (2) 工程 main.c 的 TIM1_UP_IRQHandler()：
//   状态 0            停止（PWM = 0）
//   状态 1            判断：每 40ms 采样角度，攒 3 点，判左/右极值 + 判入区
//   状态 21/22/23/24  左推 +START_PWM 持续 100ms，再右推 -START_PWM 持续 100ms，回到状态 1
//   状态 31/32/33/34  对称的反方向
//   状态 4            PID 控制：角度环 5ms、位置环 50ms，角度超出 C±R 自动回 0
// 耦合关系（唯一一行）：角度环目标 = CENTER_ANGLE - 位置环输出 - 摩擦前馈 pos_ff
//   （pos_ff 是 2026-10-04 加的：运动时给一个方向上的固定工作点偏置，治匀速粘滑顿挫）
// 【2026-10-04 新增】梯形速度曲线轨迹发生器（赛题拓展 2，参数见下面参数区）：
//   K2/K3 只设定"目的地" pos_set，位置环的 target 由 pos_cmd 按速度曲线逐步推进，
//   于是横杆是平滑加减速走过去、平滑停住，而不是被位置环猛追过去。
//   （赛题拓展 1"平滑移动到指定位置并保持静止"、拓展 3"移动中摆杆始终直立"同时受益。）
//   这一点与 STM32 版不同（STM32 是 K2/K3 直接阶跃写 Target），是有意为赛题加的。
//   操作：K2/K3 **短按** = 目标 ±360 度（走一遍梯形速度曲线）；**长按** = 手动点动
//         慢移（按住有效、松手即停，速度 JOG_STEP/50ms）。
// 与 STM32 版的两处有意差异：
//   1) 本文把 (2) 版缺失的"三点角度缓冲区清零"补上（状态 24/34 回状态 1 时清 a0/a1/a2），
//      这正是外层主线已定位并修掉的"第二次起摆就猛动掉下来"的根因。
//   2) 位置零点只由"标定键"设定并保持，起摆入区时不清编码器位置，
//      所以平衡后横杆会回到标定那一刻的初始位置。
//============================================================================
`include "src/pendulum_cfg.vh"   // 与编码器分辨率(count/圈)有关的常数（换电机只改这个文件）

module ctrl_fsm(
    input  wire               clk,
    input  wire               rst_n,
    input  wire               tick_1ms,
    input  wire               tick_5ms,
    input  wire               tick_40ms,
    input  wire               tick_50ms,

    input  wire [11:0]        angle,          // 实测角度（12bit）
    input  wire signed [31:0] location,       // 实测位置（编码器 count）

    input  wire               ev_start_stop,  // K1：启停
    input  wire               ev_step_plus,   // K2 短按：目标位置 +360 度（由速度曲线走完）
    input  wire               ev_step_minus,  // K3 短按：目标位置 -360 度（由速度曲线走完）
    input  wire               ev_jog_plus,    // K2 长按：手动点动 +（按住有效，松手即停）
    input  wire               ev_jog_minus,   // K3 长按：手动点动 -
    input  wire               ev_calib,       // K4：标定平衡点 + 记录初始位置

    output reg  [5:0]         run_state,      // 运行状态
    output reg  signed [15:0] motor_cmd,      // 电机指令 -100 ~ +100
    output wire signed [15:0] angle_out,      // 角度环输出（调试用）
    output wire signed [15:0] pos_out,        // 位置环输出（调试用）
    output reg  [11:0]        center_angle,   // 平衡点角度值（可在线标定）
    output wire signed [31:0] pos_target,     // 位置环目标 = 轨迹发生器输出 pos_cmd（2026-10-04 前是 K2/K3 直接阶跃）
    output wire signed [31:0] pos_set_out,    // 目标位置 pos_set（给 OLED 显示）
    output reg  signed [31:0] bar_vel,        // 横杆速度（count/s，50ms 差分 ×20，给 OLED 显示）
    output wire signed [31:0] target_vel,     // ???? count/s??????????? 0?
    output wire               mov_active,     // 【2026-10-04】1 = 走轨迹/点动（给 motor_pwm 选死区档）
    output reg                enc_zero        // 位置零点清零脉冲 -> encoder_if
);

    //----------------- 参数（与 STM32 版逐项一致）-----------------
    localparam signed [15:0] CENTER_RANGE = 16'sd500;   // 中心区间 ±500
    localparam signed [15:0] START_PWM    = 16'sd35;    // 起摆推力【2026-10-10 换 JGA25-370：50 -> 35；新电机扭矩约 2.3 倍，50 会把摆杆甩过头】
    localparam [7:0]         START_TIME   = 8'd100;     // 起摆推力持续时间 100ms
    localparam signed [31:0] POS_STEP     = `CFG_POS_STEP;   // 一次 360 度 = 1 圈（数值见 pendulum_cfg.vh）
    localparam signed [31:0] POS_LIMIT    = `CFG_POS_LIMIT;  // 位置目标限幅 ±10 圈
    localparam [11:0]        CENTER_INIT  = 12'd2400;   // 平衡点角度初值（0~4095 尺度）= 屏上 CT 600；上电默认值，按 K4 后会被实际标定值覆盖【2026-10-06 先设 2396(CT 599)，按用户要求取整到 600】

    //----------------- 起摆前置：摆杆静止判定（2026-10-10）-----------------
    // 需求：停止态要"摆杆不动 1 秒以上"再按 K1 才起摆（避免手还扶着、或摆杆还在晃就起摆）。
    // 静止定义：当前角度与参考点相差不超过 STILL_RANGE；一旦超了就把参考点跟到当前值并
    //   重新计时，所以缓慢漂移不会被误判成静止。
    localparam signed [12:0] STILL_RANGE = 13'sd16;    // 角度 ±16 count = 屏上 CNT 行的 ±4
    localparam [9:0]         STILL_TIME  = 10'd1000;   // 需要连续静止 1000ms

    //----------------- 轨迹发生器参数（2026-10-04，赛题拓展 2）-----------------
    // 赛题拓展 2 "让电机按照特定的速度曲线或轨迹运动"：
    //   K2/K3 设的目标 pos_set 不再直接给位置环，而是交给这里的梯形速度曲线发生器，
    //   按"加速 -> 匀速(够长才有) -> 减速"把 pos_cmd 推到 pos_set，位置环只跟随 pos_cmd。
    //   赛题拓展 1 "平滑地移动到指定方向并保持静止" 与拓展 3 "移动中摆杆始终直立"
    //   也随之改善：横杆是被"带着走"而不是被位置环猛追，摆杆倾角小。
    //   单位：vel 是 count/50ms，故 1 count/50ms = 20 count/s。
    //   最高速 = CFG_TRAJ_VMAX、加速度 = CFG_TRAJ_ASTEP / TRAJ_ACC_DIV，
    //   两者都在 pendulum_cfg.vh 里按 count/圈 折算，物理量不随换电机变化：
    //   141 度/s（约 0.4 圈/s）、约 8.8 度/s^2，走完一圈约 3.25s。
    localparam signed [31:0] TRAJ_VMAX  = `CFG_TRAJ_VMAX;   // 最大速度（count/50ms）≙ 141 度/s
    localparam signed [31:0] TRAJ_ASTEP = `CFG_TRAJ_ASTEP;  // 每档速度增量（count/50ms）
    // 【2026-10-04】加速度分频（速度每 TRAJ_ACC_DIV 拍才变一档）。
    //   曾设 2 把加速度减半试"起步太急"，但那是误判 —— 用户实测"不是速度问题"，
    //   真正的毛病是横杆运动本身不连续（见文件末"运动不连续"注释），所以调回 1。【2026-10-06 又设 2：K2/K3 短按"经常冲过头"—— 减速只有 400ms/36 count，比位置环能跟的更快，横杆一路落后、到站才猛追而冲过；加减速各减半后跟得上（一圈 2.9->3.25s）】
    //   想改加速度直接改这个数，减速距离会跟着算（见 t_dec_dist）。
    localparam integer       TRAJ_ACC_DIV = 2;      // 【2026-10-06】1 -> 2：短按冲过头，加减速各减半

    //----------------- 长按点动参数（2026-10-04）-----------------
    // K2/K3 按住不放时，目标以 JOG_STEP 每 50ms 的速度恒定缓慢推移（松手立即停），
    // 用来手动把横杆"挪"到想要的位置演示（不走梯形曲线，就是匀速慢移）。
    //   当前约 70.6 度/s（走 1 圈约 5s）。2026-10-04 由半速提上来（用户嫌慢）。
    //   注：点动速度提高还有个附带好处 —— 位置误差累积更快、更快越过"推动阈值"，
    //   粘滑的"停"那半段会变短。再快就是 6 / 8（一圈 3.4s / 2.6s）。
    localparam signed [31:0] JOG_STEP = `CFG_JOG_STEP;   // 约 70.6 度/s（一圈约 5s）【2026-10-06 试过半速，与平衡无关，已回原值】

    //----------------- 状态编码 ----------------
    localparam [5:0] S_STOP = 6'd0;
    localparam [5:0] S_JUDGE = 6'd1;
    localparam [5:0] S_21 = 6'd21, S_22 = 6'd22, S_23 = 6'd23, S_24 = 6'd24;
    localparam [5:0] S_31 = 6'd31, S_32 = 6'd32, S_33 = 6'd33, S_34 = 6'd34;
    localparam [5:0] S_PID = 6'd4;

    //----------------- 内部寄存器 ----------------
    reg [11:0] a0, a1, a2;          // 本次 / 上次 / 上上次角度（40ms 间隔采样）
    reg [7:0]  count_time;          // 起摆推力计时（ms）

    reg [9:0]  still_ms;            // 【2026-10-10】已连续静止的时长（ms）
    reg [11:0] still_ref;           // 静止判定的参考角度

    // 轨迹发生器状态（2026-10-04，赛题拓展 2）
    reg signed [31:0] pos_set;      // K2/K3 设定的目标位置（count，阶跃）
    reg signed [31:0] pos_cmd;      // 轨迹发生器输出（count）= 位置环的 target
    reg signed [31:0] traj_vel;     // 当前轨迹速度（count/50ms）
    reg signed [31:0] loc_prev;     // 上一个 50ms 的横杆位置（算速度给 OLED 显示）
    reg [3:0]         acc_ph;       // 加速度分频相位（每 TRAJ_ACC_DIV 个 50ms 才让速度变一档）

    reg        ang_calc, pos_calc;  // 两环计算脉冲
    reg        ang_clr, pos_clr;    // 两环清零/刷新脉冲
    reg signed [15:0] ang_target;   // 角度环目标

    wire signed [15:0] ang_out_w;   // 角度环输出
    wire signed [15:0] pos_out_w;   // 位置环输出

    // 【2026-10-10】起摆前置：角度偏离参考点多少 / 是否已静止够久（给 K1 用）
    wire signed [12:0] still_d  = $signed({1'b0, angle}) - $signed({1'b0, still_ref});
    wire               still_ok = (still_ms >= STILL_TIME);

    //----------------- 区间与极值判据（对应 STM32 的 C±R 判断）-----------------
    wire signed [15:0] ang_s = $signed({4'b0000, angle});
    wire signed [15:0] c_s   = $signed({4'b0000, center_angle});
    wire signed [15:0] hi    = c_s + CENTER_RANGE;
    wire signed [15:0] lo    = c_s - CENTER_RANGE;

    wire signed [15:0] p0 = $signed({4'b0000, a0});
    wire signed [15:0] p1 = $signed({4'b0000, a1});

    // ★ 与 STM32 一致：STM32 是"先赋值再判断"（A2=A1; A1=A0; A0=angle; 再用 A0/A1/A2），
    //   即用最新的三点 (angle, 旧a0, 旧a1)。这里显式写成 n0/n1/n2 ——
    //   否则非阻塞赋值下组合判据读到的是移位前的老值，判据整体滞后一拍（推力晚 40ms）。
    //   【2026-10-02 恢复】这两处修复 2026-09-29 做过，10-01 回退工程时丢失，
    //   现象就是"起摆晃一个来回就停下"（入区单点 + 极值滞后）。
    wire signed [15:0] n0 = ang_s;   // 本次 40ms 采样
    wire signed [15:0] n1 = p0;      // 上次
    wire signed [15:0] n2 = p1;      // 上上次

    wire in_zone   = (ang_s > lo) && (ang_s < hi);   // 单点：状态 4 出区判据（与 STM32 一致）
    // 状态 1 入区判据：STM32 用"最新两点都在中心区"(A0 && A1)，这里对应 (angle, a0)
    wire in_zone_2 = in_zone && (p0 > lo) && (p0 < hi);
    wire out_zone  = !in_zone;

    // 右侧最高点：3 点都在右区间，且中间那次最小
    wire right_peak = (n0 > hi) && (n1 > hi) && (n2 > hi) && (n1 < n0) && (n1 < n2);
    // 左侧最高点：3 点都在左区间，且中间那次最大
    wire left_peak  = (n0 < lo) && (n1 < lo) && (n2 < lo) && (n1 > n0) && (n1 > n2);

    //----------------- 轨迹发生器组合逻辑（2026-10-04，赛题拓展 2）-----------------
    // 把 pos_cmd 从当前位置按"加速 -> 匀速(够长才有) -> 减速"推到 pos_set：
    //   ① 减速判据：剩余距离 <= "从当前速度减速到 0 所需的距离" t_dec_dist 就开始减速，
    //      这样到位时速度刚好归零、不会冲过头 —— 这是"平滑停住"的关键。
    //      t_dec_dist = N·v(v+1)/2，N = TRAJ_ACC_DIV（速度每 N 拍才降一档，所以要乘 N）；
    //      N=1 时退化成原来的 v(v+1)/2。
    //   ② 速度每 TRAJ_ACC_DIV 拍才变一档 TRAJ_ASTEP、限幅 TRAJ_VMAX —— 这就是"加速度/最大速度限制"。
    //      TRAJ_ACC_DIV 是 2026-10-04 加的加速度分频档（用户反馈起步太快）。
    //   ③ pos_cmd 用当前速度推进，并用 pos_set 夹住；离目标 <= 1 档且速度 <= 1 档时吸附到位。
    wire signed [31:0] t_diff     = pos_set - pos_cmd;
    wire signed [31:0] t_diff_abs = (t_diff < 0) ? -t_diff : t_diff;
    wire signed [31:0] t_vel_abs  = (traj_vel < 0) ? -traj_vel : traj_vel;
    wire signed [31:0] t_dec_dist = (t_vel_abs * (t_vel_abs + 32'sd1) * TRAJ_ACC_DIV) >>> 1;
    wire               t_brake    = (t_diff_abs <= t_dec_dist);
    wire               t_snap     = (t_diff_abs <= TRAJ_ASTEP) && (t_vel_abs <= TRAJ_ASTEP);
    // 加速度分频：只有相位归零那一拍才允许速度变一档（起步和刹车都更缓）
    wire               t_acc_en   = (acc_ph == 4'd0);

    wire signed [31:0] t_vel_acc  = (t_acc_en && (t_diff > 0)) ? (traj_vel + TRAJ_ASTEP) :
                                    (t_acc_en && (t_diff < 0)) ? (traj_vel - TRAJ_ASTEP) : traj_vel;
    wire signed [31:0] t_vel_dec  = (t_acc_en && (traj_vel > 0)) ? (traj_vel - TRAJ_ASTEP) :
                                    (t_acc_en && (traj_vel < 0)) ? (traj_vel + TRAJ_ASTEP) : traj_vel;
    wire signed [31:0] t_vel_raw  = t_brake ? t_vel_dec : t_vel_acc;
    wire signed [31:0] t_vel_sat  = (t_vel_raw >  TRAJ_VMAX) ?  TRAJ_VMAX :
                                    (t_vel_raw < -TRAJ_VMAX) ? -TRAJ_VMAX : t_vel_raw;
    wire signed [31:0] vel_nxt    = t_snap ? 32'sd0 : t_vel_sat;

    wire signed [31:0] t_cmd_try  = pos_cmd + traj_vel;   // 用当前速度推进（非阻塞语义）
    // 夹住：朝目标方向运动时不许越过 pos_set；t_diff=0 时也夹（防止残余速度把目标推离）。
    // 背向目标时（比如运行中把目标改到反方向）不夹，让它先按加速度限制刹住再掉头。
    wire signed [31:0] cmd_nxt    = t_snap ? pos_set :
                                    (t_diff >= 0 && t_cmd_try > pos_set) ? pos_set :
                                    (t_diff <= 0 && t_cmd_try < pos_set) ? pos_set : t_cmd_try;

    //----------------- 位置环"运动中"标志（2026-10-04，给 pid_pos 选死区档）-----------------
    // 轨迹正在推进（traj_vel != 0）或正在手动点动 -> 用运动档小死区，
    // 否则（静止平衡、被手碰后回中）用静止档大死区压噪声。
    // 【2026-10-06 已回退】曾加“位置误差 > 40 也算运动中（用小死区）”来加快轨迹结束后的收敛，
    //   上板结果：静止时误差常越过 40 -> 长期挂在小死区 -> 噪声穿透 -> 立不住。
    //   与 10-04 那次（门限 6 < 静差）是同一类坑：**拿“误差大小”切死区档不可靠，别再试**。
    //   保持原设计：只有“轨迹推进中 / 手动点动”用小死区。
    //   （`ki_en` 这一路仍保留：只在轨迹/点动期间让积分参与，与 active 同源，行为与原版一致。）
    wire motion_cmd = (traj_vel != 32'sd0) || ev_jog_plus || ev_jog_minus;
    wire pos_active = motion_cmd;

    //----------------- 摩擦前馈（2026-10-04，治匀速运动的粘滑顿挫）-----------------
    // 现象：运动（尤其长按点动这种慢速）时横杆"走一点 → 停一下 → 再走一点"。
    // 机制：匀速在物理上只需要一个很小、恒定的输出抵消摩擦，而位置环是纯 P，没法平滑
    //   维持这种恒定小输出，只能靠误差不断累积把它顶出来 -> 误差攒够才推一下、推完误差
    //   回落又松掉 -> 电机"攒够才动一次"，周期 150~250ms。
    // 做法：运动时直接给角度目标一个方向上的固定工作点偏置 —— 不用等误差累积，
    //   横杆立刻就有持续的推力。位置环的 P 项在这个偏置上继续做微调。
    //   为什么不用积分：位置环积分会让回路多一个积分器（横杆位置本来就是对倾角的二次
    //     积分），相位裕度不够 -> 运动一会儿后低频摆（已实测，见 pid_pos.v 注释）。
    //   整定：顿挫还在就把 POS_FF 加大（如 8~10）；横杆跑得偏快/到位过冲就减小（如 3）。
    //   注意 direction 只看"运动意图"：轨迹速度的方向，或点动的按键方向。
    localparam signed [15:0] POS_FF = `CFG_POS_FF;   // 【2026-10-06】0 -> 3：位置环积分已关掉（I_MAX=0），恒定推力改由固定前馈提供（不引入第二个积分器）
                                                     //   它作用在角度环目标上（角度 count 域），**不随位置分辨率缩放**；
                                                     //   但换电机后摩擦特性变了，到货要重新整定（见 pendulum_cfg.vh）

    wire signed [15:0] pos_ff = (traj_vel >  32'sd0) ?  POS_FF :
                                (traj_vel <  32'sd0) ? -POS_FF :
                                ev_jog_plus          ?  POS_FF :
                                ev_jog_minus         ? -POS_FF : 16'sd0;

    //----------------- 起摆前的"摆杆静止"计时（2026-10-10）-----------------
    // 1ms 一拍：角度还在 STILL_RANGE 内就继续累加；一旦超出去就把参考点跟到当前角度、
    // 计时清零（所以缓慢漂移不会被当成静止）。still_ms 到 STILL_TIME 即视为静止够久。
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            still_ms  <= 10'd0;
            still_ref <= 12'd0;
        end else if (tick_1ms) begin
            if ((still_d > STILL_RANGE) || (still_d < -STILL_RANGE)) begin
                still_ref <= angle;
                still_ms  <= 10'd0;
            end else if (still_ms < STILL_TIME) begin
                still_ms <= still_ms + 10'd1;
            end
        end
    end

    //----------------- 主状态机 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            run_state    <= S_STOP;
            motor_cmd    <= 16'sd0;
            a0           <= 12'd0;
            a1           <= 12'd0;
            a2           <= 12'd0;
            count_time   <= 8'd0;
            ang_calc     <= 1'b0;
            pos_calc     <= 1'b0;
            ang_clr      <= 1'b0;
            pos_clr      <= 1'b0;
            ang_target   <= 16'sd0;
            center_angle <= CENTER_INIT;
            pos_set      <= 32'sd0;
            pos_cmd      <= 32'sd0;
            traj_vel     <= 32'sd0;
            loc_prev     <= 32'sd0;
            bar_vel      <= 32'sd0;
            acc_ph       <= 4'd0;
            enc_zero     <= 1'b0;
        end else begin
            // 脉冲信号默认拉低（单周期）
            ang_calc <= 1'b0;
            pos_calc <= 1'b0;
            ang_clr  <= 1'b0;
            pos_clr  <= 1'b0;
            enc_zero <= 1'b0;

            // 横杆位置采样（50ms 一拍，bar_vel 用它做差分）
            if (tick_50ms) begin
                loc_prev <= location;
                bar_vel  <= (location - loc_prev) * 32'sd20;
            end
            // ???????????????????????
            if (enc_zero) begin
                loc_prev <= 32'sd0;
                bar_vel  <= 32'sd0;
            end
            // 加速度分频相位：每 TRAJ_ACC_DIV 拍才让 traj_vel 变一档（见参数区说明）
            if (tick_50ms) acc_ph <= (acc_ph == TRAJ_ACC_DIV[3:0] - 4'd1) ? 4'd0 : acc_ph + 4'd1;

            //---------- K2 / K3：设定目标位置（±90 度 = ±102 count）----------
            // 【2026-10-04】改的是 pos_set（目标的"目的地"），不再直接写位置环的 target；
            //   位置环的目标由下面的梯形速度曲线发生器 pos_cmd 产生。
            if (ev_step_plus) begin
                if (pos_set <= (POS_LIMIT - POS_STEP)) pos_set <= pos_set + POS_STEP;
                else                                   pos_set <= POS_LIMIT;
            end
            if (ev_step_minus) begin
                if (pos_set >= (-POS_LIMIT + POS_STEP)) pos_set <= pos_set - POS_STEP;
                else                                    pos_set <= -POS_LIMIT;
            end

            //---------- K4：标定平衡点 + 记录初始位置（仅停止态有效）----------
            if (ev_calib && (run_state == S_STOP)) begin
                center_angle <= angle;      // 平衡点 = 当前角度（把摆杆扶到机械竖直后按）
                pos_set      <= 32'sd0;     // 目标位置回到初始位置
                pos_cmd      <= 32'sd0;     // 轨迹发生器同步归零
                traj_vel     <= 32'sd0;
                enc_zero     <= 1'b1;       // 把横杆当前位置记为初始位置
                pos_clr      <= 1'b1;       // 位置环误差历史清零，避免微分冲击
                ang_clr      <= 1'b1;       // 角度环同样清零：对应 STM32 标定时清 AnglePID 的
                                            // ErrorInt / Error0 / Error1（否则积分残留在下次进状态 4 时才发作）
            end

            //---------- K1：启停 ----------
            if (ev_start_stop) begin
                if (run_state == S_STOP) begin
                    // 【2026-10-10】起摆前置：摆杆静止 ≥1s（still_ok）才响应，否则忽略这次按键
                    if (still_ok) run_state <= S_21;            // 停止 -> 开始起摆
                end else begin
                    run_state <= S_STOP;                        // 运行 -> 停止
                end
            end

            //---------- 状态机 ----------
            case (run_state)

                // 状态 0：停止
                S_STOP: begin
                    motor_cmd <= 16'sd0;
                end

                // 状态 1：判断极值 / 判入区
                S_JUDGE: begin
                    if (tick_40ms) begin
                        a2 <= a1;
                        a1 <= a0;
                        a0 <= angle;

                        if (right_peak) begin
                            run_state <= S_21;
                        end else if (left_peak) begin
                            run_state <= S_31;
                        end else if (in_zone_2) begin
                            // 入区：位置零点重置为"起摆完成点"（与 STM32 版同步）
                            // 起摆过程中横杆被推力带着转了几圈，若沿用旧零点，位置误差会有
                            // 几百~上千 count，位置环输出立刻饱和 -> 变成固定摆杆偏置 ->
                            // 横杆朝一个方向匀加速转圈（追不上目标）。所以这里归零。
                            enc_zero   <= 1'b1;     // 对应 STM32 的 Location = 0
                            pos_set    <= 32'sd0;   // 对应 LocationPID.Target = 0
                            pos_cmd    <= 32'sd0;   // 轨迹发生器同步归零（否则旧目标会留下来）
                            traj_vel   <= 32'sd0;
                            ang_clr    <= 1'b1;     // 角度环清零
                            pos_clr    <= 1'b1;     // 位置环误差清零
                            ang_target <= c_s;      // 角度环目标复位为平衡点
                            run_state  <= S_PID;
                        end
                    end
                end

                // 状态 21/22/23/24：先左推再右推，各持续 START_TIME ms
                S_21: begin
                    motor_cmd  <= START_PWM;
                    count_time <= START_TIME;
                    run_state  <= S_22;
                end
                S_22: begin
                    if (tick_1ms) begin
                        count_time <= count_time - 8'd1;
                        if (count_time == 8'd1) run_state <= S_23;
                    end
                end
                S_23: begin
                    motor_cmd  <= -START_PWM;
                    count_time <= START_TIME;
                    run_state  <= S_24;
                end
                S_24: begin
                    if (tick_1ms) begin
                        count_time <= count_time - 8'd1;
                        if (count_time == 8'd1) begin
                            motor_cmd <= 16'sd0;
                            a0 <= 12'd0; a1 <= 12'd0; a2 <= 12'd0;   // 三点缓冲区作废
                            run_state <= S_JUDGE;
                        end
                    end
                end

                // 状态 31/32/33/34：反方向，先右推再左推
                S_31: begin
                    motor_cmd  <= -START_PWM;
                    count_time <= START_TIME;
                    run_state  <= S_32;
                end
                S_32: begin
                    if (tick_1ms) begin
                        count_time <= count_time - 8'd1;
                        if (count_time == 8'd1) run_state <= S_33;
                    end
                end
                S_33: begin
                    motor_cmd  <= START_PWM;
                    count_time <= START_TIME;
                    run_state  <= S_34;
                end
                S_34: begin
                    if (tick_1ms) begin
                        count_time <= count_time - 8'd1;
                        if (count_time == 8'd1) begin
                            motor_cmd <= 16'sd0;
                            a0 <= 12'd0; a1 <= 12'd0; a2 <= 12'd0;   // 三点缓冲区作废
                            run_state <= S_JUDGE;
                        end
                    end
                end

                // 状态 4：倒立摆 PID 控制
                S_PID: begin
                    if (out_zone) begin
                        // 摆杆倒下：自动停止（对应 STM32 的 RunState = 0）
                        run_state <= S_STOP;
                        motor_cmd <= 16'sd0;
                    end else begin
                        // 角度环 5ms，输出直接写电机 PWM
                        if (tick_5ms) begin
                            ang_calc  <= 1'b1;
                            motor_cmd <= ang_out_w;
                        end
                        // 位置环 50ms
                        if (tick_50ms) begin
                            pos_calc <= 1'b1;

                            //------- 长按点动优先（2026-10-04）-------
                            // 按住 K2/K3：pos_cmd 与 pos_set 一起以 JOG_STEP 匀速慢移，
                            // 松手当拍立刻停止（速度归零）。点动期间把 pos_set 拉到与 pos_cmd
                            // 同步，即取消未走完的曲线行程 —— 松手后目标就是当前位置。
                            if (ev_jog_plus || ev_jog_minus) begin
                                if (ev_jog_plus && (pos_cmd <= (POS_LIMIT - JOG_STEP))) begin
                                    pos_cmd <= pos_cmd + JOG_STEP;
                                    pos_set <= pos_cmd + JOG_STEP;
                                end else if (ev_jog_minus && (pos_cmd >= (-POS_LIMIT + JOG_STEP))) begin
                                    pos_cmd <= pos_cmd - JOG_STEP;
                                    pos_set <= pos_cmd - JOG_STEP;
                                end
                                traj_vel <= 32'sd0;
                            end else begin
                                //------- 梯形速度曲线轨迹发生器（2026-10-04，赛题拓展 2）-------
                                // 具体算式在下面的组合区（vel_nxt / cmd_nxt），这里只落寄存器：
                                //   vel_nxt：每 50ms 朝目标方向加/减一档 TRAJ_ASTEP（限 TRAJ_VMAX），
                                //            一旦"剩余距离 <= 从当前速度减速到 0 所需的距离"就转为减速
                                //   cmd_nxt：pos_cmd 按当前速度推进，并用 pos_set 夹住（不会过冲）
                                traj_vel <= vel_nxt;
                                pos_cmd  <= cmd_nxt;
                            end
                        end
                        // 串级耦合：位置环输出偏置角度环目标
                        //【2026-10-03 已恢复】屏蔽位置环的排查已完成：
                        //   横杆"加速转圈"确实是位置环引起的，根因是编码器计数方向与电机
                        //   方向不匹配（位置环变正反馈）。修法在 top.v —— encoder_if 的
                        //   A/B 相例化已对调，这里把串级耦合接回来。
                        //【2026-10-04】再叠加"摩擦前馈" pos_ff（运动方向上固定的工作点偏置），
                        //  治匀速/慢速运动的粘滑顿挫（见上面 pos_ff 段的说明）。
                        ang_target <= c_s - pos_out_w - pos_ff;
                    end
                end

                default: begin
                    run_state <= S_STOP;
                    motor_cmd <= 16'sd0;
                end
            endcase
        end
    end

    //----------------- 两环例化（2026-10-06 起用桌面版 PID 模块，它们没有 active/ki_en 端口）-----------------
    pid_angle u_pid_angle(
        .clk      (clk),
        .rst_n    (rst_n),
        .calc_en  (ang_calc),
        .target   (ang_target),
        .angle    (angle),
        .clr      (ang_clr),
        .out      (ang_out_w)
    );

    pid_pos u_pid_pos(
        .clk      (clk),
        .rst_n    (rst_n),
        .calc_en  (pos_calc),
        .target   (pos_target),
        .location (location),
        .clr      (pos_clr),
        .active   (motion_cmd),     // 【2026-10-06】运动态（走轨迹 / 手动移动）：位置环积分只在这时参与
        .out      (pos_out_w)
    );

    //----------------- 调试输出 ----------------
    assign angle_out   = ang_out_w;
    assign pos_out     = pos_out_w;
    // 位置环的目标 = 轨迹发生器输出（2026-10-04 起；此前是 K2/K3 直接阶跃写 pos_target）
    assign pos_target  = pos_cmd;
    // 给 OLED 显示用：目标位置 / 横杆速度（50ms 位置差分 ×20 = count/s）
    assign pos_set_out = pos_set;
    assign target_vel  = (run_state != S_PID) ? 32'sd0 :
                         ev_jog_plus  ? JOG_STEP * 32'sd20 :
                         ev_jog_minus ? -JOG_STEP * 32'sd20 : traj_vel * 32'sd20;
    // 给 motor_pwm 选死区档：运动时用小死区（让角度环的小输出真的推得动电机）
    assign mov_active  = pos_active;

endmodule

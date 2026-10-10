//============================================================================
// pendulum_cfg.vh —— 与"编码器分辨率（count/圈）"有关的常数唯一出处
//
// 为什么要集中：位置环 PID、轨迹速度/加速度、点动步长、OLED 换算，全都作用在
// "位置 count"这个域上。count/圈 一变，它们的等效值就整体偏移，必须按同一比例
// 跟着改。集中到本文件后：
//
//   【换电机 / 到货标定】
//     1. 选电机：把下面 CFG_MOTOR_OLD 那行注释掉，改成 CFG_MOTOR_JGA25
//        （或直接改对应分支里 CFG_CNT_PER_REV 的数值为实测值）
//     2. 重新构建： "E:\Gowin\Gowin_V1.9.10.02_x64\IDE\bin\gw_sh.exe" build_gw.tcl
//     3. 其它 .v 一律不用动 —— 派生量全部按 CFG_CNT_PER_REV 自动算
//
// 实测 count/圈的笨办法：手转输出轴整一圈，读 OLED 第 2 行 L 的前后差
// （L 是原始累计 count，不回绕）。
//============================================================================
`ifndef PENDULUM_CFG_VH
`define PENDULUM_CFG_VH

//----------------------------------------------------------------------------
// ① 选电机（换电机只改这一段）
//----------------------------------------------------------------------------
//`define CFG_MOTOR_OLD        // 旧电机：408 count/圈（11 线霍尔，减速比约 9.3）
`define CFG_MOTOR_JGA25      // 新电机：JGA25-370 280rpm 21.3:1 + 11 线霍尔【2026-10-09 换上】

`ifdef CFG_MOTOR_JGA25
    `define CFG_CNT_PER_REV  937    // JGA25-370 280rpm 21.3:1：11 线 x 4 倍频 x 21.3 = 937.2
                                    //   已确认：11 线、AB 两相正交（encoder_if.v 的四倍频
                                    //   状态机可直接用）、编码器 3.3V 电平（可直连 FPGA IO）、
                                    //   电机额定 12V。
                                    //   现场唯一要核的是实测 count/圈（标称减速比偏差可达
                                    //   38%），不符就改上面这个数字。
`else
    `define CFG_CNT_PER_REV  408    // 11 线霍尔，减速比 约 9.3
`endif

// 位置环系数是按这个分辨率整定的基准（勿改）
`define CFG_REF_CNT_PER_REV  408

//----------------------------------------------------------------------------
// ② 派生量（全部自动跟随 CFG_CNT_PER_REV，不要单独改）
//    注：Verilog 里无基数的十进制常量是有符号数，可直接参与有符号运算
//----------------------------------------------------------------------------

// --- 位置移动（ctrl_fsm.v）---
`define CFG_POS_STEP      (`CFG_CNT_PER_REV)                                    // K2/K3 一次 = 360 度 = 1 圈
`define CFG_POS_LIMIT     (`CFG_CNT_PER_REV * 10)                               // 位置目标限幅 ±10 圈
`define CFG_TRAJ_VMAX     ((`CFG_CNT_PER_REV * 211 + 3600) / 7200)              // 轨迹最高速【2026-10-10 用户要求 +50%：141 -> 211 度/s（937 档 = 27 count/50ms）】
`define CFG_TRAJ_ASTEP    ((`CFG_CNT_PER_REV + `CFG_REF_CNT_PER_REV / 2) / `CFG_REF_CNT_PER_REV)   // 每档加速度（保持约 8.8 度/s^2）
`define CFG_JOG_STEP      ((8 * `CFG_CNT_PER_REV + `CFG_REF_CNT_PER_REV / 2) / `CFG_REF_CNT_PER_REV)  // 长按点动【2026-10-10 用户要求 x2：约 141 度/s（937 档 = 18 count/50ms，一圈约 2.6s）】

// --- 摩擦前馈（ctrl_fsm.v）---
//   它加在"角度环目标"上，单位是角度 count（不是位置 count），所以**不随分辨率缩放**。
//   但新电机的静摩擦/工作点完全不同，到货后要重新整定：
//   匀速顿挫 -> 往大调（4~6）；横杆跑偏或到位过冲 -> 往小调（1~2）。
`define CFG_POS_FF        3

// --- 位置环 PID（pid_pos.v，Q16 定点）---
//   分辨率提高 N 倍，同一物理误差对应的 count 误差就大 N 倍，系数必须除 N，
//   否则等效增益凭空变硬 N 倍（会振荡）。
`define CFG_POS_KP_Q16    ((26214 * `CFG_REF_CNT_PER_REV + `CFG_CNT_PER_REV / 2) / `CFG_CNT_PER_REV)    // 0.4
`define CFG_POS_KD_Q16    ((262144 * `CFG_REF_CNT_PER_REV + `CFG_CNT_PER_REV / 2) / `CFG_CNT_PER_REV)   // 4.0
`define CFG_POS_KI_Q16    ((6554 * `CFG_REF_CNT_PER_REV + `CFG_CNT_PER_REV / 2) / `CFG_CNT_PER_REV)     // 0.1（代码保留，I_MAX=0 已关闭）
`define CFG_POS_DEADZONE  ((`CFG_CNT_PER_REV + `CFG_REF_CNT_PER_REV / 2) / `CFG_REF_CNT_PER_REV)        // 输出死区，按同一噪声 count 数折算

// --- OLED 显示（oled_ssd1306.v）---
//   deg_x10 = location * CFG_DEG_K >>> 10。向上取整，保证转满 1 圈显示 360 度。
//   （408 时算得 9036，原硬编码是 9039，两者显示差 <= 0.1 度）
`define CFG_DEG_K         ((3686400 + `CFG_CNT_PER_REV - 1) / `CFG_CNT_PER_REV)

`endif // PENDULUM_CFG_VH

//============================================================================
// oled_ssd1306.v --- 0.96 寸 OLED（SSD1306，128x64，4 针 I2C）
// 对应 STM32 版：江协 OLED 驱动 + main.c 的显示内容
// 【当前版本 = 四行演示显示】（2026-10-04 改成演示用四项）：
//   page0  CNT xxxx  ST xx       归一化后的 10bit 码(angle>>2) / 状态机状态
//   page2  L ±xxxx  V ±xxxx      横杆位置（编码器 count）/ 横杆速度（count/s，50ms 位置差分）
//   page4  DEG ±xxxx  POS ±xxxx  横杆实际位移换算的轮子转角(整数度，不回绕) / 目标位置(count)
//   page6  ANG xxx.x  CT xxxx    传感器绝对角度 0.0~333.3 度 / 当前平衡点
//   旧版四行（VIN/IN、CNT/ANG、RAW/AVG、ST/CT/L）见 oled_ssd1306.v.bak24
// angle 是 adc_if.v 归一化后的值：传感器 0 度 -> 0，333.3 度 -> ~4095
// 换算（新板：单 +3V、反相 0.1x 前端、REFSENSE 接 VREF ⇒ 1V 量程）：
//   AIN(mV) = (6282 - angle) × 1000/12288
//   Vin(mV) = angle × 625/768（= 0.8138 mV/count ⇒ angle 4059 ↔ 3.303V）
//   ANG(0.1 度) = angle × 833 >> 10（0.0814 度/count，满量程 3331 = 333.1 度）
// 要回到 ARM/ROD 两行显示：用 oled_ssd1306.v.bak9
// 硬件：SCL 推挽输出；SDA 开漏（写 0 拉低 / 写 1 释放高阻，靠模块上拉）
//       从机地址 0x78（0x3C<<1），页寻址模式；每帧只刷 page0/2/4/6 四行，
//       上电后第一轮刷全部 8 页（0~7）把显存里随机的上电值清掉，否则会满屏白点
// 字库：只做用到的字符（5x7 点阵放在 6x8 格内）
//============================================================================
module oled_ssd1306 #(
    parameter CLK_HZ = 50000000,
    parameter SCL_HZ = 400000
)(
    input  wire               clk,
    input  wire               rst_n,
    input  wire signed [31:0] location,      // 横杆位置(count)，408count = 360 度
    input  wire [11:0]        angle,         // 摆杆角度值 0~4095
    input  wire [11:0]        center_angle,  // 竖直平衡点角度值
    input  wire signed [31:0] pos_set,       // 【演示】目标位置(count)
    input  wire signed [31:0] bar_vel,       // 【演示】横杆速度(count/s)
    input  wire [9:0]         raw_code,      // 【保留未用】ADC 原码 0~1023
    input  wire [11:0]        avg4,          // 【保留未用】滑窗均值后的取反码 0~4092
    input  wire [5:0]         run_state,     // 控制状态机状态 0~34
    output reg                scl,
    inout  wire               sda
);

    //----------------- I2C 位节拍（每个 SCL 位 = 3 个 tick）-----------------
    // 向上取整（原来是整数截断：50MHz/1.2MHz = 41 -> SCL = 406.5kHz，超 400kHz 规格）
    // 取整后：50MHz/(42*3) = 396.8kHz，落在 SSD1306 的 400kHz 规格内
    localparam integer DIV = (CLK_HZ + SCL_HZ * 3 - 1) / (SCL_HZ * 3);

    reg [9:0] div_cnt;
    wire      tick = (div_cnt == DIV[9:0] - 10'd1);

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n)    div_cnt <= 10'd0;
        else if (tick) div_cnt <= 10'd0;
        else           div_cnt <= div_cnt + 10'd1;
    end

    //----------------- 数值换算：统一用 0.1 度为单位 -----------------
    // 横杆：超过一圈（408 count = 360 度）自动清零回绕，屏幕永远在 -360 ~ +360 度内；
    //       deg*10 = location * 3600 / 408 约等于 (location * 9039) >>> 10（误差 0.04%）
    //       只影响显示：位置环、串口上报仍用原始 location
    //       Verilog 的 % 取余符号跟随被除数，所以 +410 -> +2、-410 -> -2，双向回绕都正确
    wire signed [31:0] loc_wrap = location % 32'sd408;      // -407 ~ +407
    wire signed [31:0] arm_v    = (loc_wrap * 32'sd9039) >>> 10;
    // 摆杆：1 count 约 0.1 度（相对平衡点）
    wire signed [31:0] rod_v   = $signed({20'd0, angle}) - $signed({20'd0, center_angle});

    // 与 STM32 版 %+6.1f 的显示范围对齐；ARM 取模后最多 ±359.1 度，本项主要给 ROD 兜底
    function signed [31:0] sat36000;        // 饱和到 +/-36000（即 +/-3600.0 度）
        input signed [31:0] v;
        begin
            if      (v >  32'sd36000) sat36000 =  32'sd36000;
            else if (v < -32'sd36000) sat36000 = -32'sd36000;
            else                      sat36000 = v;
        end
    endfunction

    // 4 位十进制拆位：0~9999 -> {千,百,十,个}（各 4bit）
    // 【2026-10-06】改用窄位宽（14bit）级联 /10 替代 32bit 常数除法 —— 每个 32bit 常数
    //   除法综合时会展开成一个宽乘法器，本文件十几组拆位叠起来是综合耗时的大头。
    function [15:0] bcd4;
        input [13:0] v;
        reg   [13:0] t1, t2, t3;
        reg   [17:0] m1, m2, m3;
        reg   [13:0] e0, e1, e2;
        begin
            t1   = v  / 14'd10;
            t2   = t1 / 14'd10;
            t3   = t2 / 14'd10;
            m1   = t1 * 4'd10;
            m2   = t2 * 4'd10;
            m3   = t3 * 4'd10;
            e0   = v  - m1[13:0];
            e1   = t1 - m2[13:0];
            e2   = t2 - m3[13:0];
            bcd4 = {t3[3:0], e2[3:0], e1[3:0], e0[3:0]};
        end
    endfunction

    wire signed [31:0] arm_val = sat36000(arm_v);
    wire signed [31:0] rod_val = sat36000(rod_v);

    wire        arm_neg  = arm_val < 0;
    wire [31:0] arm_absw = arm_neg ? (-arm_val) : arm_val;   // 0~36000
    wire [15:0] arm_abs  = arm_absw[15:0];
    wire [15:0] arm_i_w  = arm_abs / 16'd10;                 // 整数部分 0~3600
    wire [13:0] arm_i    = arm_i_w[13:0];
    wire [15:0] arm_f_w  = arm_abs % 16'd10;                 // 小数位
    wire [3:0]  arm_f    = arm_f_w[3:0];
    wire [15:0] arm_bcd  = bcd4(arm_i);                      // arm_i ≤ 3600
    wire [3:0]  arm_d3   = arm_bcd[15:12];                   // 千
    wire [3:0]  arm_d2   = arm_bcd[11:8];                    // 百
    wire [3:0]  arm_d1   = arm_bcd[7:4];                     // 十
    wire [3:0]  arm_d0   = arm_bcd[3:0];                     // 个

    wire        rod_neg  = rod_val < 0;
    wire [31:0] rod_absw = rod_neg ? (-rod_val) : rod_val;   // 0~36000
    wire [15:0] rod_abs  = rod_absw[15:0];
    wire [15:0] rod_i_w  = rod_abs / 16'd10;
    wire [13:0] rod_i    = rod_i_w[13:0];
    wire [15:0] rod_f_w  = rod_abs % 16'd10;
    wire [3:0]  rod_f    = rod_f_w[3:0];
    wire [15:0] rod_bcd  = bcd4(rod_i);
    wire [3:0]  rod_d3   = rod_bcd[15:12];
    wire [3:0]  rod_d2   = rod_bcd[11:8];
    wire [3:0]  rod_d1   = rod_bcd[7:4];
    wire [3:0]  rod_d0   = rod_bcd[3:0];

    //----------------- 两行字符（ASCII，每行 10 个有效字符，其余补空格）-----------------
    function [6:0] arm_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0: arm_char = 7'h41;                     // 'A'
                5'd1: arm_char = 7'h52;                     // 'R'
                5'd2: arm_char = 7'h4D;                     // 'M'
                5'd3: arm_char = 7'h20;                     // ' '
                5'd4: arm_char = arm_neg ? 7'h2D : 7'h2B;   // '-' / '+'
                5'd5: arm_char = 7'h30 + {3'd0, arm_d3};
                5'd6: arm_char = 7'h30 + {3'd0, arm_d2};
                5'd7: arm_char = 7'h30 + {3'd0, arm_d1};
                5'd8: arm_char = 7'h30 + {3'd0, arm_d0};
                5'd9: arm_char = 7'h2E;                     // '.'
                5'd10: arm_char = 7'h30 + {3'd0, arm_f};
                default: arm_char = 7'h20;
            endcase
        end
    endfunction

    function [6:0] rod_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0: rod_char = 7'h52;                     // 'R'
                5'd1: rod_char = 7'h4F;                     // 'O'
                5'd2: rod_char = 7'h44;                     // 'D'
                5'd3: rod_char = 7'h20;
                5'd4: rod_char = rod_neg ? 7'h2D : 7'h2B;
                5'd5: rod_char = 7'h30 + {3'd0, rod_d3};
                5'd6: rod_char = 7'h30 + {3'd0, rod_d2};
                5'd7: rod_char = 7'h30 + {3'd0, rod_d1};
                5'd8: rod_char = 7'h30 + {3'd0, rod_d0};
                5'd9: rod_char = 7'h2E;
                5'd10: rod_char = 7'h30 + {3'd0, rod_f};
                default: rod_char = 7'h20;
            endcase
        end
    endfunction

    //----------------- 5x7 点阵（每字节一列，bit0 = 最上一行）-----------------
    function [7:0] glyph;
        input [6:0] code;
        input [2:0] col;
        reg [7:0] c0, c1, c2, c3, c4;
        begin
            case (code)
                7'h20: begin c0=8'h00; c1=8'h00; c2=8'h00; c3=8'h00; c4=8'h00; end // 空格
                7'h2B: begin c0=8'h08; c1=8'h08; c2=8'h3E; c3=8'h08; c4=8'h08; end // +
                7'h2D: begin c0=8'h08; c1=8'h08; c2=8'h08; c3=8'h08; c4=8'h08; end // -
                7'h2E: begin c0=8'h00; c1=8'h60; c2=8'h60; c3=8'h00; c4=8'h00; end // .
                7'h30: begin c0=8'h3E; c1=8'h51; c2=8'h49; c3=8'h45; c4=8'h3E; end // 0
                7'h31: begin c0=8'h00; c1=8'h42; c2=8'h7F; c3=8'h40; c4=8'h00; end // 1
                7'h32: begin c0=8'h42; c1=8'h61; c2=8'h51; c3=8'h49; c4=8'h46; end // 2
                7'h33: begin c0=8'h21; c1=8'h41; c2=8'h45; c3=8'h4B; c4=8'h31; end // 3
                7'h34: begin c0=8'h18; c1=8'h14; c2=8'h12; c3=8'h7F; c4=8'h10; end // 4
                7'h35: begin c0=8'h27; c1=8'h45; c2=8'h45; c3=8'h45; c4=8'h39; end // 5
                7'h36: begin c0=8'h3C; c1=8'h4A; c2=8'h49; c3=8'h49; c4=8'h30; end // 6
                7'h37: begin c0=8'h01; c1=8'h71; c2=8'h09; c3=8'h05; c4=8'h03; end // 7
                7'h38: begin c0=8'h36; c1=8'h49; c2=8'h49; c3=8'h49; c4=8'h36; end // 8
                7'h39: begin c0=8'h06; c1=8'h49; c2=8'h29; c3=8'h29; c4=8'h1E; end // 9
                7'h41: begin c0=8'h7E; c1=8'h11; c2=8'h11; c3=8'h11; c4=8'h7E; end // A
                7'h44: begin c0=8'h7F; c1=8'h41; c2=8'h41; c3=8'h22; c4=8'h1C; end // D
                7'h4D: begin c0=8'h7F; c1=8'h02; c2=8'h0C; c3=8'h02; c4=8'h7F; end // M
                7'h4F: begin c0=8'h3E; c1=8'h41; c2=8'h41; c3=8'h41; c4=8'h3E; end // O
                7'h52: begin c0=8'h7F; c1=8'h09; c2=8'h19; c3=8'h29; c4=8'h46; end // R
                7'h43: begin c0=8'h3E; c1=8'h41; c2=8'h41; c3=8'h41; c4=8'h22; end // C
                7'h49: begin c0=8'h00; c1=8'h41; c2=8'h7F; c3=8'h41; c4=8'h00; end // I
                7'h4E: begin c0=8'h7F; c1=8'h04; c2=8'h08; c3=8'h10; c4=8'h7F; end // N
                7'h54: begin c0=8'h01; c1=8'h01; c2=8'h7F; c3=8'h01; c4=8'h01; end // T
                7'h56: begin c0=8'h1F; c1=8'h20; c2=8'h40; c3=8'h20; c4=8'h1F; end // V
                7'h47: begin c0=8'h3E; c1=8'h41; c2=8'h41; c3=8'h51; c4=8'h32; end // G
                7'h53: begin c0=8'h46; c1=8'h49; c2=8'h49; c3=8'h49; c4=8'h31; end // S
                7'h57: begin c0=8'h7F; c1=8'h20; c2=8'h18; c3=8'h20; c4=8'h7F; end // W
                7'h45: begin c0=8'h7F; c1=8'h49; c2=8'h49; c3=8'h49; c4=8'h41; end // E
                7'h58: begin c0=8'h63; c1=8'h14; c2=8'h08; c3=8'h14; c4=8'h63; end // X
                7'h4C: begin c0=8'h7F; c1=8'h40; c2=8'h40; c3=8'h40; c4=8'h40; end // L
                7'h50: begin c0=8'h7F; c1=8'h09; c2=8'h09; c3=8'h09; c4=8'h06; end // P
                default: begin c0=8'h00; c1=8'h00; c2=8'h00; c3=8'h00; c4=8'h00; end
            endcase
            case (col)
                3'd0:    glyph = c0;
                3'd1:    glyph = c1;
                3'd2:    glyph = c2;
                3'd3:    glyph = c3;
                3'd4:    glyph = c4;
                default: glyph = 8'h00;      // 第 6 列留空，作字间距
            endcase
        end
    endfunction

    //----------------- 初始化命令表（SSD1306，128x64，页寻址）-----------------
    localparam integer N_INIT = 25;

    reg [7:0] cmd_idx;      // 初始化命令索引
    reg [7:0] init_cmd;
    always @(*) begin
        case (cmd_idx)
            8'd0:  init_cmd = 8'hAE;    // 关显示
            8'd1:  init_cmd = 8'hD5;    // 显示时钟分频
            8'd2:  init_cmd = 8'h80;
            8'd3:  init_cmd = 8'hA8;    // 多路复用率
            8'd4:  init_cmd = 8'h3F;
            8'd5:  init_cmd = 8'hD3;    // 显示偏移
            8'd6:  init_cmd = 8'h00;
            8'd7:  init_cmd = 8'h40;    // 起始行
            8'd8:  init_cmd = 8'h8D;    // 充电泵
            8'd9:  init_cmd = 8'h14;
            8'd10: init_cmd = 8'h20;    // 页寻址模式
            8'd11: init_cmd = 8'h02;
            8'd12: init_cmd = 8'hA1;    // 段重映射
            8'd13: init_cmd = 8'hC8;    // 行扫描方向
            8'd14: init_cmd = 8'hDA;    // COM 硬件配置
            8'd15: init_cmd = 8'h12;
            8'd16: init_cmd = 8'h81;    // 对比度
            8'd17: init_cmd = 8'hCF;
            8'd18: init_cmd = 8'hD9;    // 预充电周期
            8'd19: init_cmd = 8'hF1;
            8'd20: init_cmd = 8'hDB;    // VCOMH
            8'd21: init_cmd = 8'h30;
            8'd22: init_cmd = 8'hA4;    // 正常显示
            8'd23: init_cmd = 8'hA6;    // 不反色
            default: init_cmd = 8'hAF;  // 开显示
        endcase
    end

    //----------------- 主状态机 ----------------
    localparam [2:0]
        S_PWR  = 3'd0,   // 上电延时
        S_TXN  = 3'd1,   // 起始条件
        S_DATA = 3'd2,   // 逐字节发送
        S_STOP = 3'd3,   // 停止条件
        S_NEXT = 3'd4,   // 决定下一个事务
        S_WAIT = 3'd5;   // 帧间延时

    localparam [1:0]
        T_INIT = 2'd0,   // 初始化命令
        T_PSET = 2'd1,   // 设置页/列地址
        T_PDAT = 2'd2;   // 页数据

    reg [2:0]  st;
    reg [1:0]  txn;
    reg [2:0]  row;           // 当前页号 0~7（page0 = 第 1 行，page2 = 第 3 行，其余页只用于清屏）
    reg        all_pages;     // 1 = 上电后的第一轮：刷全部 0~7 页（清除上电时随机的显存）
    reg [7:0]  byte_idx;      // 事务内字节索引
    reg [5:0]  ci;            // 字符索引
    reg [2:0]  cc;            // 字符内列
    reg [3:0]  bit_idx;       // 0..8（8 个数据位 + 1 个 ACK 位）
    reg [1:0]  ph;            // 位相位
    reg        sda_low;       // 1 = 拉低 SDA
    reg [19:0] delay_cnt;

    wire [7:0] txn_len = (txn == T_INIT) ? 8'd3 :
                         (txn == T_PSET) ? 8'd5 : 8'd130;   // 页数据 = 2 字节头 + 128 列

    //================= 临时调试：四行实时 AD / 控制量显示 =================
    // angle = adc_if.v 归一化后的角度（0~4095 对应传感器 0~333.3 度）
    wire [9:0]  ad_cnt  = angle >> 2;                        // 归一化后的 10bit 码 0~1023
    // AIN 与原始码成正比：原码 = 523.5 - angle/12，AIN = 原码/1024×1000mV
    wire [12:0] ad_x23  = 13'd6282 - angle;                  // 2187~6282
    wire [22:0] ad_mv_w = (ad_x23 * 13'd1000) / 23'd12288;   // AIN 电压（mV）0~511
    wire [9:0]  ad_mv   = ad_mv_w[9:0];
    // 反推到传感器接口 H2-2 的输入电压（mV）：Vin = angle × 625/768（0.8138 mV/count）
    wire [23:0] in_mv_w = (angle * 12'd625) / 24'd768;       // 0~3332
    wire [12:0] in_mv   = in_mv_w[12:0];
    wire               in_neg = 1'b0;                        // 上式恒正（保留原名兼容下游）
    wire [12:0]        in_abs = in_mv;

    wire [9:0]  a_i   = ad_mv / 10'd1000;                    // AIN 整数位 0~1
    wire [9:0]  a_f   = ad_mv % 10'd1000;                    // AIN 小数 0~999
    wire [12:0] b_i_w = in_abs / 13'd1000;                   // 输入整数位 0~3
    wire [9:0]  b_i   = b_i_w[9:0];
    wire [12:0] b_f_w = in_abs % 13'd1000;                   // 输入小数 0~999
    wire [9:0]  b_f   = b_f_w[9:0];
    wire [15:0] c_bcd = bcd4({4'b0, ad_cnt});                // 原始码 0~1023
    wire [3:0]  c_d3  = c_bcd[15:12];                        // 千位

    // 小数百位
    wire [9:0]  a_f_h = a_f / 10'd100;
    wire [9:0]  b_f_h = b_f / 10'd100;
    wire [3:0]  c_d2 = c_bcd[11:8];
    wire [3:0]  c_d1 = c_bcd[7:4];
    wire [3:0]  c_d0 = c_bcd[3:0];

    //----------------- 2026-10-02 新增：ANG / RAW / AVG / STA / CMD 的拆位 ----------------
    // ANG：传感器绝对角度（单位 0.1 度）。4095 count ↔ 333.3 度 ⇒ 0.0814 度/count，
    //      用 x833>>10（=0.8135，即 0.1 度/count）代替 32bit 除法，满量程 3331（333.1 度）
    wire [11:0] ang_x10 = (angle * 12'd833) >> 10;               // 0~3331
    wire [11:0] ang_iv_w = ang_x10 / 12'd10;                     // 整数度 0~333
    wire [8:0]  ang_iv   = ang_iv_w[8:0];
    wire [11:0] ang_f_w  = ang_x10 % 12'd10;                     // 小数 0.1 度
    wire [3:0]  ang_f    = ang_f_w[3:0];
    wire [15:0] ang_bcd = bcd4({5'b0, ang_iv});                  // ang_iv ≤ 333
    wire [3:0]  ang_i3  = ang_bcd[11:8];                         // 百
    wire [3:0]  ang_i2  = ang_bcd[7:4];                          // 十
    wire [3:0]  ang_i1  = ang_bcd[3:0];                          // 个

    // RAW：ADC 原码（10bit，0~1023）
    wire [15:0] r_bcd = bcd4({4'b0, raw_code});
    wire [3:0]  r_d3 = r_bcd[15:12];
    wire [3:0]  r_d2 = r_bcd[11:8];
    wire [3:0]  r_d1 = r_bcd[7:4];
    wire [3:0]  r_d0 = r_bcd[3:0];

    // AVG：8 次平均 + 4 点滑窗后的取反码（0~4092）
    wire [15:0] v_bcd = bcd4(avg4[11:0]);
    wire [3:0]  v_d3 = v_bcd[15:12];
    wire [3:0]  v_d2 = v_bcd[11:8];
    wire [3:0]  v_d1 = v_bcd[7:4];
    wire [3:0]  v_d0 = v_bcd[3:0];

    // STA：状态机状态（0~34）
    wire [15:0] s_bcd = bcd4({8'b0, run_state});
    wire [3:0]  s_d1 = s_bcd[7:4];
    wire [3:0]  s_d0 = s_bcd[3:0];

    // 【调试】CT：当前平衡点（center_angle）的 CNT 尺度值，与 page2 的 CNT 同口径，方便对照
    wire [9:0]  ct_v  = ({20'd0, center_angle}) >> 2;   // 0~1023
    wire [15:0] ct_bcd = bcd4({4'b0, ct_v});
    wire [3:0]  ct_d3 = ct_bcd[15:12];
    wire [3:0]  ct_d2 = ct_bcd[11:8];
    wire [3:0]  ct_d1 = ct_bcd[7:4];
    wire [3:0]  ct_d0 = ct_bcd[3:0];

    // 【调试】LOC：横杆位置（编码器 count），带符号显示，超过 ±9999 就限幅
    //   用途：停止态用手转动横杆，看 LOC 会不会跟着变 —— 判断编码器有没有信号
    //   （位置环"没在工作"的两种可能：没信号 / 方向反，前者会让 LOC 恒定不动）
    wire               loc_neg = (location < 32'sd0);
    wire [31:0]        loc_abs = loc_neg ? (-location) : location;
    wire [13:0]        loc_lim = (loc_abs > 32'd9999) ? 14'd9999 : loc_abs[13:0];
    wire [15:0]        lc_bcd  = bcd4(loc_lim);
    wire [3:0]  lc_d3 = lc_bcd[15:12];
    wire [3:0]  lc_d2 = lc_bcd[11:8];
    wire [3:0]  lc_d1 = lc_bcd[7:4];
    wire [3:0]  lc_d0 = lc_bcd[3:0];

    //----------------- 2026-10-04：演示用三项的拆位（速度 / 位移角度 / 目标位置）-----------------
    // V：横杆速度（count/s，来自 ctrl_fsm 的 bar_vel），带符号，限幅 ±9999
    wire        bv_neg = (bar_vel < 32'sd0);
    wire [31:0] bv_abs = bv_neg ? (-bar_vel) : bar_vel;
    wire [13:0] bv_lim = (bv_abs > 32'd9999) ? 14'd9999 : bv_abs[13:0];
    wire [15:0] vv_bcd = bcd4(bv_lim);
    wire [3:0]  vv_d3  = vv_bcd[15:12];
    wire [3:0]  vv_d2  = vv_bcd[11:8];
    wire [3:0]  vv_d1  = vv_bcd[7:4];
    wire [3:0]  vv_d0  = vv_bcd[3:0];

    // DEG：横杆"实际位移"换算成轮子转角（整数度，408 count = 360 度）。
    //   x9039>>10 -> 0.1 度（与旧 arm_v 同系数），再 /10 取整度。
    //   ⚠ 与旧 arm_v 的区别：这里 **不做一圈回绕**，所以按一次 K2 能看到 360 一直累加，
    //     而不是到 360 就跳回 0（演示"走了多少度"要的就是累计值）。
    wire signed [31:0] deg_x10 = (location * 32'sd9039) >>> 10;   // 0.1 度
    wire signed [31:0] deg_v   = deg_x10 / 32'sd10;               // 整数度
    wire        dg_neg = (deg_v < 32'sd0);
    wire [31:0] dg_abs = dg_neg ? (-deg_v) : deg_v;
    wire [13:0] dg_lim = (dg_abs > 32'd9999) ? 14'd9999 : dg_abs[13:0];
    wire [15:0] de_bcd = bcd4(dg_lim);
    wire [3:0]  de_d3  = de_bcd[15:12];
    wire [3:0]  de_d2  = de_bcd[11:8];
    wire [3:0]  de_d1  = de_bcd[7:4];
    wire [3:0]  de_d0  = de_bcd[3:0];

    // POS：目标位置 pos_set（带符号 count，限幅 ±9999）
    wire        ps_neg = (pos_set < 32'sd0);
    wire [31:0] ps_abs = ps_neg ? (-pos_set) : pos_set;
    wire [13:0] ps_lim = (ps_abs > 32'd9999) ? 14'd9999 : ps_abs[13:0];
    wire [15:0] p_bcd  = bcd4(ps_lim);
    wire [3:0]  p_d3   = p_bcd[15:12];
    wire [3:0]  p_d2   = p_bcd[11:8];
    wire [3:0]  p_d1   = p_bcd[7:4];
    wire [3:0]  p_d0   = p_bcd[3:0];

    //----------------- page0：CNT xxxx  ST xx（归一化码 / 状态机状态）-----------------
    function [6:0] ln0_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  ln0_char = 7'h43;                                    // 'C'
                5'd1:  ln0_char = 7'h4E;                                    // 'N'
                5'd2:  ln0_char = 7'h54;                                    // 'T'
                5'd3:  ln0_char = 7'h20;
                5'd4:  ln0_char = 7'h30 + {3'd0, c_d3};
                5'd5:  ln0_char = 7'h30 + {3'd0, c_d2};
                5'd6:  ln0_char = 7'h30 + {3'd0, c_d1};
                5'd7:  ln0_char = 7'h30 + {3'd0, c_d0};
                5'd8:  ln0_char = 7'h20;
                5'd9:  ln0_char = 7'h20;
                5'd10: ln0_char = 7'h53;                                    // 'S'
                5'd11: ln0_char = 7'h54;                                    // 'T'
                5'd12: ln0_char = 7'h20;
                // 状态 0~9 时十位补空格，避免出现 "ST 04" 以外的 "ST 4" 之类歧义排版
                5'd13: ln0_char = (run_state >= 6'd10) ? (7'h30 + {3'd0, s_d1}) : 7'h20;
                5'd14: ln0_char = 7'h30 + {3'd0, s_d0};
                default: ln0_char = 7'h20;
            endcase
        end
    endfunction

    //----------------- page2：L ±xxxx  V ±xxxx（横杆位置 count / 横杆速度 count/s）-----------------
    function [6:0] ln2_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  ln2_char = 7'h4C;                                    // 'L'
                5'd1:  ln2_char = 7'h20;
                5'd2:  ln2_char = loc_neg ? 7'h2D : 7'h2B;                  // '-' / '+'
                5'd3:  ln2_char = 7'h30 + {3'd0, lc_d3};
                5'd4:  ln2_char = 7'h30 + {3'd0, lc_d2};
                5'd5:  ln2_char = 7'h30 + {3'd0, lc_d1};
                5'd6:  ln2_char = 7'h30 + {3'd0, lc_d0};
                5'd7:  ln2_char = 7'h20;
                5'd8:  ln2_char = 7'h20;
                5'd9:  ln2_char = 7'h56;                                    // 'V'
                5'd10: ln2_char = 7'h20;
                5'd11: ln2_char = bv_neg ? 7'h2D : 7'h2B;                   // '-' / '+'
                5'd12: ln2_char = 7'h30 + {3'd0, vv_d3};
                5'd13: ln2_char = 7'h30 + {3'd0, vv_d2};
                5'd14: ln2_char = 7'h30 + {3'd0, vv_d1};
                5'd15: ln2_char = 7'h30 + {3'd0, vv_d0};
                default: ln2_char = 7'h20;
            endcase
        end
    endfunction

    //----------------- page4：DEG ±xxxx  POS ±xxxx（横杆位移角度 / 目标位置）-----------------
    // DEG = 横杆实际位移换算的轮子转角（整数度，不回绕）：按一次 K2（+408 count）应看到 360
    // POS = 目标位置 pos_set（count）
    function [6:0] ln4_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  ln4_char = 7'h44;                                    // 'D'
                5'd1:  ln4_char = 7'h45;                                    // 'E'
                5'd2:  ln4_char = 7'h47;                                    // 'G'
                5'd3:  ln4_char = 7'h20;
                5'd4:  ln4_char = dg_neg ? 7'h2D : 7'h2B;                   // '-' / '+'
                5'd5:  ln4_char = 7'h30 + {3'd0, de_d3};
                5'd6:  ln4_char = 7'h30 + {3'd0, de_d2};
                5'd7:  ln4_char = 7'h30 + {3'd0, de_d1};
                5'd8:  ln4_char = 7'h30 + {3'd0, de_d0};
                5'd9:  ln4_char = 7'h20;
                5'd10: ln4_char = 7'h20;
                5'd11: ln4_char = 7'h50;                                    // 'P'
                5'd12: ln4_char = 7'h4F;                                    // 'O'
                5'd13: ln4_char = 7'h53;                                    // 'S'
                5'd14: ln4_char = 7'h20;
                5'd15: ln4_char = ps_neg ? 7'h2D : 7'h2B;                   // '-' / '+'
                5'd16: ln4_char = 7'h30 + {3'd0, p_d3};
                5'd17: ln4_char = 7'h30 + {3'd0, p_d2};
                5'd18: ln4_char = 7'h30 + {3'd0, p_d1};
                5'd19: ln4_char = 7'h30 + {3'd0, p_d0};
                default: ln4_char = 7'h20;
            endcase
        end
    endfunction

    //----------------- page6：ANG xxx.x  CT xxxx（传感器绝对角度 / 平衡点）-----------------
    // ANG：传感器绝对角度 0.0~333.3 度（百位/十位为 0 时补空格，避免出现 "033.3"）
    // CT ：当前平衡点（center_angle 的 CNT 尺度值），按 K4 标定后应等于当时的 CNT
    function [6:0] ln6_char;
        input [4:0] idx;
        begin
            case (idx)
                5'd0:  ln6_char = 7'h41;                                    // 'A'
                5'd1:  ln6_char = 7'h4E;                                    // 'N'
                5'd2:  ln6_char = 7'h47;                                    // 'G'
                5'd3:  ln6_char = 7'h20;
                5'd4:  ln6_char = (ang_i3 == 4'd0) ?
                                  7'h20 : (7'h30 + {3'd0, ang_i3});
                5'd5:  ln6_char = ((ang_i3 == 4'd0) && (ang_i2 == 4'd0)) ?
                                  7'h20 : (7'h30 + {3'd0, ang_i2});
                5'd6:  ln6_char = 7'h30 + {3'd0, ang_i1};
                5'd7:  ln6_char = 7'h2E;                                    // '.'
                5'd8:  ln6_char = 7'h30 + {3'd0, ang_f};
                5'd9:  ln6_char = 7'h20;
                5'd10: ln6_char = 7'h20;
                5'd11: ln6_char = 7'h43;                                    // 'C'
                5'd12: ln6_char = 7'h54;                                    // 'T'
                5'd13: ln6_char = 7'h20;
                5'd14: ln6_char = 7'h30 + {3'd0, ct_d3};
                5'd15: ln6_char = 7'h30 + {3'd0, ct_d2};
                5'd16: ln6_char = 7'h30 + {3'd0, ct_d1};
                5'd17: ln6_char = 7'h30 + {3'd0, ct_d0};
                default: ln6_char = 7'h20;
            endcase
        end
    endfunction

    // 每行两项（2026-10-04 改演示内容）：row=0 CNT/ST、row=2 L/V、row=4 DEG/POS、row=6 ANG/CT，
    // 其余页全空格（上电首轮刷 0~7 页清屏）
    wire [6:0] cur_ch   = (row == 3'd0) ? ln0_char(ci[4:0]) :
                          (row == 3'd2) ? ln2_char(ci[4:0]) :
                          (row == 3'd4) ? ln4_char(ci[4:0]) :
                          (row == 3'd6) ? ln6_char(ci[4:0]) : 7'h20;
    wire [7:0] cur_data = glyph(cur_ch, cc);

    reg [7:0] tx_byte;
    always @(*) begin
        case (txn)
            T_INIT: case (byte_idx)
                        8'd0:    tx_byte = 8'h78;            // 从机地址 + 写
                        8'd1:    tx_byte = 8'h00;            // 控制字节：命令
                        default: tx_byte = init_cmd;
                    endcase
            T_PSET: case (byte_idx)
                        8'd0:    tx_byte = 8'h78;
                        8'd1:    tx_byte = 8'h00;
                        8'd2:    tx_byte = 8'hB0 | {5'd0, row};             // 页地址 = row（0~7）
                        8'd3:    tx_byte = 8'h00;            // 列地址低 4 位
                        default: tx_byte = 8'h10;            // 列地址高 4 位
                    endcase
            default: case (byte_idx)
                        8'd0:    tx_byte = 8'h78;
                        8'd1:    tx_byte = 8'h40;            // 控制字节：数据
                        default: tx_byte = cur_data;
                    endcase
        endcase
    end

    wire bit_val = (bit_idx < 4'd8) ? tx_byte[7 - bit_idx] : 1'b1;   // 第 9 位是 ACK，释放 SDA

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            st        <= S_PWR;
            txn       <= T_INIT;
            cmd_idx   <= 8'd0;
            row       <= 3'd0;
            all_pages <= 1'b1;
            byte_idx  <= 8'd0;
            ci        <= 6'd0;
            cc        <= 3'd0;
            bit_idx   <= 4'd0;
            ph        <= 2'd0;
            scl       <= 1'b1;
            sda_low   <= 1'b0;
            delay_cnt <= 20'd0;
        end else if (tick) begin
            case (st)

            //---------- 上电延时（约 125ms）----------
            S_PWR: begin
                scl <= 1'b1;
                if (delay_cnt == 20'd15000) begin        // 【示波器观察用】原 152000(约127ms)，现约100ms
                    delay_cnt <= 20'd0;
                    st        <= S_TXN;
                end else begin
                    delay_cnt <= delay_cnt + 20'd1;
                end
            end

            //---------- 起始条件 ----------
            S_TXN: begin
                byte_idx <= 8'd0;
                bit_idx  <= 4'd0;
                case (ph)
                    2'd0:    begin scl <= 1'b1; sda_low <= 1'b0; ph <= 2'd1; end
                    2'd1:    begin sda_low <= 1'b1;              ph <= 2'd2; end
                    default: begin scl <= 1'b0; ph <= 2'd0; st <= S_DATA; end
                endcase
            end

            //---------- 逐字节发送（8 个数据位 + 1 个 ACK 位）----------
            S_DATA: begin
                case (ph)
                    2'd0: begin scl <= 1'b0; sda_low <= ~bit_val; ph <= 2'd1; end
                    2'd1: begin scl <= 1'b1;                      ph <= 2'd2; end
                    default: begin
                        scl     <= 1'b0;
                        bit_idx <= bit_idx + 4'd1;
                        ph      <= 2'd0;
                        if (bit_idx == 4'd8) begin
                            bit_idx  <= 4'd0;
                            byte_idx <= byte_idx + 8'd1;
                            // 页数据阶段：每 6 列换下一个字符
                            if ((txn == T_PDAT) && (byte_idx >= 8'd2)) begin
                                if (cc == 3'd5) begin cc <= 3'd0; ci <= ci + 6'd1; end
                                else                 cc <= cc + 3'd1;
                            end
                            if (byte_idx + 8'd1 >= txn_len) st <= S_STOP;
                        end
                    end
                endcase
            end

            //---------- 停止条件 ----------
            S_STOP: begin
                case (ph)
                    2'd0:    begin scl <= 1'b0; sda_low <= 1'b1; ph <= 2'd1; end
                    2'd1:    begin scl <= 1'b1;                  ph <= 2'd2; end
                    default: begin sda_low <= 1'b0; ph <= 2'd0; st <= S_NEXT; end
                endcase
            end

            //---------- 下一个事务 ----------
            S_NEXT: begin
                if (txn == T_INIT) begin
                    if (cmd_idx + 8'd1 >= N_INIT[7:0]) begin
                        cmd_idx   <= 8'd0;
                        txn       <= T_PSET;
                        row       <= 3'd0;
                        all_pages <= 1'b1;   // 上电后第一轮：刷 0~7 页清屏
                    end else begin
                        cmd_idx <= cmd_idx + 8'd1;
                    end
                    st <= S_TXN;
                end else if (txn == T_PSET) begin
                    txn <= T_PDAT;
                    ci  <= 6'd0;
                    cc  <= 3'd0;
                    st  <= S_TXN;
                end else begin
                    // 页数据发完，决定下一页
                    if (all_pages) begin
                        if (row == 3'd7) begin
                            all_pages <= 1'b0;      // 清屏轮结束，转入正常的三行刷新
                            row       <= 3'd0;
                            txn       <= T_PSET;
                            st        <= S_TXN;
                        end else begin
                            row <= row + 3'd1;
                            txn <= T_PSET;
                            st  <= S_TXN;
                        end
                    end else if (row == 3'd0) begin
                        row <= 3'd2;                // page0 -> page2
                        txn <= T_PSET;
                        st  <= S_TXN;
                    end else if (row == 3'd2) begin
                        row <= 3'd4;                // page2 -> page4
                        txn <= T_PSET;
                        st  <= S_TXN;
                    end else if (row == 3'd4) begin
                        row <= 3'd6;                // page4 -> page6
                        txn <= T_PSET;
                        st  <= S_TXN;
                    end else begin
                        row       <= 3'd0;
                        txn       <= T_PSET;
                        delay_cnt <= 20'd0;
                        st        <= S_WAIT;        // 一帧刷完，等下一帧
                    end
                end
            end

            //---------- 帧间延时（【示波器观察用】原 200ms，现约 10ms，让 I2C 波形基本连续）----------
            S_WAIT: begin
                if (delay_cnt == 20'd1500) begin         // 【示波器观察用】原 244000(约200ms)，现约10ms
                    delay_cnt <= 20'd0;
                    st        <= S_TXN;
                end else begin
                    delay_cnt <= delay_cnt + 20'd1;
                end
            end

            default: st <= S_PWR;
            endcase
        end
    end

    //----------------- SDA 开漏输出 ----------------
    assign sda = sda_low ? 1'b0 : 1'bz;

endmodule

//============================================================================
// key_ctrl.v --- 按键：20ms 消抖 + 松手单击 + 长按（点动电平）
// 对应 STM32 版：Key.c（20ms 扫描、松手生效）
// 现在是 4 个按键全部外接（top 的 key[3:0]），一键一功能，不再复用：
//   key[0] = K1 启停 / key[1] = K2 目标 ±360 度（短按）+ 点动（长按）
//   key[2] = K3 同 K2 反向 / key[3] = K4 标定平衡点
// 说明：
//   ev_single：松手才产生单击脉冲（与 STM32 版"松手生效"一致）
//   ev_hold  ：按住超过 HOLD_MS 后为高，松手立刻变低（给 ctrl_fsm 做"按住慢移"）
//   **进入过长按态的这次按压，松手时不再产生 ev_single** —— 这样"轻点一下"和
//   "按住不放"不会互相干扰。
//============================================================================
module key_ctrl #(
    parameter DB_MS   = 20,         // 消抖时间(ms)
    parameter HOLD_MS = 400         // 长按判定时间(ms)
)(
    input  wire clk,                // 50MHz 系统时钟
    input  wire rst_n,              // 低电平复位
    input  wire tick_1ms,           // 1ms 节拍
    input  wire key_in,             // 按键输入，低有效（按下 = 0）
    output reg  ev_single,          // 松手单击（单周期脉冲）
    output wire ev_hold             // 长按按住期间为高（点动电平）
);

    //----------------- 输入同步 ----------------
    reg k_s0, k_s1;
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            k_s0 <= 1'b1;
            k_s1 <= 1'b1;
        end else begin
            k_s0 <= key_in;
            k_s1 <= k_s0;
        end
    end

    //----------------- 双向消抖（按下/松手各需连续 DB_MS）-----------------
    reg [7:0] cnt_low, cnt_high;
    reg       key_down;             // 消抖后的按键状态：1 = 按下

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_low  <= 8'd0;
            cnt_high <= 8'd0;
            key_down <= 1'b0;
        end else if (tick_1ms) begin
            if (k_s1 == 1'b0) begin
                cnt_high <= 8'd0;
                if (cnt_low < DB_MS) cnt_low <= cnt_low + 8'd1;
            end else begin
                cnt_low <= 8'd0;
                if (cnt_high < DB_MS) cnt_high <= cnt_high + 8'd1;
            end

            if (cnt_low  >= DB_MS) key_down <= 1'b1;
            if (cnt_high >= DB_MS) key_down <= 1'b0;
        end
    end

    //----------------- 长按检测（按住 >= HOLD_MS 进入点动态，松手立刻退出）-----------------
    // 注意 hold_f 在松手那一拍仍是高（它在下一个 tick_1ms 才清零），所以下面判断
    // ev_single 时用 ~hold_f 就能正确吃掉"长按后的松手"，不会误发一次单击。
    reg [15:0] press_ms;
    reg        hold_f;

    localparam [15:0] HOLD_VAL = HOLD_MS;   // 门限（转成 16bit 便于和 press_ms 同宽比较）

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            press_ms <= 16'd0;
            hold_f   <= 1'b0;
        end else if (tick_1ms) begin
            if (key_down) begin
                if (press_ms < HOLD_VAL) press_ms <= press_ms + 16'd1;
                else                     hold_f   <= 1'b1;   // 达到长按门限
            end else begin
                press_ms <= 16'd0;
                hold_f   <= 1'b0;
            end
        end
    end

    assign ev_hold = hold_f;

    //----------------- 松手沿出单击脉冲（进入过长按态的不算）-----------------
    reg key_down_d;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            key_down_d <= 1'b0;
            ev_single  <= 1'b0;
        end else begin
            key_down_d <= key_down;
            // 上一拍按下、这一拍已松开，且这次按压没进入过长按态 -> 单击
            ev_single  <= key_down_d & ~key_down & ~hold_f;
        end
    end

endmodule

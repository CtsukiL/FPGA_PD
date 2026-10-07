`timescale 1ns/1ps
//============================================================================
// tb_states.v —— 各状态对应的电机输出模拟
//
// 逐个构造触发条件，把 0 / 1 / 21~24 / 31~34 / 4 每个状态下"电机收到什么"
// 打印出来，方便对着 STM32 版逐状态核对：
//   状态 0        停止           -> 电机指令 0
//   状态 1        判断           -> 不驱动（等判据）
//   状态 21/22    左推           -> +35 持续 100ms
//   状态 23/24    右推           -> -35 持续 100ms
//   状态 31/32    右推           -> -35
//   状态 33/34    左推           -> +35
//   状态 4        PID 平衡       -> 角度环输出（随摆角变化）
//============================================================================
module tb_states;

    localparam integer DIV       = 50;
    localparam real    CENTER    = 2010.0;
    localparam integer NS_PER_MS = 20000;

    //----------------- 时钟 / DUT ----------------
    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;

    reg  [9:0] adc_d   = 10'd0;
    reg        adc_otr = 1'b0;
    reg        enc_a   = 1'b0;
    reg        enc_b   = 1'b0;
    reg  [3:0] key     = 4'b1111;      // 4 个外接按键，低有效
    wire [3:0] led;
    wire       uart_tx, adc_clk, adc_oe, oled_scl, motor_pwm, motor_in1, motor_in2;
    wire       oled_sda;

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

    //----------------- 观察信号 ----------------
    wire [5:0]         run_state  = u_top.run_state;
    wire               tick_40ms  = u_top.tick_40ms;
    wire [11:0]        angle_dut  = u_top.angle;
    wire [11:0]        center_dut = u_top.center_angle;
    wire signed [15:0] ang_target = u_top.u_ctrl_fsm.ang_target;
    wire signed [15:0] angle_out  = u_top.angle_out;
    wire signed [15:0] pos_out    = u_top.pos_out;
    wire signed [15:0] motor_cmd  = u_top.motor_cmd;

    //----------------- 激励任务 ----------------
    task set_adc(input integer v12);        // 直接给 12bit 角度值
        integer v;
        begin
            v = v12;
            if (v < 0)    v = 0;
            if (v > 4095) v = 4095;
            adc_d = v[11:2];                // 12bit -> 10bit
        end
    endtask

    task set_deg(input real deg);           // 按"度"给角度（正 = 使 12bit 值变小）
        begin
            set_adc($rtoi(CENTER - deg * 10.0 + 0.5));
        end
    endtask

    integer i;
    real    degs [0:8];

    // 物理时间计数器：加到波形上当时间参考用
    // （波形的时间轴是仿真时间，1ms 物理 = 20us 仿真，看这个信号最直观）
    integer phys_ms = 0;
    always @(posedge u_top.tick_1ms) phys_ms = phys_ms + 1;

    //----------------- 状态切换就打印 ----------------
    always @(run_state) begin
        #50;    // 等 2.5 个时钟周期：motor_cmd 下一拍更新，motor_pwm 的 IN1/IN2 再下一拍更新
        $display("   [%6d ms] state -> %2d : cmd %4d (IN1=%b IN2=%b)  angle12=%4d  tgt=%5d",
                 $time / NS_PER_MS, run_state, motor_cmd, motor_in1, motor_in2,
                 angle_dut, ang_target);
    end

    //----------------- 测试流程 ----------------
    initial begin
        degs[0] =  20.0; degs[1] =  10.0; degs[2] =   5.0;
        degs[3] =   2.0; degs[4] =   0.0; degs[5] =  -2.0;
        degs[6] =  -5.0; degs[7] = -10.0; degs[8] = -20.0;

        $display("==== tb_states : motor output for each state ====");

        //========== 阶段 0：上电，停止态 ==========
        set_deg(0.0);
        #(NS_PER_MS * 80);
        $display("");
        $display("[STATE 0] stop, K1 not pressed");
        $display("   motor_cmd = %0d (IN1=%b IN2=%b)   expect 0 / 0 / 0",
                 motor_cmd, motor_in1, motor_in2);

        //========== 标定 ==========
        key[3] = 1'b0; #(NS_PER_MS * 1100); key[3] = 1'b1; #(NS_PER_MS * 120);
        $display("");
        $display("[CALIB] press key[3] -> CENTER_ANGLE = %0d", center_dut);

        //========== 阶段 1：按 K1 -> 状态 21/22/23/24 ==========
        $display("");
        set_adc(1000);                          // 先置于左区间（非中心区），避免推力结束就入区
        #(NS_PER_MS * 20);
        $display("[K1 CLICK] state transitions / motor output (angle starts in left zone):");
        key[0] = 1'b0; #(NS_PER_MS * 40); key[0] = 1'b1;
        #(NS_PER_MS * 320);                     // 覆盖 21/22/23/24 全程
        set_adc(1000);                          // 回到左区间（非中心区）
        #(NS_PER_MS * 200);

        //========== 阶段 2：喂"右极值" -> 状态 21/22/23/24 ==========
        $display("");
        $display("[STATE 1 + right-peak 2800/2700/2750] expect 21(+35) -> 23(-35)");
        @(posedge tick_40ms); set_adc(2800);
        @(posedge tick_40ms); set_adc(2700);
        @(posedge tick_40ms); set_adc(2750);
        @(posedge tick_40ms);
        #(NS_PER_MS * 300);
        set_adc(1000);
        #(NS_PER_MS * 200);

        //========== 阶段 3：喂"左极值" -> 状态 31/32/33/34 ==========
        $display("");
        $display("[STATE 1 + left-peak 1000/1100/1050]  expect 31(-35) -> 33(+35)");
        @(posedge tick_40ms); set_adc(1000);
        @(posedge tick_40ms); set_adc(1100);
        @(posedge tick_40ms); set_adc(1050);
        @(posedge tick_40ms);
        #(NS_PER_MS * 300);
        set_adc(1000);
        #(NS_PER_MS * 300);

        //========== 阶段 4：喂中心区 -> 入区，看状态 4 ==========
        $display("");
        $display("[STATE 1 + in-zone] expect -> state 4");
        set_deg(0.0);
        #(NS_PER_MS * 300);
        $display("   current state = %0d  (4 = PID balance)", run_state);

        //========== 阶段 5：状态 4 下不同摆角 ==========
        $display("");
        $display("[STATE 4 PID] motor output vs lean angle:");
        $display("     lean(deg)  angle12   tgt    angOut   posOut   motor_cmd  IN1 IN2");
        $display("  --------------------------------------------------------------------");
        for (i = 0; i <= 8; i = i + 1) begin
            set_deg(degs[i]);
            #(NS_PER_MS * 400);
            $display("    %8.1f   %6d  %7d  %7d  %9d  %8d    %b   %b",
                     degs[i], angle_dut, ang_target, angle_out, pos_out,
                     motor_cmd, motor_in1, motor_in2);
        end

        $display("");
        $display("==== done ====");
        $finish;
    end

    initial begin
        #(NS_PER_MS * 30000);
        $display("!! timeout");
        $finish;
    end

endmodule

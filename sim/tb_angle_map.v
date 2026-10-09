`timescale 1ns/1ps
//============================================================================
// tb_angle_map.v —— 角度环开环测试
//
// 目的：按完 K1（进入 PID 平衡态）后，给角度传感器输入不同的角度值，
//       观察最终输出给电机的信号（方向 + 指令大小），验证：
//         1) 极性：摆往哪边偏，电机往哪边转（必须是"扶回来"的方向）
//         2) 增益：角度 → 电机指令 的对应关系
//         3) 死区/饱和：多小的偏差电机不转、多大偏差输出打满
//
// 做法：
//   - 本 tb 不接摆模型，直接由 set_deg() 设置电位器角度（经 10bit 并行口送进去）
//   - 编码器不给脉冲，location 恒为 0，位置环输出保持 0，不干扰角度环
//   - 先按 key[3]（K4）标定，再按 key[0]（K1）进 PID 态，然后逐段给定角度
//============================================================================
module tb_angle_map;

    localparam integer DIV       = 50;      // 仿真加速比（与 ctrl_fsm 侧一致）
    localparam real    CNT_DEG   = 10.0;    // 电位器：每度对应多少 ADC 码
    localparam real    CENTER    = 2010.0;  // 平衡点 ADC 值
    localparam integer NS_PER_MS = 20000;   // 1ms 物理 = 20us 仿真

    //----------------- 时钟 / DUT ----------------
    reg sys_clk = 1'b0;
    always #10 sys_clk = ~sys_clk;          // 50MHz

    reg  [9:0] adc_d   = 10'd0;
    reg        adc_otr = 1'b0;
    reg        enc_a   = 1'b0;
    reg        enc_b   = 1'b0;
    reg  [3:0] key     = 4'b1111;           // 4 个外接按键，低有效
    wire [3:0] led;
    wire       uart_tx, adc_clk, oled_scl, motor_pwm, motor_in1, motor_in2;
    wire       oled_sda;

    top u_top(
        .sys_clk   (sys_clk),
        .key       (key),
        .led       (led),
        .uart_tx   (uart_tx),
        .adc_clk   (adc_clk),
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
    wire [11:0]        angle_dut  = u_top.angle;
    wire [11:0]        center_dut = u_top.center_angle;
    wire signed [15:0] ang_target = u_top.u_ctrl_fsm.ang_target;
    wire signed [15:0] angle_out  = u_top.angle_out;
    wire signed [15:0] pos_out    = u_top.pos_out;
    wire signed [15:0] motor_cmd  = u_top.motor_cmd;
    wire signed [31:0] location   = u_top.location;

    //----------------- 设置电位器角度（deg：相对竖直，正=偏向一侧）-----------------
    task set_deg(input real deg);
        integer v;
        begin
            v = $rtoi(CENTER - deg * CNT_DEG + 0.5);
            if (v < 0)    v = 0;
            if (v > 4095) v = 4095;
            adc_d = v[11:2];                // 12bit -> 10bit（adc_if 内部会左移 2 位还原）
        end
    endtask

    real    degs [0:8];
    integer i;

    initial begin
        degs[0] =  20.0; degs[1] =  10.0; degs[2] =   5.0;
        degs[3] =   2.0; degs[4] =   0.0; degs[5] =  -2.0;
        degs[6] =  -5.0; degs[7] = -10.0; degs[8] = -20.0;

        $display("==== tb_angle_map：按 K1 后逐段给定摆角，观察电机输出 ====");

        // 上电，摆角 0（竖直）
        set_deg(0.0);
        #(NS_PER_MS * 60);

        // ---- 标定：按 key[3]（K4）1.1s ----
        key[3] = 1'b0;
        #(NS_PER_MS * 1100);
        key[3] = 1'b1;
        #(NS_PER_MS * 120);
        $display("标定完成：CENTER_ANGLE = %0d（摆竖直时的读数，应约 2008）", center_dut);

        // ---- 按 key[0]（K1）：进 PID 平衡态（摆已在中心区，很快入区）----
        key[0] = 1'b0;
        #(NS_PER_MS * 40);
        key[0] = 1'b1;
        #(NS_PER_MS * 600);
        $display("按 K1 后 600ms：RunState = %0d（4 = PID 平衡态）", run_state);
        $display("");

        $display("   摆角(度)  12bit角度  CENTER  环目标   环输出  位置环Out  电机指令  IN1 IN2");
        $display("  ---------------------------------------------------------------------------");

        for (i = 0; i <= 8; i = i + 1) begin
            set_deg(degs[i]);               // 给定角度
            #(NS_PER_MS * 400);             // 等 400ms 让 3ms 平均 + 5ms 角度环稳定
            $display("   %8.1f   %6d    %6d  %7d  %8d  %9d  %8d     %b   %b",
                     degs[i], angle_dut, center_dut, ang_target,
                     angle_out, pos_out, motor_cmd, motor_in1, motor_in2);
        end

        $display("");
        $display("说明：IN1/IN2 = 0/1 正转、1/0 反转、0/0 停止（TB6612）；电机指令 -100~+100");
        $display("==== 结束 ====");
        $finish;
    end

    initial begin
        #(NS_PER_MS * 20000);
        $display("!! timeout");
        $finish;
    end

endmodule

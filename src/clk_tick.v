//============================================================================
// clk_tick.v —— 节拍生成
// 对应 STM32 版：Timer.c 的 1ms 定时中断 + main.c 中断里的计次分频
//   tick_1ms  : 1ms  采样节拍（原 1ms 中断）
//   tick_5ms  : 5ms  角度环（原 Count1 >= 5）
//   tick_40ms : 40ms 启摆判据三点采样（原 Count0 >= 40）
//   tick_50ms : 50ms 位置环（原 Count2 >= 50）
// 说明：全部用独立计数器直接分频，不使用 enable 级联，避免额外累积误差。
// 参数 DIV：仿真加速比。综合时保持默认 1（计数值与真实 50MHz 完全一致）；
//           仿真时用 defparam 设成 50，则 1ms 物理时间 = 1000 个时钟周期，
//           物理时间轴（5ms/40ms/50ms/100ms）保持不变。
//============================================================================
module clk_tick #(
    parameter integer DIV = 1           // 仿真加速比，综合时用 1
)(
    input  wire clk,            // 50MHz 系统时钟
    input  wire rst_n,          // 低电平复位
    output reg  tick_1ms,       // 1ms 单周期脉冲
    output reg  tick_5ms,       // 5ms 单周期脉冲
    output reg  tick_40ms,      // 40ms 单周期脉冲
    output reg  tick_50ms       // 50ms 单周期脉冲
);

    // 50MHz 下的计数值（周期数 - 1）；DIV=1 时与真实值完全相同
    localparam [31:0] CNT_1MS  = (32'd50_000    / DIV) - 32'd1;
    localparam [31:0] CNT_5MS  = (32'd250_000   / DIV) - 32'd1;
    localparam [31:0] CNT_40MS = (32'd2_000_000 / DIV) - 32'd1;
    localparam [31:0] CNT_50MS = (32'd2_500_000 / DIV) - 32'd1;

    reg [31:0] cnt_1ms, cnt_5ms, cnt_40ms, cnt_50ms;

    //---------------------- 1ms ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_1ms  <= 32'd0;
            tick_1ms <= 1'b0;
        end else if (cnt_1ms >= CNT_1MS) begin
            cnt_1ms  <= 32'd0;
            tick_1ms <= 1'b1;
        end else begin
            cnt_1ms  <= cnt_1ms + 32'd1;
            tick_1ms <= 1'b0;
        end
    end

    //---------------------- 5ms ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_5ms  <= 32'd0;
            tick_5ms <= 1'b0;
        end else if (cnt_5ms >= CNT_5MS) begin
            cnt_5ms  <= 32'd0;
            tick_5ms <= 1'b1;
        end else begin
            cnt_5ms  <= cnt_5ms + 32'd1;
            tick_5ms <= 1'b0;
        end
    end

    //---------------------- 40ms ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_40ms  <= 32'd0;
            tick_40ms <= 1'b0;
        end else if (cnt_40ms >= CNT_40MS) begin
            cnt_40ms  <= 32'd0;
            tick_40ms <= 1'b1;
        end else begin
            cnt_40ms  <= cnt_40ms + 32'd1;
            tick_40ms <= 1'b0;
        end
    end

    //---------------------- 50ms ----------------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_50ms  <= 32'd0;
            tick_50ms <= 1'b0;
        end else if (cnt_50ms >= CNT_50MS) begin
            cnt_50ms  <= 32'd0;
            tick_50ms <= 1'b1;
        end else begin
            cnt_50ms  <= cnt_50ms + 32'd1;
            tick_50ms <= 1'b0;
        end
    end

endmodule

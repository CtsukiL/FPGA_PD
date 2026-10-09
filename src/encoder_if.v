//============================================================================
// encoder_if.v —— AB 相编码器接口（摆臂位置）
// 对应 STM32 版：Encoder.c（TIM3 编码器模式 TI12）+ main.c 的 Location += Speed
// 移植自 J280 官方例程 Interface_test/src/encoder.v：
//   两级触发器同步 + 四倍频状态机
// 与 STM32 版的差异（有意为之）：
//   STM32 的 Location 是 int16 每 1ms 累加（约 7.8s 就溢出）；
//   这里位置用 32bit 累加，并且支持 pos_clr 把当前点记为位置零点，
//   与 main.c 中"K4 标定同时记录初始位置（Location = 0）"的行为一致。
//   A/B 相加了一级 3.6us 数字滤波，对应 Encoder.c 的 TIM_ICFilter = 0xF。
//============================================================================
module encoder_if(
    input  wire               clk,       // 50MHz 系统时钟
    input  wire               rst_n,     // 低电平复位
    input  wire               enc_a,     // 编码器 A 相
    input  wire               enc_b,     // 编码器 B 相
    input  wire               pos_clr,   // 单周期脉冲：把当前位置记为初始位置（K4 标定）
    output reg  signed [31:0] pos        // 累计位置（count，1 圈 = CFG_CNT_PER_REV，见 pendulum_cfg.vh）
);

    //----------------- 两级触发器同步（消除亚稳态）-----------------
    reg [1:0] sync_a, sync_b;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            sync_a <= 2'b11;
            sync_b <= 2'b11;
        end else begin
            sync_a <= {sync_a[0], enc_a};
            sync_b <= {sync_b[0], enc_b};
        end
    end

    //----------------- 输入滤波（对应 Encoder.c 的 TIM_ICFilter = 0xF）-----------------
    // STM32 那个滤波器在 72MHz 下约 8 x 32 / 72MHz = 3.6us；
    // 这里用"连续 FILT_MAX 个时钟电平不变才认"抑制同样的窄脉冲。
    localparam [7:0] FILT_MAX = 8'd179;     // 50MHz 下 180 拍 = 3.6us

    reg [7:0] cnt_a, cnt_b;
    reg       fa, fb;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            cnt_a <= 8'd0;
            cnt_b <= 8'd0;
            fa    <= 1'b1;
            fb    <= 1'b1;
        end else begin
            if (sync_a[1] == fa) begin
                cnt_a <= 8'd0;
            end else if (cnt_a >= FILT_MAX) begin
                fa    <= sync_a[1];
                cnt_a <= 8'd0;
            end else begin
                cnt_a <= cnt_a + 8'd1;
            end

            if (sync_b[1] == fb) begin
                cnt_b <= 8'd0;
            end else if (cnt_b >= FILT_MAX) begin
                fb    <= sync_b[1];
                cnt_b <= 8'd0;
            end else begin
                cnt_b <= cnt_b + 8'd1;
            end
        end
    end

    //----------------- 边沿状态（用滤波后的电平）-----------------
    reg [1:0] state_now, state_last;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state_now  <= 2'b11;
            state_last <= 2'b11;
        end else begin
            state_now  <= {fa, fb};
            state_last <= state_now;
        end
    end

    //----------------- 四倍频方向判断 + 位置累加 ----------------
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            pos <= 32'sd0;
        end else if (pos_clr) begin
            pos <= 32'sd0;              // 当前位置 = 初始位置（位置零点）
        end else begin
            case ({state_last, state_now})
                4'b00_01: pos <= pos - 32'sd1;
                4'b01_11: pos <= pos - 32'sd1;
                4'b11_10: pos <= pos - 32'sd1;
                4'b10_00: pos <= pos - 32'sd1;
                4'b00_10: pos <= pos + 32'sd1;
                4'b10_11: pos <= pos + 32'sd1;
                4'b11_01: pos <= pos + 32'sd1;
                4'b01_00: pos <= pos + 32'sd1;
                default : pos <= pos;   // 其余组（原地/双跳）不动
            endcase
        end
    end

endmodule

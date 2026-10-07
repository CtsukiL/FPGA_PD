//============================================================================
// uart_dbg.v - UART telemetry (115200 8N1, 49-byte frame every 20ms).
// 对应 STM32 版：Serial.c（原程序未使用）。FPGA 上没有 Keil Watch，
// 所以把关键量定时上报，便于调参和排障。
//   波特率 115200（50MHz / 115200 ≈ 434）
//   Frame layout is documented in doc/telemetry.md.
//     [0]0xAA [1]0x55                       帧头
//     [2..3]  angle        (uint16, 12bit)  角度值
//     [4..7]  location     (int32)          横杆位置 count
//     [8..9]  angle_out    (int16)          角度环输出
//     [10..11]pos_out      (int16)          位置环输出
//     [12]    run_state    (uint8)          运行状态
//     [13..14]center_angle (uint16, 12bit)  平衡点角度
//     [15]0x5A                              帧尾
// 用 doc\uart_parse.py 可直接解析成 CSV。
//============================================================================
module uart_dbg #(
    parameter integer FRAME_MS = 20
)(
    input  wire               clk,          // 50MHz 系统时钟
    input  wire               rst_n,        // 低电平复位
    input  wire [11:0]        angle,        // 角度值
    input  wire signed [31:0] location,     // 横杆位置
    input  wire signed [15:0] angle_out,    // 角度环输出
    input  wire signed [15:0] pos_out,      // 位置环输出
    input  wire [5:0]         run_state,    // 运行状态
    input  wire [11:0]        center_angle, // 平衡点角度
    input  wire signed [31:0] pos_target,
    input  wire signed [31:0] pos_set,
    input  wire signed [31:0] bar_vel,
    input  wire signed [31:0] target_vel,
    input  wire signed [15:0] motor_cmd,
    input  wire [9:0]         raw_code,
    input  wire [11:0]        avg4,
    input  wire               mov_active,
    output reg                tx            // 串口发送
);

    localparam [8:0] BAUD_DIV = 9'd434;             // 50MHz / 115200
    localparam integer FRAME_GAP = 50000 * FRAME_MS; // 50MHz 下每毫秒 50000 拍

    //----------------- 帧周期 ----------------
    reg [31:0] frame_cnt;
    reg        frame_req;
    reg [31:0] uptime_ms;
    reg [15:0] ms_div;

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            frame_cnt <= 32'd0;
            frame_req <= 1'b0;
            uptime_ms <= 32'd0;
            ms_div    <= 16'd0;
        end else if (frame_cnt >= (FRAME_GAP - 1)) begin
            frame_cnt <= 32'd0;
            frame_req <= 1'b1;
        end else begin
            frame_cnt <= frame_cnt + 32'd1;
            frame_req <= 1'b0;
            if (ms_div == 16'd49999) begin ms_div <= 16'd0; uptime_ms <= uptime_ms + 1'b1; end
            else ms_div <= ms_div + 1'b1;
        end
    end

    //----------------- 帧内字节索引 ----------------
    reg [5:0] idx;
    reg [15:0] sequence;
    reg [391:0] frame;
    reg [15:0] crc;

    function [15:0] crc_byte;
        input [15:0] crc_in;
        input [7:0] data;
        reg [15:0] c;
        integer k;
        begin
            c = crc_in ^ {data, 8'h00};
            for (k = 0; k < 8; k = k + 1)
                c = c[15] ? ((c << 1) ^ 16'h1021) : (c << 1);
            crc_byte = c;
        end
    endfunction

    wire [7:0] frame_byte = frame[idx * 8 +: 8];

    //----------------- 待发字节（按索引生成）-----------------
    reg [7:0] byte_in;
    always @(*) byte_in = frame_byte;

    //----------------- 发送状态机 ----------------
    localparam [2:0] U_IDLE = 3'd0, U_CRC = 3'd1, U_LOAD = 3'd2, U_SHIFT = 3'd3, U_DONE = 3'd4;

    reg [2:0] u_st;
    reg [9:0] sh;       // {stop, data[7:0], start}
    reg [3:0] bitn;     // 已发送位数
    reg [8:0] div;      // 波特率分频

    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            u_st <= U_IDLE;
            tx   <= 1'b1;
            sh   <= 10'h3FF;
            bitn <= 4'd0;
            div  <= 9'd0;
            idx  <= 6'd0;
            sequence <= 16'd0;
            frame <= 392'd0;
            crc <= 16'hFFFF;
        end else begin
            case (u_st)
                U_IDLE: begin
                    tx <= 1'b1;
                    if (frame_req) begin
                        frame <= 392'd0;
                        frame[7:0] <= 8'hAA; frame[15:8] <= 8'h55;
                        frame[23:16] <= 8'd2; frame[31:24] <= 8'd49;
                        frame[63:32] <= uptime_ms; frame[79:64] <= sequence;
                        frame[95:80] <= {4'b0, angle}; frame[127:96] <= location;
                        frame[143:128] <= angle_out; frame[159:144] <= pos_out;
                        frame[167:160] <= {2'b0, run_state}; frame[183:168] <= {4'b0, center_angle};
                        frame[215:184] <= pos_target; frame[247:216] <= pos_set;
                        frame[279:248] <= bar_vel; frame[311:280] <= target_vel;
                        frame[327:312] <= motor_cmd; frame[343:328] <= {6'b0, raw_code};
                        frame[359:344] <= avg4;
                        frame[367:360] <= {7'b0, mov_active}; frame[383:368] <= 16'd0;
                        frame[391:384] <= 8'h5A;
                        sequence <= sequence + 1'b1;
                        idx <= 6'd2;
                        crc <= 16'hFFFF;
                        u_st <= U_CRC;
                    end
                end

                U_CRC: begin
                    crc <= crc_byte(crc, frame_byte);
                    if (idx == 6'd45) begin
                        frame[383:368] <= crc_byte(crc, frame_byte);
                        idx <= 6'd0;
                        u_st <= U_LOAD;
                    end else idx <= idx + 1'b1;
                end

                U_LOAD: begin
                    sh   <= {1'b1, byte_in, 1'b0};  // 停止位 / 数据 / 起始位
                    bitn <= 4'd0;
                    div  <= BAUD_DIV - 9'd1;
                    u_st <= U_SHIFT;
                end

                U_SHIFT: begin
                    tx <= sh[0];
                    if (div == 9'd0) begin
                        div <= BAUD_DIV - 9'd1;
                        if (bitn == 4'd9) begin
                            u_st <= U_DONE;
                        end else begin
                            sh   <= {1'b1, sh[9:1]};
                            bitn <= bitn + 4'd1;
                        end
                    end else begin
                        div <= div - 9'd1;
                    end
                end

                U_DONE: begin
                    tx <= 1'b1;
                    if (idx == 6'd48) begin
                        u_st <= U_IDLE;
                    end else begin
                        idx  <= idx + 4'd1;
                        u_st <= U_LOAD;
                    end
                end

                default: u_st <= U_IDLE;
            endcase
        end
    end

endmodule

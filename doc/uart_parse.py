#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
uart_parse.py —— 解析 FPGA 倒立摆工程通过串口上报的数据帧

帧格式（115200-8-N-1，每 100ms 一帧，固定 16 字节，小端）：
    [0]      0xAA        帧头
    [1]      0x55        帧头
    [2..3]   angle       角度值（uint16，12bit 有效）   0~4095
    [4..7]   location    横杆位置（int32，count）        1 圈 = 408
    [8..9]   angle_out   角度环输出（int16）            -100~100
    [10..11] pos_out     位置环输出（int16）            -100~100
    [12]     run_state   运行状态（uint8）0/1/4/21..34
    [13..14] center_angle 平衡点角度（uint16，12bit）
    [15]     0x5A        帧尾

用法：
    python uart_parse.py COM3            # 读串口并打印（需要 pyserial）
    python uart_parse.py COM3 -c         # 读串口并输出 CSV
    python uart_parse.py dump.bin        # 解析已经存下来的二进制文件
    python uart_parse.py dump.bin -c     # 解析文件并输出 CSV

安装依赖（只有读串口时才需要）：
    pip install pyserial
"""

import struct
import sys

FRAME_LEN = 49
HEAD = b"\xAA\x55"
TAIL = 0x5A


def parse_stream(data, csv=False):
    """从字节流里按 16 字节帧提取数据，返回解析出的记录列表"""
    rows = []
    i = 0
    n = len(data)
    while i + FRAME_LEN <= n:
        if data[i : i + 2] != HEAD:
            i += 1
            continue
        frame = data[i : i + FRAME_LEN]
        if frame[48] != TAIL or frame[3] != FRAME_LEN:
            i += 1
            continue
        payload = frame[2:46]
        crc = 0xFFFF
        for value in payload:
            crc ^= value << 8
            for _ in range(8):
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
        if struct.unpack_from("<H", frame, 46)[0] != crc:
            i += 1
            continue
        rows.append({
            "time_ms": struct.unpack_from("<I", frame, 4)[0],
            "seq": struct.unpack_from("<H", frame, 8)[0],
            "angle": struct.unpack_from("<H", frame, 10)[0],
            "location": struct.unpack_from("<i", frame, 12)[0],
            "angle_out": struct.unpack_from("<h", frame, 16)[0],
            "pos_out": struct.unpack_from("<h", frame, 18)[0],
            "run_state": frame[20],
            "center_angle": struct.unpack_from("<H", frame, 21)[0],
            "pos_target": struct.unpack_from("<i", frame, 23)[0],
            "pos_set": struct.unpack_from("<i", frame, 27)[0],
            "bar_vel": struct.unpack_from("<i", frame, 31)[0],
            "target_vel": struct.unpack_from("<i", frame, 35)[0],
            "motor_cmd": struct.unpack_from("<h", frame, 39)[0],
            "raw_code": struct.unpack_from("<H", frame, 41)[0],
            "avg4": struct.unpack_from("<H", frame, 43)[0],
            "mov_active": frame[45] & 1,
        })
        i += FRAME_LEN
    if csv:
        print("time_ms,seq,angle,location,angle_out,pos_out,run_state,center_angle,pos_target,pos_set,bar_vel,target_vel,motor_cmd,raw_code,avg4,mov_active")
        for r in rows:
            print(",".join(str(r[k]) for k in r))
    else:
        print(rows[-1] if rows else "no frame")
    return rows


def read_serial(port, baud=115200, csv=False):
    try:
        import serial  # pyserial
    except ImportError:
        print("缺少 pyserial，请先执行：pip install pyserial")
        return
    ser = serial.Serial(port, baud, timeout=0.5)
    print("# 打开", port, baud, "8N1，Ctrl+C 结束")
    buf = b""
    try:
        while True:
            buf += ser.read(4096)
            frames = buf.split(HEAD)
            if len(frames) > 1:
                # 保留可能的半个帧头
                buf = HEAD + frames[-1]
                for chunk in frames[1:-1]:
                    parse_stream(HEAD + chunk, csv=csv)
                parse_stream(HEAD + frames[-1], csv=csv)
            else:
                buf = buf[-64:]
    except KeyboardInterrupt:
        print("\n# 结束")
    finally:
        ser.close()


def main():
    if len(sys.argv) < 2:
        print(__doc__)
        return
    arg = sys.argv[1]
    csv = "-c" in sys.argv
    if arg.upper().startswith("COM") or arg.startswith("/dev/"):
        read_serial(arg, csv=csv)
    else:
        with open(arg, "rb") as f:
            parse_stream(f.read(), csv=csv)


if __name__ == "__main__":
    main()

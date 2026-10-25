#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
0xEA 报文字段诊断工具：交互式发送原始两个字 (v1, v2)，用于确认驱动器真实语义。
用法：python3 test_0xea.py /dev/ttyUSB0
命令（输入后回车执行）：
    v1 v2        发送 frame(v1, v2)，如: 500 500 / -500 -500 / 0 500 / 500 0
    b            切换字节序 大端/小端（默认小端，即低字节在前）
    s            切换字序 交换/不交换（交换 = 把 v1 v2 对调位置发送）
    q            退出（会发送 0 0 停车）
建议测试序列（每步观察两轮转向与快慢）：
    1) 500 0     2) 0 500     3) 500 500     4) 500 -500     5) -500 500
    若某一步出现"两轮同向、小车直走"，记下对应的 v1/v2 与当前 b/s 状态，
    即可反推字段含义（哪个是左轮、哪个是右轮、正方向朝哪）。
"""
import sys, time, struct
import serial

def crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF

def frame(v1, v2, little=True, swap=False, endian_hint=False):
    if endian_hint:
        # 大端：高字节在前
        b1 = struct.pack(">h", v1)
        b2 = struct.pack(">h", v2)
    else:
        b1 = struct.pack("<h", v1)
        b2 = struct.pack("<h", v2)
    if swap:
        b1, b2 = b2, b1
    f = bytes([0x01, 0xEA, 0x00, 0x00, 0x00]) + b1 + b2
    c = crc16(f)
    return f + bytes([c & 0xFF, c >> 8])

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    ser = serial.Serial(port, 115200, timeout=0.05)
    little = True   # False -> 高字节在前（大端）
    swap = False    # True  -> 两个 16 位字对调
    print(__doc__)
    while True:
        cmd = input(f"[b={'LE' if little else 'BE'} s={swap}] > ").strip().lower()
        if cmd == "q":
            ser.write(frame(0, 0, little, swap)); time.sleep(0.1)
            break
        if cmd == "b":
            little = not little; continue
        if cmd == "s":
            swap = not swap; continue
        try:
            if cmd.startswith("r "):
                # 原始字节模式：r F4 01 00 00  -> 01 EA + 你给的字节 + 自动CRC
                raw = bytes(int(x, 16) for x in cmd[2:].split())
                f = bytes([0x01, 0xEA]) + raw
                c = crc16(f)
                f = f + bytes([c & 0xFF, c >> 8])
            else:
                v1, v2 = (int(x) for x in cmd.split())
                f = frame(v1, v2, little, swap)
        except ValueError:
            print("输入示例: 500 -500  或  r F4 01 00 00 00 00"); continue
        ser.write(f); ser.flush()
        time.sleep(0.05)
        resp = ser.read(64)
        print(f"TX: {f.hex(' ').upper()}   RX: {resp.hex(' ').upper() or '(无)'}")
    ser.close()

if __name__ == "__main__":
    main()

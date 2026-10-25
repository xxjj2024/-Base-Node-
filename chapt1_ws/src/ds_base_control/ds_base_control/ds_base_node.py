#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ds_base_node.py  两轮差速底盘控制节点.

功能: 订阅 /cmd_vel, 经差速运动学解算为左右轮 RPM,
通过串口以 Modbus RTU (功能码 0x44) 下发给驱动器.

协议字典 (HEX, 均为 CRC-16/MODBUS, 低字节在前):
    使能: 01 44 21 00 31 00 00 01 00 01 75 34   (已验证: CRC-16/MODBUS 吻合)
    失能: 01 44 21 00 31 00 00 00 00 00 E5 34   (已验证: CRC-16/MODBUS 吻合)
    0x44 速度帧: 包头 01 44 23 18 33 18 + A轴RPM(大端有符号16位)
    + B轴RPM(大端有符号16位) + CRC16(低前高后)
"""

import math
import struct
import time

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

import serial


# ---------------- 补偿与硬件参数 ----------------
WHEEL_DIAMETER_MM = 168.0   # 轮径 168 mm
WHEEL_TRACK_MM = 315.0      # 轮距 315 mm
A_AXIS_INVERT = True        # A轴出厂装反, 发送给A轴的RPM需乘 -1
LINE_INVERT = True          # 整体前后方向翻转, 线速度乘 -1
ANG_INVERT = True           # 整体左右方向翻转, 角速度乘 -1

# ---------------- 速度平滑(Velocity Smoother)参数 ----------------
# 含义: current 每个控制周期最多变化 加速度上限 * dt
# 例: MAX_ACCEL_V=800 mm/s^2, dt=0.05s -> 每周期最多变 40 mm/s
#     从 0 加速到 500 mm/s 需要 500/800 = 0.625s
MAX_ACCEL_V = 800.0         # 线加速度上限 (mm/s^2), 起步/刹车共用, 调小更柔和
MAX_ACCEL_W = 2000.0        # 角加速度上限 (mrad/s^2), 转向平滑度, 调小更柔和

# ---------------- 协议字典 ----------------
HEADER = bytes([0x01, 0x44, 0x23, 0x18, 0x33, 0x18])    # 0x44速度帧包头
ENABLE_FRAME = bytes([0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x01, 0x00, 0x01])  # 使能帧载荷
DISABLE_FRAME = bytes([0x01, 0x44, 0x21, 0x00, 0x31, 0x00, 0x00, 0x00, 0x00, 0x00])  # 失能帧载荷


def crc16_modbus(data: bytes) -> int:
    """CRC-16/MODBUS: 初始值0xFFFF, 多项式0xA001反射, 发送时低字节在前."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc


class DsBaseNode(Node):
    def __init__(self):
        super().__init__('ds_base_node')

        # ---------------- 参数声明 ----------------
        self.declare_parameter('port', '/dev/ttyUSB0')
        port = self.get_parameter('port').get_parameter_value().string_value

        # ---------------- 状态变量 ----------------
        self.target_v = 0.0     # 目标线速度 mm/s (来自话题, 已应用 LINE_INVERT)
        self.target_w = 0.0     # 目标角速度 mrad/s (来自话题, 已应用 ANG_INVERT)
        self.current_v = 0.0    # 当前平滑线速度 mm/s (每帧真实下发值)
        self.current_w = 0.0    # 当前平滑角速度 mrad/s (每帧真实下发值)
        self.last_cmd_time = self.get_clock().now()  # 最近一次收到 /cmd_vel 的时刻
        self.WATCHDOG_TIMEOUT = 0.5                  # 看门狗超时: 秒
        self.CONTROL_PERIOD = 0.05                   # 控制周期: 20Hz
        self.last_warn_time = 0.0                    # 看门狗告警节流

        # ---------------- 打开串口 ----------------
        self.get_logger().info(f'打开串口: {port} @ 115200')
        self.ser = serial.Serial(
            port=port,
            baudrate=115200,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=0.1,
        )
        self.ser.reset_input_buffer()   # 清空接收缓冲
        self.ser.reset_output_buffer()  # 清空发送缓冲

        # ---------------- 下发使能指令 ----------------
        self.ser.write(ENABLE_FRAME + struct.pack('<H', crc16_modbus(ENABLE_FRAME)))
        self.ser.flush()
        self.get_logger().info('已下发使能指令')

        # ---------------- 订阅 /cmd_vel ----------------
        self.cmd_sub = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # ---------------- 20Hz 定时下发 ----------------
        self.timer = self.create_timer(self.CONTROL_PERIOD, self.timer_callback)
        self.get_logger().info('ds_base_node 初始化完成, 等待 /cmd_vel ...')

    # ---------------- /cmd_vel 回调 ----------------
    def cmd_vel_callback(self, msg: Twist):
        """接收Twist: 只更新 target, 不直接下发; 平滑交给 20Hz 定时器."""
        v_line = msg.linear.x * 1000.0   # m/s   -> mm/s
        v_ang = msg.angular.z * 1000.0   # rad/s -> mrad/s
        if LINE_INVERT:
            v_line = -v_line
        if ANG_INVERT:
            v_ang = -v_ang
        self.target_v = v_line
        self.target_w = v_ang
        self.last_cmd_time = self.get_clock().now()  # 刷新看门狗

    # ---------------- 速度平滑(斜坡逼近) ----------------
    def ramp_towards(self, current: float, target: float,
                     max_accel: float, dt: float) -> float:
        """
        核心: 让 current 以每周期不超过 max_accel*dt 的步长逼近 target.

        原理: 速度斜坡 = 对加速度做限幅积分.
          - 期望变化量 error = target - current
          - 本周期允许的最大变化量 step = max_accel * dt (匀速逼近的物理依据)
          - |error| <= step : 一帧到位(剩余误差小于单步, 直接贴齐, 避免极限环抖动)
          - |error| >  step : 只走一步, 符号与 error 相同 -> 匀加/减速
        返回更新后的 current.
        """
        error = target - current            # 期望变化量
        step = max_accel * dt               # 本周期允许的最大变化量
        if abs(error) <= step:
            # 误差小于一个步长: 直接贴齐目标, 下一帧 error==0 保持稳定
            return target
        # 超出步长: 沿误差方向走一步 (error>0 则 +step, error<0 则 -step)
        return current + math.copysign(step, error)

    # ---------------- 差速运动学解算 ----------------
    def compute_wheel_rpm(self):
        """差速运动学解算: 平滑后的线速度/角速度 -> 左右轮RPM."""
        half_track = WHEEL_TRACK_MM / 2.0
        v_line = self.current_v                      # 使用平滑值而非目标值
        v_ang = self.current_w / 1000.0  # mrad/s -> rad/s 比例还原(mm/s 量纲下的差速项)

        v_left = v_line - v_ang * half_track
        v_right = v_line + v_ang * half_track

        rpm_left = (v_left / (math.pi * WHEEL_DIAMETER_MM)) * 60.0
        rpm_right = (v_right / (math.pi * WHEEL_DIAMETER_MM)) * 60.0

        # A轴装反补偿 (发送帧中 A 轴在前)
        rpm_a = -rpm_left if A_AXIS_INVERT else rpm_left
        rpm_b = rpm_right

        return rpm_a, rpm_b

    # ---------------- 构造并下发速度帧 ----------------
    def send_speed_frame(self, rpm_a: float, rpm_b: float):
        """构造0x44速度帧并经串口下发."""
        payload = HEADER + struct.pack('>h', int(round(rpm_a))) \
                         + struct.pack('>h', int(round(rpm_b)))
        crc = crc16_modbus(payload)
        frame = payload + struct.pack('<H', crc)  # <H: 低字节在前
        self.ser.write(frame)
        self.ser.flush()

    # ---------------- 20Hz 定时回调 ----------------
    def timer_callback(self):
        now = self.get_clock().now()
        elapsed = (now - self.last_cmd_time).nanoseconds / 1e9

        # 看门狗: 超过 0.5s 未收到 /cmd_vel 则目标速度清零.
        # 注意: 这里只把 target 归零, 不直接动 current --
        # 小车会以 MAX_ACCEL/MAX_ACCEL_W 决定的减速度"平滑刹停",
        # 而不是目标值瞬间归零带来的急刹.
        if elapsed > self.WATCHDOG_TIMEOUT:
            if self.target_v != 0.0 or self.target_w != 0.0:
                self.get_logger().warn('看门狗超时(>0.5s无/cmd_vel), 平滑刹停', throttle_duration_sec=1.0)
            self.target_v = 0.0
            self.target_w = 0.0

        # 斜坡逼近: current 每帧最多变化 MAX_ACCEL * dt, 平稳追踪 target
        self.current_v = self.ramp_towards(
            self.current_v, self.target_v, MAX_ACCEL_V, self.CONTROL_PERIOD)
        self.current_w = self.ramp_towards(
            self.current_w, self.target_w, MAX_ACCEL_W, self.CONTROL_PERIOD)

        rpm_a, rpm_b = self.compute_wheel_rpm()
        self.send_speed_frame(rpm_a, rpm_b)

    # ---------------- 节点销毁 ----------------
    def destroy_node(self):
        """退出前: 停定时器 -> 0速报文 -> 失能指令 -> 关闭串口."""
        self.get_logger().info('节点退出: 下发 0 RPM -> 失能 -> 关闭串口')
        self.timer.cancel()                        # 0. 先停 20Hz 定时, 防止清理期间再写串口
        try:
            self.send_speed_frame(0, 0)            # 1. 0 RPM 报文(停车)
            time.sleep(0.05)
            self.ser.write(DISABLE_FRAME + struct.pack('<H', crc16_modbus(DISABLE_FRAME)))  # 2. 失能
            self.ser.flush()
            time.sleep(0.05)
        except Exception as e:
            self.get_logger().error(f'退出清理时串口异常: {e}')
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()                   # 3. 关闭串口
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DsBaseNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass  # Ctrl+C或外部关闭: 均属正常退出路径
    finally:
        node.destroy_node()
        if rclpy.ok():          # Ctrl+C时信号处理器可能已shutdown, 避免二次关闭报错
            rclpy.shutdown()


if __name__ == '__main__':
    main()

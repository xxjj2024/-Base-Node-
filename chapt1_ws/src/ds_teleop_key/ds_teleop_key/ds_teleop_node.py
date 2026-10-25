#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ds_teleop_node.py  自定义键盘遥控节点.

功能: 通过 termios/tty/select 实现纯终端非阻塞键盘监听(SSH 可用),
将按键映射为 /cmd_vel (Twist) 速度指令, 完全替代 teleop_twist_keyboard.

按键说明:
    W: 前进            S: 后退
    A: 原地左转        B 或 D: 原地右转
    Q: 左前弧线(前进+左转)    E: 右前弧线(前进+右转)
    + / =: 提升速度上限   -: 降低速度上限
    空格: 急停(最高优先级, 立即发布全0速度)
    Ctrl+C: 安全退出(先发布0速度再退出)
    其他任意键: 停车

安全机制:
    1. 0.2s 内未检测到有效 WASDB 输入 -> 判定松开, 自动发布零速
    2. 急停/退出均立即发布全 0 Twist
"""

import select
import signal
import sys
import termios
import threading
import time
import tty

from geometry_msgs.msg import Twist

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

# ---------------- 速度参数 ----------------
LIN_STEP = 0.1      # 每按一次 +/- 线速度上限增量 (m/s)
ANG_STEP = 0.5      # 每按一次 +/- 角速度上限增量 (rad/s)
LIN_MIN = 0.1       # 线速度下限 (m/s)
LIN_MAX = 2.0       # 线速度上限 (m/s)
ANG_MIN = 0.5       # 角速度下限 (rad/s)
ANG_MAX = 5.0       # 角速度上限 (rad/s)
RELEASE_TIMEOUT = 0.2   # 松开判定: 0.2s 无按键 -> 停车 (秒)


class DsTeleopNode(Node):
    def __init__(self):
        super().__init__('ds_teleop_node')

        # ---------------- 发布器 ----------------
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # ---------------- 速度状态 ----------------
        self.lin_max = 0.5   # 当前线速度上限 (m/s)
        self.ang_max = 1.0   # 当前角速度上限 (rad/s)
        self.target_lin = 0.0   # 目标线速度 (m/s)
        self.target_ang = 0.0   # 目标角速度 (rad/s)
        self.last_key_time = 0.0    # 最近一次有效 WASDB 按键时刻
        self.exit_flag = False      # 线程退出标志

        # ---------------- 后台键盘线程 ----------------
        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        # ---------------- 20Hz 松开检测定时器 ----------------
        self.timer = self.create_timer(0.05, self.check_release)

        self.print_help()
        self.get_logger().info('ds_teleop_node 就绪, 按 W/A/S/B/D 遥控, 空格急停')

    # ---------------- 帮助信息 ----------------
    def print_help(self):
        print('\n================ ds_teleop_key 键盘遥控 ================')
        print('  [W] 前进    [S] 后退    [A] 左转    [B]/[D] 右转')
        print('  [Q] 左前弧线  [E] 右前弧线')
        print('  [+/=] 加速档  [-] 减速档  [空格] 急停  [Ctrl+C] 退出')
        print('  松开方向键 0.2s 后自动停车')
        print('========================================================\n')
        self.print_speed()

    def print_speed(self):
        print(f'>>> 当前档位: 线速度上限 {self.lin_max:.1f} m/s | '
              f'角速度上限 {self.ang_max:.1f} rad/s', flush=True)

    # ---------------- 发布 Twist ----------------
    def publish_cmd(self, lin: float, ang: float):
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.cmd_pub.publish(msg)

    # ---------------- 键盘循环(后台线程) ----------------
    def keyboard_loop(self):
        """Cbreak模式非阻塞监听, select超时轮询."""
        fd = sys.stdin.fileno()
        old_attr = termios.tcgetattr(fd)   # 保存原始终端属性, 退出时恢复
        try:
            tty.setcbreak(fd)              # cbreak 模式: 无回显, 单键即读
            while not self.exit_flag:
                # select 0.05s: 无按键时让出 CPU, 且能及时响应 exit_flag
                ready, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not ready:
                    continue
                ch = sys.stdin.read(1)
                if ch == '\x03':            # Ctrl+C
                    self.handle_exit()
                    break
                self.handle_key(ch)
        except Exception as e:
            self.get_logger().error(f'键盘线程异常: {e}')
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)  # 恢复终端

    # ---------------- 按键处理 ----------------
    def handle_key(self, ch: str):
        key = ch.lower()
        if key == ' ':
            # 急停: 最高优先级, 清空目标并立即发布0
            self.target_lin = 0.0
            self.target_ang = 0.0
            self.last_key_time = 0.0
            self.publish_cmd(0.0, 0.0)
            print('!!! 急停 !!!', flush=True)
        elif key in ('+', '='):
            self.lin_max = min(self.lin_max + LIN_STEP, LIN_MAX)
            self.ang_max = min(self.ang_max + ANG_STEP, ANG_MAX)
            self.print_speed()
        elif key == '-':
            self.lin_max = max(self.lin_max - LIN_STEP, LIN_MIN)
            self.ang_max = max(self.ang_max - ANG_STEP, ANG_MIN)
            self.print_speed()
        elif key == 'w':
            self.target_lin = self.lin_max
            self.target_ang = 0.0
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        elif key == 's':
            self.target_lin = -self.lin_max
            self.target_ang = 0.0
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        elif key == 'a':
            self.target_lin = 0.0
            self.target_ang = self.ang_max
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        elif key in ('b', 'd'):
            self.target_lin = 0.0
            self.target_ang = -self.ang_max
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        elif key == 'q':
            # 左前弧线: 前进 + 左转 (线速度正, 角速度正)
            self.target_lin = self.lin_max
            self.target_ang = self.ang_max
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        elif key == 'e':
            # 右前弧线: 前进 + 右转 (线速度正, 角速度负)
            self.target_lin = self.lin_max
            self.target_ang = -self.ang_max
            self.last_key_time = time.monotonic()
            self.publish_cmd(self.target_lin, self.target_ang)
        else:
            # 未知按键: 停车
            self.target_lin = 0.0
            self.target_ang = 0.0
            self.last_key_time = 0.0
            self.publish_cmd(0.0, 0.0)

    # ---------------- 松开检测(20Hz) ----------------
    def check_release(self):
        """0.2s 内无有效 WASDB 输入 -> 判定松开, 发布零速."""
        if self.last_key_time == 0.0:
            return
        if (time.monotonic() - self.last_key_time) > RELEASE_TIMEOUT \
                and (self.target_lin != 0.0 or self.target_ang != 0.0):
            self.target_lin = 0.0
            self.target_ang = 0.0
            self.last_key_time = 0.0
            self.publish_cmd(0.0, 0.0)
            self.get_logger().info('按键松开超时, 自动停车')

    # ---------------- 安全退出 ----------------
    def handle_exit(self):
        """Ctrl+C: 先发布0速度, 再置退出标志."""
        self.publish_cmd(0.0, 0.0)
        self.exit_flag = True
        print('\n退出: 已发布停车指令', flush=True)

    def destroy_node(self):
        self.exit_flag = True
        try:
            self.publish_cmd(0.0, 0.0)   # 兜底: 退出前再发一次0速
        except Exception:
            pass
        if self.key_thread.is_alive():
            self.key_thread.join(timeout=0.5)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DsTeleopNode()

    # 拦截 SIGINT: 不让 rclpy 默认处理直接退出, 交给键盘线程优雅停车
    def sigint_handler(signum, frame):
        node.handle_exit()
        raise SystemExit(0)

    signal.signal(signal.SIGINT, sigint_handler)

    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

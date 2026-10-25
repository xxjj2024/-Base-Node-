#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================
 树莓派 4B (Ubuntu) + USB转RS485 —— 键盘遥控两轮差速小车
 驱动器：森创 DS/RS 二合一伺服驱动器 (Modbus RTU 扩展协议)
================================================================

【为什么不用 0xEA 差速协议？】
 两台电机在车体两侧镜像 180° 对称安装，出厂"正转"方向定义相反
 （实测 A 轴顺时针、B 轴逆时针）。驱动器自带 0xEA 差速协议的
 内部解算不经过轴方向参数，直接下发线速度/角速度会导致原地打转，
 且改 F1.1.003 / Pn6.00C 均无法纠正。
 因此本脚本完全在 Python 端做差速解算，把左右轮 RPM 通过
 0x44 双写指令直接下发给两轴，并提供软件方向翻转开关。

【运行】
    python3 rs485_diff_car.py [/dev/ttyUSB0]
    （需要把当前用户加入 dialout 组以免 sudo：
      sudo usermod -aG dialout $USER ，注销重登后生效）

【按键】
    W 前进   S 后退   A 原地左转   D 原地右转
    组合键（如按住 W+A）走弧线；松开全部按键自动归零
    + / =  全局速度上限 +100     -  全局速度上限 -100
    空格   急停并失能退出         Ctrl+C 安全退出
"""

import sys
import time
import math
import struct
import signal
import threading
import select
import termios
import tty

import serial          # sudo apt install python3-serial
try:
    import evdev       # sudo apt install python3-evdev（推荐，支持真实按下/松开）
    HAVE_EVDEV = True
except ImportError:
    HAVE_EVDEV = False

# ================================================================
# 一、物理修正开关（装车后按实测翻转，True = 该项 RPM 乘 -1）
# ================================================================
A_AXIS_INVERT = True    # A轴(CN1/右轮)出厂正方向与B轴相反 → 软件取反
LINE_INVERT   = True    # 前后行驶方向全局翻转（True: 下发正速度=车头前进）
ANG_INVERT    = False   # 左右转向全局翻转   （True: 下发正角速度=左转）

# ================================================================
# 二、机械参数（与驱动器 Pn6.00A/Pn6.00B 一致）
# ================================================================
WHEEL_DIAMETER_MM = 168.0    # 轮径 mm
WHEEL_TRACK_MM    = 315.0    # 轮距 mm（两轮中心距）

# ================================================================
# 三、速度档位与控制节奏
# ================================================================
LINE_SPEED_STEP = 500        # 单次 W/S 线速度指令  mm/s
ANG_SPEED_STEP  = 500        # 单次 A/D 角速度指令  mrad/s
SPEED_LIMIT     = 1000       # 全局速度上限（+/- 键调整，步进 100）
SEND_PERIOD     = 0.05       # 0x44 报文下发周期 50ms (20Hz)
KEY_RELEASE_TIMEOUT = 0.2    # 看门狗：0.2s 无有效按键视为全部松开
ACCEL_LINE  = 800.0          # 线速度斜坡加速度 mm/s²（起步/停止的柔和程度）
ACCEL_ANG   = 1000.0         # 角速度斜坡加速度 mrad/s²

# ================================================================
# 四、串口与协议
# ================================================================
SERIAL_PORT = "/dev/ttyUSB0"
BAUDRATE    = 115200
STATION_ID  = 0x01

# 预制报文（CRC 已算好，可对照手册）
CMD_ENABLE  = bytes.fromhex("014421003100000100017534")  # 两轴使能
CMD_DISABLE = bytes.fromhex("01442100310000000000E534")  # 两轴失能
CMD_SPIN_HEAD = bytes.fromhex("014423183318")            # 0x44 双写速度指令头
                                                          # 地址: A=0x2318, B=0x3318


# ========================== CRC-16 (Modbus RTU) ==========================
def crc16_modbus(data: bytes) -> int:
    """
    标准 Modbus RTU CRC-16：
      多项式 0xA001（0x8005 的位反转），初值 0xFFFF，低位在先。
    发送时低字节在前、高字节在后。
    """
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(frame: bytes) -> bytes:
    """报文末尾追加 CRC16，低字节在前。"""
    crc = crc16_modbus(frame)
    return frame + bytes([crc & 0xFF, (crc >> 8) & 0xFF])


# ========================== 差速解算与报文打包 ==========================
def build_move_cmd(line_speed: int, ang_speed_mrad: int) -> bytes:
    """
    差速运动学解算 + 0x44 双写两轴 RPM 报文。

    输入:
      line_speed     : 整车线速度 mm/s  （正 = 车头前进）
      ang_speed_mrad : 整车角速度 mrad/s（正 = 左转/逆时针，车顶视角）
    输出:
      完整 Modbus RTU 报文（含 CRC）

    解算（ω 单位换算: mrad/s → rad/s 即 ÷1000）:
      V_left  = V - ω*(W/2)
      V_right = V + ω*(W/2)
      RPM     = V / (π*D) * 60
    轴映射（实测确定）: A轴=CN1=右轮，B轴=CN2=左轮
    """
    # ---- 全局方向修正（物理装配补偿）----
    if LINE_INVERT:
        line_speed = -line_speed
    if ANG_INVERT:
        ang_speed_mrad = -ang_speed_mrad

    # ---- 差速解算（mm/s）----
    v_left  = line_speed - ang_speed_mrad / 1000.0 * (WHEEL_TRACK_MM / 2.0)
    v_right = line_speed + ang_speed_mrad / 1000.0 * (WHEEL_TRACK_MM / 2.0)

    # ---- 线速度 → 转速（rpm）----
    circumference = math.pi * WHEEL_DIAMETER_MM            # 轮周长 mm
    rpm_left  = v_left  / circumference * 60.0
    rpm_right = v_right / circumference * 60.0

    # ---- A/B 轴出厂方向相反，软件翻转统一物理方向 ----
    if A_AXIS_INVERT:
        rpm_right = -rpm_right     # A轴 = 右轮
    # B_AXIS 不翻转（两轴对称翻转等效于全局方向翻转，已由 LINE_INVERT 承担）

    # ---- 限幅并转为 16 位有符号整数（防溢出）----
    rpm_right = max(-32768, min(32767, int(round(rpm_right))))
    rpm_left  = max(-32768, min(32767, int(round(rpm_left))))

    # ---- 打包: 大端 16 位有符号补码（struct '>h' 自动处理负数）----
    a_hi, a_lo = struct.pack(">h", rpm_right)   # A轴 = 右轮
    b_hi, b_lo = struct.pack(">h", rpm_left)    # B轴 = 左轮
    frame = CMD_SPIN_HEAD + bytes([a_hi, a_lo, b_hi, b_lo])
    return append_crc(frame)


# ========================== 差速控制器（串口 + 20Hz 下发线程） ==========================
class DiffCarController:
    """管理串口、共享速度状态、20Hz 发送线程与安全退出。"""

    def __init__(self, port: str):
        self.port = port
        self.ser = None
        # ---- 共享目标速度（由键盘线程写、发送线程读，必须加锁）----
        self.target_line = 0          # 目标线速度 mm/s（键盘直接设定）
        self.target_ang  = 0          # 目标角速度 mrad/s
        self.line_speed  = 0          # 实际下发线速度（斜坡逼近目标）
        self.ang_speed   = 0          # 实际下发角速度
        self.step_line   = LINE_SPEED_STEP   # 线速度档位（+/- 键动态调整）
        self.step_ang    = ANG_SPEED_STEP    # 角速度档位（+/- 键动态调整）
        self.speed_limit = SPEED_LIMIT       # 全局速度上限
        self.lock = threading.Lock()
        # ---- 线程控制 ----
        self.running = True            # 发送线程运行标志
        self.stop_event = threading.Event()   # 急停/退出请求
        self.send_thread = None

    # ---------------- 串口初始化与使能 ----------------
    def open(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=BAUDRATE,
            bytesize=serial.EIGHTBITS,     # 8 位数据位
            parity=serial.PARITY_NONE,     # 无校验
            stopbits=serial.STOPBITS_ONE,  # 1 位停止位
            timeout=0.02,
        )
        print(f"[INFO] 串口已打开: {self.port} @ {BAUDRATE}")
        self.ser.reset_input_buffer()      # 清残留应答

        # 发送使能指令
        self.ser.write(CMD_ENABLE)
        self.ser.flush()
        time.sleep(0.05)
        print(f"[INFO] 使能指令已发送: {CMD_ENABLE.hex(' ').upper()}")

        # 启动 20Hz 发送线程
        self.send_thread = threading.Thread(target=self._send_loop, daemon=True)
        self.send_thread.start()
        print(f"[INFO] 0x44 速度下发线程已启动 (20Hz, 上限 ±{self.speed_limit})")

    # ---------------- 20Hz 固定周期下发线程 ----------------
    def _send_loop(self):
        next_t = time.monotonic()
        while self.running:
            with self.lock:
                tgt_line, tgt_ang = self.target_line, self.target_ang
                # ---- 斜坡加减速：实际下发速度以固定加速度逼近目标 ----
                # 每周期最大速度增量 = 加速度 * 周期时间
                step_line = ACCEL_LINE * SEND_PERIOD
                step_ang  = ACCEL_ANG  * SEND_PERIOD
                self.line_speed = self._ramp(self.line_speed, tgt_line, step_line)
                self.ang_speed  = self._ramp(self.ang_speed,  tgt_ang,  step_ang)
                line, ang = self.line_speed, self.ang_speed
            frame = build_move_cmd(int(line), int(ang))
            try:
                self.ser.write(frame)
                self.ser.flush()
            except (OSError, serial.SerialException) as e:
                print(f"\n[WARN] 串口写入失败: {e}")
                break
            # 固定周期调度（防累计漂移）
            next_t += SEND_PERIOD
            delay = next_t - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                next_t = time.monotonic()

    @staticmethod
    def _ramp(current: float, target: float, max_step: float) -> float:
        """把 current 朝 target 逼近，单步最多变化 max_step（斜坡核心）。"""
        diff = target - current
        if abs(diff) <= max_step:
            return float(target)
        return current + max_step if diff > 0 else current - max_step

    # ---------------- 更新目标速度（键盘线程调用） ----------------
    def set_speed(self, line: int, ang: int):
        """按全局上限限幅后写入共享【目标】速度（实际下发由斜坡逼近）。"""
        lim = self.speed_limit
        line = max(-lim, min(lim, line))
        ang = max(-lim, min(lim, ang))
        with self.lock:
            self.target_line = line
            self.target_ang = ang

    def adjust_limit(self, delta: int):
        """
        +/- 键：线速度与角速度档位同步增减 100。
        （档位 = 每次按 W/A 实际下发的速度值；上限只作保护）
        """
        with self.lock:
            self.step_line = max(100, min(self.speed_limit, self.step_line + delta))
            self.step_ang  = max(100, min(self.speed_limit, self.step_ang + delta))
            sl, sa = self.step_line, self.step_ang
        print(f"\n[INFO] 速度档位: 线速度 ±{sl} mm/s   角速度 ±{sa} mrad/s")

    # ---------------- 急停：立即零速 + 失能（绕过斜坡） ----------------
    def emergency_stop(self):
        print("\n[INFO] 急停：下发 0 RPM 并失能电机")
        with self.lock:
            self.target_line = 0
            self.target_ang = 0
            self.line_speed = 0          # 直接清零，不经斜坡
            self.ang_speed = 0
        try:
            self.ser.write(build_move_cmd(0, 0))     # ① 先发 0 RPM
            self.ser.flush()
            time.sleep(0.05)
            self.ser.write(CMD_DISABLE)              # ② 再发失能指令
            self.ser.flush()
            time.sleep(0.05)
        except Exception as e:
            print(f"[WARN] 急停指令发送异常: {e}")

    # ---------------- 安全关闭 ----------------
    def close(self):
        self.running = False
        if self.send_thread:
            self.send_thread.join(timeout=1.0)
        self.emergency_stop()
        if self.ser and self.ser.is_open:
            self.ser.close()
        print("[INFO] 串口已关闭，程序退出")


# ========================== 键盘监听 ==========================
def find_keyboard():
    """自动扫描 /dev/input/event* 找到带 WASD 按键的键盘设备（需 input 组权限）。"""
    import evdev
    for path in evdev.list_devices():
        try:
            dev = evdev.InputDevice(path)
            caps = dev.capabilities(verbose=False).get(1, [])   # 1 = EV_KEY
            keyset = {c for c in caps}
            # 必须同时具备 W A S D 和空格
            need = {17, 30, 31, 32, 57}     # KEY_W=17 A=30 S=31 D=32 SPACE=57
            if need <= keyset:
                return dev
        except (OSError, PermissionError):
            continue
    return None


def run_keyboard_evdev(car: DiffCarController):
    """
    evdev 方案：读 /dev/input 事件，有真实的按下(1)/松开(0)事件，
    组合键（W+A 边走边转）天然支持，无看门狗猜测。
    """
    import evdev
    from evdev import ecodes as e
    dev = find_keyboard()
    if dev is None:
        print("[WARN] 未找到键盘输入设备（需要 input 组权限：sudo usermod -aG input $USER 后重登）")
        return False
    print(f"[INFO] 键盘设备: {dev.path} ({dev.name})")
    keys = {"w": False, "s": False, "a": False, "d": False}
    keymap = {e.KEY_W: "w", e.KEY_A: "a", e.KEY_S: "s", e.KEY_D: "d"}

    def update():
        with car.lock:
            step_l, step_a = car.step_line, car.step_ang
        line = (step_l if keys["w"] else 0) + (-step_l if keys["s"] else 0)
        ang  = (step_a if keys["a"] else 0) + (-step_a if keys["d"] else 0)
        car.set_speed(line, ang)
        state = "".join(k.upper() for k in "wasd" if keys[k]) or "-"
        print(f"\r[{state:^4}] 线速度: {line:+5d} mm/s   角速度: {ang:+5d} mrad/s   ",
              end="", flush=True)

    update()
    try:
        for event in dev.read_loop():
            if event.type == evdev.ecodes.EV_KEY:
                if event.code == e.KEY_SPACE and event.value == 1:   # 空格急停
                    car.stop_event.set()
                    break
                elif event.code == e.KEY_PLUS or event.code == e.KEY_EQUAL or \
                     event.code == e.KEY_KPPLUS:
                    if event.value == 1:
                        car.adjust_limit(+100)
                elif event.code == e.KEY_MINUS or event.code == e.KEY_KPMINUS:
                    if event.value == 1:
                        car.adjust_limit(-100)
                elif event.code in keymap:
                    keys[keymap[event.code]] = (event.value == 1)   # 1=按下 0=松开
                    update()
    except (OSError, serial.SerialException):
        pass
    car.stop_event.set()
    return True


def run_keyboard_tty(car: DiffCarController):
    """
    备用方案（无 evdev 时）：termios cbreak + select 超时轮询。
    局限：无法感知按键松开事件，靠 0.2s 看门狗猜测，组合键不可靠。
    """
    keys = {"w": False, "s": False, "a": False, "d": False}

    def update():
        """根据按键状态合成目标速度（天然支持组合键与归零）。"""
        with car.lock:
            step_l, step_a = car.step_line, car.step_ang
        line = (step_l if keys["w"] else 0) + (-step_l if keys["s"] else 0)
        ang  = (step_a if keys["a"] else 0) + (-step_a if keys["d"] else 0)
        car.set_speed(line, ang)
        state = "".join(k.upper() for k, v in keys.items() if v) or "-"
        print(f"\r[{state:^4}] 线速度: {line:+5d} mm/s   角速度: {ang:+5d} mrad/s   ",
              end="", flush=True)

    fd = sys.stdin.fileno()
    old_attr = termios.tcgetattr(fd)               # 保存终端原状态
    print("键盘控制已就绪（切换模式）:")
    print("  W/S 前进/后退(再按一次取消)   A/D 左转/右转(再按一次取消)")
    print("  前进+转弯可叠加 => 弧线行驶    0 或 X = 全部清零")
    print("  +/- 调速档位   空格急停   Ctrl+C 退出")
    try:
        tty.setcbreak(fd)                          # cbreak: 单键即读
        while not car.stop_event.is_set():
            # select 最多等 0.05s：兼顾响应速度与 Ctrl+C 信号及时打断
            r, _, _ = select.select([sys.stdin], [], [], 0.05)
            if r:
                ch = sys.stdin.read(1).lower()
                if ch == " ":                      # 空格 = 急停退出
                    car.stop_event.set()
                    break
                elif ch in "+=":                   # 上限 +100
                    car.adjust_limit(+100)
                elif ch == "-":                    # 上限 -100
                    car.adjust_limit(-100)
                elif ch in keys:
                    keys[ch] = True
                    update()
                    # --- 看门狗：持续按住时反复读键，超时判定松开 ---
                    deadline = time.monotonic() + KEY_RELEASE_TIMEOUT
                    while time.monotonic() < deadline and not car.stop_event.is_set():
                        r2, _, _ = select.select([sys.stdin], [], [], 0.05)
                        if not r2:
                            continue
                        ch2 = sys.stdin.read(1).lower()
                        if ch2 == " ":
                            car.stop_event.set()
                            break
                        elif ch2 in "+=":
                            car.adjust_limit(+100)
                            deadline = time.monotonic() + KEY_RELEASE_TIMEOUT
                        elif ch2 == "-":
                            car.adjust_limit(-100)
                            deadline = time.monotonic() + KEY_RELEASE_TIMEOUT
                        elif ch2 in keys:
                            keys[ch2] = True
                            update()
                            deadline = time.monotonic() + KEY_RELEASE_TIMEOUT   # 有输入则续期
                    if car.stop_event.is_set():
                        break
                    for k in keys:                 # 超时无输入 → 全部松开
                        keys[k] = False
                    update()
            # 无按键时循环继续（速度保持，发送线程持续下发）
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_attr)   # 恢复终端设置
    car.stop_event.set()


def run_keyboard(car: DiffCarController):
    """统一入口：优先 evdev（真实按下/松开，组合键可靠），失败退回 termios。"""
    if HAVE_EVDEV:
        if run_keyboard_evdev(car):
            return
    run_keyboard_tty(car)


# ========================== 主程序 ==========================
def main():
    port = sys.argv[1] if len(sys.argv) > 1 else SERIAL_PORT
    car = DiffCarController(port)

    # Ctrl+C 信号 → 触发急停退出（不打断 termios 清理）
    signal.signal(signal.SIGINT, lambda *_: car.stop_event.set())

    try:
        car.open()
        run_keyboard(car)
    except serial.SerialException as e:
        print(f"[FATAL] 串口打开失败: {e}\n"
              f"        检查设备号（ls /dev/ttyUSB*）与权限（usermod -aG dialout $USER）")
    finally:
        car.close()          # 任何退出路径都保证: 停线程→0 RPM→失能→关串口


if __name__ == "__main__":
    main()

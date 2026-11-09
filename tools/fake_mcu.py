#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""假电控板：往 CAN 总线灌 IMU 四元数与弹速帧，同时打印视觉下发的指令帧。

用途：手上没有 C 板 / 电控硬件时，配合 vcan 虚拟 CAN 接口，把
      mt_auto_aim_debug 的整条链路跑通（CBoard 能开、能进 auto_aim 模式、
      aimer 输出的指令帧能被看到）。

依赖：仅 Python 标准库（AF_CAN socket，Python >= 3.9）

准备（一次性）：
    sudo modprobe vcan                        # 若报错见下方说明
    sudo ip link add dev vcan0 type vcan      # 已存在会报 File exists，忽略
    sudo ip link set up vcan0
    ip link show vcan0                        # 确认 <...,UP,...>

    # 若 modprobe vcan 报 module not found：
    # sudo apt install linux-modules-extra-$(uname -r)

把配置里的接口名改成 vcan0（改前先备份）：
    cp configs/sentry.yaml configs/sentry.yaml.bak
    sed -i 's/can_interface: "can0"/can_interface: "vcan0"/' configs/sentry.yaml

两个终端分别运行：
    sudo python3 tools/fake_mcu.py -i vcan0          # 终端 A
    ./build/mt_auto_aim_debug                        # 终端 B（项目根目录）

帧格式与 io/cboard.cpp 严格对应，全部大端（big-endian）：
  四元数帧 quaternion_canid
      data[0..1] = x    data[2..3] = y
      data[4..5] = z    data[6..7] = w      均为 int16，真实值 x 1e4
      注意顺序是 x,y,z,w —— 与常见 w,x,y,z 不同
  弹速帧   bullet_speed_canid
      data[0..1] = bullet_speed (x1e2)   data[2] = mode
      data[3]    = shoot_mode            data[4..5] = ft_angle (x1e4)
  指令帧   send_canid（本脚本只收不发）
      data[0] = control   data[1] = shoot
      data[2..3] = yaw (x1e4)   data[4..5] = pitch (x1e4)   data[6..7] = horizon_distance (x1e4)
"""

import argparse
import math
import select
import socket
import struct
import sys
import time

# struct can_frame 是 16 字节：can_id(4) dlc(1) pad(3) data(8)
CAN_FRAME_FMT = "=IB3x8s"

MODE_IDLE = 0
MODE_AUTO_AIM = 1


def i16(value):
    """四舍五入并夹到 int16 范围内，避免 struct.error。"""
    return max(-32768, min(32767, int(round(value))))


def pack_quat(yaw: float, pitch: float) -> bytes:
    """构造绕 z 轴 yaw、绕 y 轴 pitch 的旋转四元数，按 x,y,z,w 顺序打包。"""
    cyaw, syaw = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    cpitch, spitch = math.cos(pitch / 2.0), math.sin(pitch / 2.0)

    # q = q_yaw(z) ⊗ q_pitch(y)
    x = -syaw * spitch
    y = cyaw * spitch
    z = syaw * cpitch
    w = cyaw * cpitch

    return struct.pack(">hhhh", i16(x * 1e4), i16(y * 1e4), i16(z * 1e4), i16(w * 1e4))


def pack_bullet(speed: float, mode: int, shoot_mode: int, ft_angle: float = 0.0) -> bytes:
    data = struct.pack(">hBBh", i16(speed * 100), mode, shoot_mode, i16(ft_angle * 1e4))
    return data + b"\x00" * (8 - len(data))


def parse_command(data: bytes):
    yaw = struct.unpack(">h", data[2:4])[0] / 1e4
    pitch = struct.unpack(">h", data[4:6])[0] / 1e4
    horizon = struct.unpack(">h", data[6:8])[0] / 1e4
    return data[0], data[1], yaw, pitch, horizon


def open_can(interface: str) -> socket.socket:
    sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
    try:
        sock.bind((interface,))
    except (OSError, TypeError) as exc:
        sock.close()
        sys.exit(
            "绑定 CAN 接口 %r 失败: %s\n"
            "  1. 确认接口存在且已 up: ip link show %s\n"
            "  2. 确认用 sudo 运行（AF_CAN 需要 CAP_NET_RAW）\n"
            "  3. vcan 模块没加载: sudo modprobe vcan" % (interface, exc, interface)
        )
    return sock


def main() -> int:
    parser = argparse.ArgumentParser(
        description="假电控板：灌四元数/弹速帧，打印视觉下发的指令帧",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument("-i", "--interface", default="vcan0", help="CAN 接口名")
    parser.add_argument("-q", "--quat-id", type=lambda s: int(s, 0), default=0x01,
                        help="四元数帧 CAN ID（对齐 yaml 的 quaternion_canid）")
    parser.add_argument("-b", "--bullet-id", type=lambda s: int(s, 0), default=0x110,
                        help="弹速帧 CAN ID（对齐 yaml 的 bullet_speed_canid）")
    parser.add_argument("-c", "--cmd-id", type=lambda s: int(s, 0), default=0xFF,
                        help="视觉下发指令帧 CAN ID（对齐 yaml 的 send_canid）")
    parser.add_argument("--speed", type=float, default=15.0, help="弹速 m/s")
    parser.add_argument("--mode", type=int, default=MODE_AUTO_AIM,
                        help="0=idle 1=auto_aim 2=small_buff 3=big_buff 4=outpost")
    parser.add_argument("--shoot-mode", type=int, default=0,
                        help="0=left 1=right 2=both")
    parser.add_argument("--amp", type=float, default=25.0, help="yaw 摆幅（度）")
    parser.add_argument("--freq", type=float, default=0.5, help="yaw 摆动频率 Hz")
    parser.add_argument("--pitch-amp", type=float, default=0.0, help="pitch 摆幅（度）")
    parser.add_argument("--spin", action="store_true", help="匀速旋转而非正弦摆动")
    parser.add_argument("--rate", type=float, default=200.0, help="四元数发送频率 Hz")
    args = parser.parse_args()

    sock = open_can(args.interface)
    sock.setblocking(False)

    print("[fake_mcu] 接口 %s 已绑定" % args.interface)
    print("[fake_mcu] 四元数 0x%02X @ %.0fHz | 弹速 0x%02X = %.1f m/s | mode=%d"
          % (args.quat_id, args.rate, args.bullet_id, args.speed, args.mode))
    print("[fake_mcu] 等待视觉下发指令帧 (0x%02X)... Ctrl-C 退出" % args.cmd_id)

    amp = math.radians(args.amp)
    pitch_amp = math.radians(args.pitch_amp)
    quat_period = 1.0 / args.rate
    bullet_period = 0.1

    t0 = time.perf_counter()
    next_quat = t0
    next_bullet = t0
    last_cmd_print = 0.0
    cmd_count = 0

    try:
        while True:
            now = time.perf_counter()
            elapsed = now - t0

            # ---- 四元数帧 ----
            if now >= next_quat:
                if args.spin:
                    yaw = 2.0 * math.pi * args.freq * elapsed
                else:
                    yaw = amp * math.sin(2.0 * math.pi * args.freq * elapsed)
                pitch = pitch_amp * math.sin(2.0 * math.pi * args.freq * 0.5 * elapsed)

                sock.send(struct.pack(CAN_FRAME_FMT, args.quat_id, 8, pack_quat(yaw, pitch)))
                next_quat += quat_period
                if next_quat < now:          # 落后太多就直接对齐，避免追帧风暴
                    next_quat = now + quat_period

            # ---- 弹速帧 ----
            if now >= next_bullet:
                sock.send(struct.pack(
                    CAN_FRAME_FMT, args.bullet_id, 8,
                    pack_bullet(args.speed, args.mode, args.shoot_mode)))
                next_bullet += bullet_period
                if next_bullet < now:
                    next_bullet = now + bullet_period

            # ---- 回收视觉下发的指令帧 ----
            readable, _, _ = select.select([sock], [], [], 0)
            if readable:
                try:
                    raw = sock.recv(16)
                except BlockingIOError:
                    raw = b""
                if len(raw) >= 16:
                    can_id = struct.unpack("=I", raw[0:4])[0] & 0x1FFFFFFF
                    dlc = raw[4]
                    data = raw[8:16]
                    if can_id == args.cmd_id and dlc >= 8:
                        cmd_count += 1
                        if now - last_cmd_print > 0.2:
                            control, shoot, yaw, pitch, horizon = parse_command(data)
                            print("[指令] #%-6d control=%d shoot=%d yaw=%+8.2f° "
                                  "pitch=%+8.2f° horizon=%6.2f m"
                                  % (cmd_count, control, shoot,
                                     math.degrees(yaw), math.degrees(pitch), horizon))
                            last_cmd_print = now

            time.sleep(0.001)

    except KeyboardInterrupt:
        print("\n[fake_mcu] 退出，共收到 %d 帧指令" % cmd_count)
    finally:
        sock.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())

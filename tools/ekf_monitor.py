#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""EKF 监视器：接收 mt_auto_aim_debug 通过 Plotter 发出的 UDP 数据流，画实时曲线。

配合标定流程使用（P0 第 2~5 步）：
    # 终端 A：假电控（四元数冻结为 identity，世界系=相机系）
    sudo python3 tools/fake_mcu.py -i can0 --amp 0
    # 终端 B：主程序
    ./build/mt_auto_aim_debug configs/mindvision_vm.yaml
    # 终端 C：本监视器
    python3 tools/ekf_monitor.py

数据来源 tools/plotter.cpp：UDP -> 127.0.0.1:9870，每帧一个 json。
可用 key（mt_auto_aim_debug.cpp 里 data[...] 的全部字段）：
  状态   x vx y vy z vz a w r l h last_id
  卡方   residual_yaw residual_pitch residual_distance residual_angle
         nis nees nis_fail nees_fail recent_nis_failures
  观测   z_yaw z_pitch z_dist z_angle（建目标后才有）
  其他   armor_num armor_x armor_y armor_yaw armor_yaw_raw
         gimbal_yaw gimbal_pitch bullet_speed t

依赖：matplotlib（可选）。没装时自动降级为终端统计模式：
    sudo apt install python3-matplotlib   # 或 pip3 install matplotlib

常用组合：
    python3 tools/ekf_monitor.py                          # 默认: nis x y z r
    python3 tools/ekf_monitor.py --keys nis,w,a,r        # 测转速跟踪
    python3 tools/ekf_monitor.py --keys z_dist,x,y       # 看原始观测
"""

import argparse
import json
import socket
import sys
import threading
import time
from collections import deque

try:
    import matplotlib.pyplot as plt
    HAS_MPL = True
except ImportError:
    HAS_MPL = False

DEFAULT_KEYS = "nis,x,y,z,r"
STATS_WINDOW = 200          # nis 滑动统计窗口（帧）


class Receiver:
    def __init__(self, port, keys):
        self.port = port
        self.keys = [k.strip() for k in keys.split(",") if k.strip()]
        self.latest = {}
        self.series = {k: deque(maxlen=3000) for k in self.keys}
        self.nis_hist = deque(maxlen=3000)
        self.lock = threading.Lock()
        self.frame_count = 0

    def start(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        sock.bind(("127.0.0.1", self.port))
        sock.settimeout(0.5)

        def worker():
            while True:
                try:
                    raw, _ = sock.recvfrom(65536)
                except socket.timeout:
                    continue
                try:
                    data = json.loads(raw.decode("utf-8"))
                except (ValueError, UnicodeDecodeError):
                    continue
                with self.lock:
                    self.frame_count += 1
                    self.latest = data
                    for k in self.keys:
                        if k in data:
                            try:
                                self.series[k].append(float(data[k]))
                            except (TypeError, ValueError):
                                pass
                    if "nis" in data:
                        try:
                            self.nis_hist.append(float(data["nis"]))
                        except (TypeError, ValueError):
                            pass

        t = threading.Thread(target=worker, daemon=True)
        t.start()

    def nis_stats(self):
        with self.lock:
            vals = list(self.nis_hist)[-STATS_WINDOW:]
        if not vals:
            return None
        n = len(vals)
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / max(1, n - 1)
        over = sum(1 for v in vals if v > 9.488)  # chi2(4) 95%
        return n, mean, var ** 0.5, over


def terminal_mode(rx):
    """无 matplotlib 时的降级模式：每秒打印一行摘要。"""
    print("[ekf_monitor] 无 matplotlib，降级为终端统计模式（Ctrl-C 退出）")
    print("%-12s %8s %8s %8s %8s %8s  %s" %
          ("time", "nis均值", "nisσ", "超限%", "x", "z", "状态"))
    t0 = time.time()
    while True:
        try:
            time.sleep(1.0)
        except KeyboardInterrupt:
            break
        stats = rx.nis_stats()
        with rx.lock:
            lat = rx.latest
        if stats is None:
            print("%-12.1f  %-28s" % (time.time() - t0, "等待数据..."))
            continue
        n, mean, sd, over = stats
        x = lat.get("x", float("nan"))
        z = lat.get("z", float("nan"))
        state = "TRACK" if lat.get("armor_num", 0) else "----"
        print("%-12.1f %8.2f %8.2f %7.1f%% %8.3f %8.3f  %s (n=%d)" %
              (time.time() - t0, mean, sd, 100.0 * over / n, x, z, state, n))


def plot_mode(rx, window):
    n = len(rx.keys)
    fig, axes = plt.subplots(n, 1, figsize=(9, 2.2 * n), squeeze=False)
    lines = {}
    for ax, key in zip(axes[:, 0], rx.keys):
        line, = ax.plot([], lw=1.2)
        lines[key] = line
        ax.set_ylabel(key)
        ax.grid(True, alpha=0.3)
    axes[-1, 0].set_xlabel("frame")
    fig.suptitle("ekf_monitor  UDP:9870")
    plt.ion()
    plt.show()

    while True:
        with rx.lock:
            for key in rx.keys:
                vals = list(rx.series[key])[-window:]
        for key in rx.keys:
            lines[key].set_ydata(vals)
            lines[key].set_xdata(range(len(vals)))
            ax = lines[key].axes
            ax.relim()
            ax.autoscale_view()

        stats = rx.nis_stats()
        with rx.lock:
            lat = rx.latest
        if stats:
            n, mean, sd, over = stats
            fig.suptitle(
                "nis: mean=%.2f (target 4.00)  sd=%.2f  chi2(4)95%%超限 %d/%d  |  "
                "armor=%s  x=%.2f y=%.2f z=%.2f r=%.3f w=%.2f" %
                (mean, sd, over, n, "yes" if lat.get("armor_num") else "no",
                 lat.get("x", float("nan")), lat.get("y", float("nan")),
                 lat.get("z", float("nan")), lat.get("r", float("nan")),
                 lat.get("w", float("nan"))))
        fig.canvas.draw_idle()
        fig.canvas.flush_events()
        time.sleep(0.2)


def main():
    parser = argparse.ArgumentParser(
        description="EKF 实时监视器：接收 Plotter 的 UDP 数据流",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--port", type=int, default=9870, help="UDP 端口")
    parser.add_argument("--keys", default=DEFAULT_KEYS,
                        help="要画的字段，逗号分隔")
    parser.add_argument("--window", type=int, default=600,
                        help="曲线显示的帧数窗口")
    args = parser.parse_args()

    rx = Receiver(args.port, args.keys)
    rx.start()
    print("[ekf_monitor] 监听 127.0.0.1:%d，字段: %s" % (args.port, args.keys))
    if not HAS_MPL:
        terminal_mode(rx)
    else:
        try:
            plot_mode(rx, args.window)
        except KeyboardInterrupt:
            pass
    print("\n[ekf_monitor] 退出，共收到 %d 帧" % rx.frame_count)
    return 0


if __name__ == "__main__":
    sys.exit(main())

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
    python3 tools/ekf_monitor.py --stats 30 \\
        --keys z_yaw,z_pitch,z_dist,z_angle,nis          # 静止采 30s，直接出 R 建议值
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

    def nis_stats(self, window=STATS_WINDOW):
        """window 为 0 时用全部样本（统计模式），否则取最近 window 帧（实时模式）。"""
        with self.lock:
            hist = list(self.nis_hist)
        vals = hist[-window:] if window > 0 else hist
        if not vals:
            return None
        n = len(vals)
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / max(1, n - 1)
        over = sum(1 for v in vals if v > 9.488)  # chi2(4) 95%
        return n, mean, var ** 0.5, over

    def key_stats(self, key):
        """单个字段的样本数、均值、标准差、最小最大值、以及相邻重复对数。

        dup / pairs 是判断"同一帧被重复喂入"的关键指标：
        真实相机即使 400fps，相邻帧也应有微小差异，重复率接近 0；
        若重复率很高，说明取图线程在自由跑、把同一帧反复送进了 EKF。
        """
        with self.lock:
            vals = list(self.series[key])
        if len(vals) < 2:
            return None
        n = len(vals)
        mean = sum(vals) / n
        var = sum((v - mean) ** 2 for v in vals) / (n - 1)
        dup = sum(1 for i in range(1, n) if vals[i] == vals[i - 1])
        return n, mean, var ** 0.5, min(vals), max(vals), dup, n - 1


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
            snapshot = {key: list(rx.series[key])[-window:] for key in rx.keys}
        for key in rx.keys:
            vals = snapshot[key]
            lines[key].set_ydata(vals)
            lines[key].set_xdata(range(len(vals)))
            if vals:                       # 空数据时保持上一个视图，避免坐标轴乱跳
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


def stats_mode(rx, duration):
    """采集固定时长后打印各字段的均值/标准差/峰峰值，并给出 R_dig 建议方差。

    目标静止时，观测值的波动即纯观测噪声，因此 σ² 可直接作为对应 R 分量。
    """
    print("[ekf_monitor] 统计模式：采集 %.1f 秒，保持目标静止..." % duration)
    for remain in range(int(duration), 0, -1):
        time.sleep(1.0)
        print("  剩余 %2d s   已收 %d 帧" % (remain - 1, rx.frame_count), end="\r")
    print()

    print("\n=== 观测统计（%.1f s，总帧 %d）===" % (duration, rx.frame_count))
    print("%-10s %7s %11s %10s %10s %13s %8s" %
          ("字段", "n", "均值", "σ", "峰峰值", "σ²(=R建议)", "重复率"))
    r_map = {"z_yaw": "R_dig[0] yaw", "z_pitch": "R_dig[1] pitch",
             "z_dist": "R_dig[2] dist", "z_angle": "R_dig[3] angle"}
    for key in rx.keys:
        st = rx.key_stats(key)
        if st is None:
            print("%-10s  样本不足（该字段可能未出现）" % key)
            continue
        n, mean, sd, lo, hi, dup, pairs = st
        print("%-10s %7d %11.5f %10.5f %10.5f %13.4e %7.1f%%   %s" %
              (key, n, mean, sd, hi - lo, sd * sd, 100.0 * dup / max(1, pairs),
               r_map.get(key, "")))

    # 先判断是否存在"同一帧重复喂入"，这会让 nis 失效，必须先排除
    dup_flags = {}
    for key in rx.keys:
        st = rx.key_stats(key)
        if st:
            dup_flags[key] = 100.0 * st[5] / max(1, st[6])

    print()
    dup_detected = False
    worst = [(k, v) for k, v in dup_flags.items()
             if k.startswith("z_") and v > 10.0]
    if worst:
        dup_detected = True
        print("!! 警告：以下观测字段相邻帧重复率异常高，说明同一帧被重复送入 EKF：")
        for k, v in sorted(worst, key=lambda kv: -kv[1]):
            print("     %-10s 重复率 %.1f%%" % (k, v))
        print("   → 此时 nis 会被压到接近 0，读数不可用。")
        print("   → 先修取图/主循环帧率，再回来标 R。")
        print()

    stats = rx.nis_stats(window=0)
    if stats:
        n, mean, sd, over = stats
        print("\nnis: 均值 %.2f（理论 4.00）  标准差 %.2f  超 χ²(4)95%% 阈值 %d/%d 帧"
              % (mean, sd, over, n))
        if dup_detected:
            print("  → 存在重复帧，nis 不反映真实的一致性，本节结论作废")
        elif mean > 6:
            print("  → nis 明显偏大：滤波器过度自信，R 或 Q 偏小")
        elif mean < 2:
            print("  → nis 偏小：R 给得偏大（对观测过于不信任）")
        else:
            print("  → nis 量级正常")
    print("\n提示：以上 σ² 是「当前距离与姿态」下的观测方差；"
          "R_dig 的 dist/angle 两项随几何变化，换距离需复核。")


def main():
    parser = argparse.ArgumentParser(
        description="EKF 实时监视器：接收 Plotter 的 UDP 数据流",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    parser.add_argument("--port", type=int, default=9870, help="UDP 端口")
    parser.add_argument("--keys", default=DEFAULT_KEYS,
                        help="要画的字段，逗号分隔")
    parser.add_argument("--window", type=int, default=600,
                        help="曲线显示的帧数窗口")
    parser.add_argument("--stats", type=float, default=0,
                        help="采集 N 秒后打印统计并退出（用于标定 R）；0 表示持续绘图")
    args = parser.parse_args()

    rx = Receiver(args.port, args.keys)
    rx.start()
    print("[ekf_monitor] 监听 127.0.0.1:%d，字段: %s" % (args.port, args.keys))
    if args.stats > 0:
        stats_mode(rx, args.stats)
    elif not HAS_MPL:
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

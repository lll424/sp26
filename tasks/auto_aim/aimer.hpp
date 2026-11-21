#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/cboard.hpp"
#include "io/command.hpp"
#include "target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
  // 这个瞄准点对应的槽位号（-1 = 未知）。
  // 有了它才能把高度还原成「中心高度 + 该槽位的 h」，
  // 从而彻底摆脱 predict 里 vz*dt 的污染。
  int id = -1;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  // 输出限幅用的上一条指令（2026-11-21）
  // 上游（EKF 预测、迭代弹道）偶发尖峰会让 yaw/pitch 瞬间跳十几度，
  // 直接发到 CAN 就是让云台暴冲。这里兜一道底，保证指令连续。
  double last_yaw_ = 0.0;
  double last_pitch_ = 0.0;
  bool has_last_cmd_ = false;

  // 垂直速度的低通滤波状态（2026-11-21）
  // vz 的观测噪声（装甲板只有 56mm 高，垂直方向像素少）峰值 1.5m/s，
  // 比真实垂直运动（通常 <0.3m/s）还大。直接用它算垂直提前量会把
  // 噪声经「z偏→pitch大→飞时长→z更偏」的正反馈放大成几十厘米。
  // 但不能干脆不用 —— 垂直提前量是自瞄的必要功能。
  // 解法：限幅去掉物理上不可能的尖峰，再做一阶低通把快变噪声和慢变运动分开。
  double vz_filtered_ = 0.0;
  std::chrono::steady_clock::time_point last_aim_time_;
  bool has_last_aim_time_ = false;

  // 静止时中心高度的低通状态（2026-11-21）
  // EKF 的 z 单帧观测精度约 1.5mm，实测峰峰却有 2.7cm（18 倍），
  // 说明有偶发离群值（检测框抖动等）。静止时对这个量做低通，
  // 把偶发跳变摊平；目标在动时直接跟随，不引入滞后。
  double z_lp_ = 0.0;
  bool has_z_lp_ = false;
  // 静止判定的迟滞状态（2026-11-21）
  // 如果只用单一阈值，目标速度刚好在阈值附近时会在「静止/运动」之间反复切换，
  // 导致 vz_filtered 在 0 和真实值之间跳、z_lp 在滞后值和实时值之间跳，
  // 反而制造出新的抖动。进用 0.2、出用 0.3，留 0.1 的死区。
  bool was_static_ = false;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
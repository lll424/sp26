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

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP
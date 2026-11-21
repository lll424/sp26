#include "aimer.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <vector>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/trajectory.hpp"

namespace auto_aim
{
Aimer::Aimer(const std::string & config_path)
: left_yaw_offset_(std::nullopt), right_yaw_offset_(std::nullopt)
{
  auto yaml = YAML::LoadFile(config_path);
  yaw_offset_ = yaml["yaw_offset"].as<double>() / 57.3;        // degree to rad
  pitch_offset_ = yaml["pitch_offset"].as<double>() / 57.3;    // degree to rad
  comming_angle_ = yaml["comming_angle"].as<double>() / 57.3;  // degree to rad
  leaving_angle_ = yaml["leaving_angle"].as<double>() / 57.3;  // degree to rad
  high_speed_delay_time_ = yaml["high_speed_delay_time"].as<double>();
  low_speed_delay_time_ = yaml["low_speed_delay_time"].as<double>();
  decision_speed_ = yaml["decision_speed"].as<double>();
  if (yaml["left_yaw_offset"].IsDefined() && yaml["right_yaw_offset"].IsDefined()) {
    left_yaw_offset_ = yaml["left_yaw_offset"].as<double>() / 57.3;    // degree to rad
    right_yaw_offset_ = yaml["right_yaw_offset"].as<double>() / 57.3;  // degree to rad
    tools::logger()->info("[Aimer] successfully loading shootmode");
  }
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  bool to_now)
{
  if (targets.empty()) return {false, false, 0, 0};
  auto target = targets.front();

  // 高度锚点：在【任何 predict 之前】记下中心高度（2026-11-21）
  // 装甲板高度在实战中几乎不变（车不跳、不平地上下），所以 vz 的非零值
  // 基本都是观测噪声（装甲板只有 56mm 高，垂直方向像素少）。
  // 一旦允许 predict 用 vz 推高度，会形成正反馈：
  //   z 偏 -> pitch 变大 -> fly_time 变长 -> 下一次 predict 的 dt 更大 -> z 更偏
  // 实测 aim_z 的 sigma 只有 1cm，峰峰值却有 38cm，而 EKF 的 z 只跳 1mm。
  // 所以瞄准点的高度一律钳在锚点附近（容差覆盖 h 的几十毫米偏移）。
  const double z_anchor = target.ekf_x()[4];
  // 容差要按 h 的实际估计自适应：1/3 号板的 z 本来就等于 中心 + h，
  // 固定用一个大容差会漏掉真正该钳的漂移，固定用小容差又会削掉合法的 h。
  const double h_est = std::abs(target.ekf_x()[10]);
  const double MAX_Z_DEV = std::max(0.05, h_est + 0.02);  // 至少 5cm，再给 h 留 2cm 余量
  auto clamp_z = [&](AimPoint & ap) {
    if (!ap.valid) return;
    double dz = ap.xyza[2] - z_anchor;
    if (std::abs(dz) > MAX_Z_DEV) {
      ap.xyza[2] = z_anchor + (dz > 0 ? MAX_Z_DEV : -MAX_Z_DEV);
    }
  };

  auto ekf = target.ekf();
  double delay_time =
    target.ekf_x()[7] > decision_speed_ ? high_speed_delay_time_ : low_speed_delay_time_;

  if (bullet_speed < 14) bullet_speed = 23;

  // 考虑detecor和tracker所消耗的时间，此外假设aimer的用时可忽略不计
  auto future = timestamp;
  if (to_now) {
    double dt;
    dt = tools::delta_time(std::chrono::steady_clock::now(), timestamp) + delay_time;
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  else {
    auto dt = 0.005 + delay_time;  //detector-aimer耗时0.005+发弹延时0.1
    // tools::logger()->info("dt is {:.4f} second", dt);
    future += std::chrono::microseconds(int(dt * 1e6));
    target.predict(future);
  }

  auto aim_point0 = choose_aim_point(target);
  clamp_z(aim_point0);
  debug_aim_point = aim_point0;
  if (!aim_point0.valid) {
    // tools::logger()->debug("Invalid aim_point0.");
    return {false, false, 0, 0};
  }

  Eigen::Vector3d xyz0 = aim_point0.xyza.head(3);
  auto d0 = std::sqrt(xyz0[0] * xyz0[0] + xyz0[1] * xyz0[1]);
  tools::Trajectory trajectory0(bullet_speed, d0, xyz0[2]);
  if (trajectory0.unsolvable) {
    tools::logger()->debug(
      "[Aimer] Unsolvable trajectory0: {:.2f} {:.2f} {:.2f}", bullet_speed, d0, xyz0[2]);
    debug_aim_point.valid = false;
    return {false, false, 0, 0};
  }

  // 迭代求解飞行时间 (最多10次，收敛条件：相邻两次fly_time差 <0.001)
  bool converged = false;
  double prev_fly_time = trajectory0.fly_time;
  tools::Trajectory current_traj = trajectory0;
  std::vector<Target> iteration_target(10, target);  // 创建10个目标副本用于迭代预测

  for (int iter = 0; iter < 10; ++iter) {
    // 预测目标在 future + prev_fly_time 时刻的位置
    auto predict_time = future + std::chrono::microseconds(static_cast<int>(prev_fly_time * 1e6));
    iteration_target[iter].predict(predict_time);

    // 计算瞄准点
    auto aim_point = choose_aim_point(iteration_target[iter]);
    clamp_z(aim_point);
    debug_aim_point = aim_point;
    if (!aim_point.valid) {
      return {false, false, 0, 0};
    }

    // 计算新弹道
    Eigen::Vector3d xyz = aim_point.xyza.head(3);

    // 发散检测（2026-11-21 加）
    // 迭代里每轮都会 predict 到 future + fly_time，而 fly_time 又依赖上一轮的 pitch，
    // 构成正反馈：z 偏 → pitch 变大 → fly_time 变长 → z 偏得更多。
    // vz 的观测噪声（装甲板只有 56mm 高，垂直方向像素少、噪声大）经这个正反馈
    // 会被放大到几十厘米。实测 aim_z 的 sigma 只有 1cm，峰峰值却有 38cm。
    // 这里用首帧结果做锚：偏离超过 5cm 就判定发散，退回首帧，不再迭代。
    constexpr double MAX_Z_DRIFT = 0.05;  // m
    if (std::abs(xyz.z() - xyz0.z()) > MAX_Z_DRIFT) {
      debug_aim_point = aim_point0;
      current_traj = trajectory0;
      break;
    }

    double d = std::sqrt(xyz.x() * xyz.x() + xyz.y() * xyz.y());
    current_traj = tools::Trajectory(bullet_speed, d, xyz.z());

    // 检查弹道是否可解
    if (current_traj.unsolvable) {
      tools::logger()->debug(
        "[Aimer] Unsolvable trajectory in iter {}: speed={:.2f}, d={:.2f}, z={:.2f}", iter + 1,
        bullet_speed, d, xyz.z());
      debug_aim_point.valid = false;
      return {false, false, 0, 0};
    }

    // 检查收敛条件
    if (std::abs(current_traj.fly_time - prev_fly_time) < 0.001) {
      converged = true;
      break;
    }
    prev_fly_time = current_traj.fly_time;
  }

  // 计算最终角度
  Eigen::Vector3d final_xyz = debug_aim_point.xyza.head(3);
  double yaw = std::atan2(final_xyz.y(), final_xyz.x()) + yaw_offset_;
  double pitch = -(current_traj.pitch + pitch_offset_);  //世界坐标系下pitch向上为负

  // 输出限幅（2026-11-21 加）
  // 上游偶发尖峰（EKF 预测被 vz 噪声推偏、迭代发散、跳板）会让 yaw/pitch
  // 在一帧内跳十几度，直接发 CAN 就是云台暴冲。这里限制单次调用的最大变化量。
  // 8°/次 × 90Hz = 720°/s，远超云台实际能力，正常跟踪完全不受影响；
  // 只有真正的跳变会被削成几帧的平滑过渡。
  constexpr double MAX_STEP = 8.0 / 57.3;  // rad
  if (has_last_cmd_) {
    double dyaw = tools::limit_rad(yaw - last_yaw_);
    if (std::abs(dyaw) > MAX_STEP) yaw = last_yaw_ + (dyaw > 0 ? MAX_STEP : -MAX_STEP);
    double dpitch = pitch - last_pitch_;
    if (std::abs(dpitch) > MAX_STEP) pitch = last_pitch_ + (dpitch > 0 ? MAX_STEP : -MAX_STEP);
  }
  last_yaw_ = yaw;
  last_pitch_ = pitch;
  has_last_cmd_ = true;

  return {true, false, yaw, pitch};
}

io::Command Aimer::aim(
  std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
  io::ShootMode shoot_mode, bool to_now)
{
  double yaw_offset;
  if (shoot_mode == io::left_shoot && left_yaw_offset_.has_value()) {
    yaw_offset = left_yaw_offset_.value();
  } else if (shoot_mode == io::right_shoot && right_yaw_offset_.has_value()) {
    yaw_offset = right_yaw_offset_.value();
  } else {
    yaw_offset = yaw_offset_;
  }

  auto command = aim(targets, timestamp, bullet_speed, to_now);
  command.yaw = command.yaw - yaw_offset_ + yaw_offset;

  return command;
}

AimPoint Aimer::choose_aim_point(const Target & target)
{
  Eigen::VectorXd ekf_x = target.ekf_x();
  std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
  auto armor_num = armor_xyza_list.size();
  // 如果装甲板未发生过跳变，则只有当前装甲板的位置已知
  if (!target.jumped) return {true, armor_xyza_list[0]};

  // 整车旋转中心的球坐标yaw
  auto center_yaw = std::atan2(ekf_x[2], ekf_x[0]);

  // 如果delta_angle为0，则该装甲板中心和整车中心的连线在世界坐标系的xy平面过原点
  std::vector<double> delta_angle_list;
  for (int i = 0; i < armor_num; i++) {
    auto delta_angle = tools::limit_rad(armor_xyza_list[i][3] - center_yaw);
    delta_angle_list.emplace_back(delta_angle);
  }

  // 不考虑小陀螺
  if (std::abs(target.ekf_x()[8]) <= 2 && target.name != ArmorName::outpost) {
    // 选择在可射击范围内的装甲板
    std::vector<int> id_list;
    for (int i = 0; i < armor_num; i++) {
      if (std::abs(delta_angle_list[i]) > 60 / 57.3) continue;
      id_list.push_back(i);
    }
    // 绝无可能
    if (id_list.empty()) {
      tools::logger()->warn("Empty id list!");
      return {false, armor_xyza_list[0]};
    }

    // 锁定模式：防止在两个都呈45度的装甲板之间来回切换
    if (id_list.size() > 1) {
      int id0 = id_list[0], id1 = id_list[1];

      // 未处于锁定模式时，选择delta_angle绝对值较小的装甲板，进入锁定模式
      if (lock_id_ != id0 && lock_id_ != id1)
        lock_id_ = (std::abs(delta_angle_list[id0]) < std::abs(delta_angle_list[id1])) ? id0 : id1;

      return {true, armor_xyza_list[lock_id_]};
    }

    // 只有一个装甲板在可射击范围内时，退出锁定模式
    lock_id_ = -1;
    return {true, armor_xyza_list[id_list[0]]};
  }

  double coming_angle, leaving_angle;
  if (target.name == ArmorName::outpost) {
    coming_angle = 70 / 57.3;
    leaving_angle = 30 / 57.3;
  } else {
    coming_angle = comming_angle_;
    leaving_angle = leaving_angle_;
  }

  // 在小陀螺时，一侧的装甲板不断出现，另一侧的装甲板不断消失，显然前者被打中的概率更高
  for (int i = 0; i < armor_num; i++) {
    if (std::abs(delta_angle_list[i]) > coming_angle) continue;
    if (ekf_x[7] > 0 && delta_angle_list[i] < leaving_angle) return {true, armor_xyza_list[i]};
    if (ekf_x[7] < 0 && delta_angle_list[i] > -leaving_angle) return {true, armor_xyza_list[i]};
  }

  return {false, armor_xyza_list[0]};
}

}  // namespace auto_aim
#include "trajectory.hpp"

#include <cmath>

namespace tools
{
constexpr double g = 9.7833;

Trajectory::Trajectory(const double v0, const double d, const double h)
{
  // 弹速异常保护（2026-11-21 加）
  // v0 = 0 时 a = g*d²/(2v0²) 会变成 inf/nan，后面的 `if (delta < 0)` 对 nan
  // 判为 false，会继续算出 nan 的 pitch；转成 int16 发到 CAN 就是任意值，
  // 云台会暴冲。弹速没收到（cboard 初始值 0）或解析失败时都可能触发。
  if (v0 < 1.0) {
    unsolvable = true;
    return;
  }

  auto a = g * d * d / (2 * v0 * v0);
  auto b = -d;
  auto c = a + h;
  auto delta = b * b - 4 * a * c;

  if (delta < 0) {
    unsolvable = true;
    return;
  }

  unsolvable = false;
  auto tan_pitch_1 = (-b + std::sqrt(delta)) / (2 * a);
  auto tan_pitch_2 = (-b - std::sqrt(delta)) / (2 * a);
  auto pitch_1 = std::atan(tan_pitch_1);
  auto pitch_2 = std::atan(tan_pitch_2);
  auto t_1 = d / (v0 * std::cos(pitch_1));
  auto t_2 = d / (v0 * std::cos(pitch_2));

  pitch = (t_1 < t_2) ? pitch_1 : pitch_2;
  fly_time = (t_1 < t_2) ? t_1 : t_2;
}

}  // namespace tools
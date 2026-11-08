#include "extended_kalman_filter.hpp"

#include <cmath>
#include <numeric>

namespace tools
{
namespace
{
// 卡方分布的 95% 分位数（Wilson-Hilferty 近似，误差 < 0.5%）
// NIS 的自由度 = 观测维数，NEES 的自由度 = 状态维数
// 参考值：dof=4 -> 9.49，dof=11 -> 19.68
double chi2_95(int dof)
{
  const double k = static_cast<double>(dof);
  const double a = 2.0 / (9.0 * k);
  return k * std::pow(1.0 - a + 1.6448536 * std::sqrt(a), 3.0);
}
}  // namespace
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // 卡方检验阈值：取 95% 置信水平，自由度由维度自动确定
  // 原值 0.711 是 χ²(4) 的 5% 分位（查表时把下分位当成了上分位），
  // 会导致 95% 的正常帧都被判为失败，监控数据没有意义
  const double nis_threshold = chi2_95(static_cast<int>(z.rows()));
  const double nees_threshold = chi2_95(static_cast<int>(x.rows()));

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools
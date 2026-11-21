#include <fmt/core.h>

#include <chrono>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/aimer.hpp"
#include "tasks/auto_aim/multithread/commandgener.hpp"
#include "tasks/auto_aim/multithread/mt_detector.hpp"
#include "tasks/auto_aim/shooter.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/plotter.hpp"
#include "tools/recorder.hpp"

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/sentry.yaml | 位置参数，yaml配置文件路径 }";

using namespace std::chrono;

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  tools::Exiter exiter;
  tools::Plotter plotter;
  tools::Recorder recorder(100);  //根据实际帧率调整

  io::CBoard cboard(config_path);
  io::Camera camera(config_path);

  auto_aim::multithread::MultiThreadDetector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::Tracker tracker(config_path, solver);
  auto_aim::Aimer aimer(config_path);
  auto_aim::Shooter shooter(config_path);
  auto_aim::multithread::CommandGener commandgener(shooter, aimer, cboard, plotter, true);

  auto detect_thread = std::thread([&]() {
    cv::Mat img;
    std::chrono::steady_clock::time_point t;

    while (!exiter.exit()) {
      camera.read(img, t);
      detector.push(img, t);
    }
  });

  auto mode = io::Mode::idle;
  auto last_mode = io::Mode::idle;

  while (!exiter.exit()) {
    auto t0 = std::chrono::steady_clock::now();
    /// 自瞄核心逻辑
    auto [img, armors, t] = detector.debug_pop();
    Eigen::Quaterniond q = cboard.imu_at(t - 1ms);
    mode = cboard.mode;

    if (last_mode != mode) {
      tools::logger()->info("Switch to {}", io::MODES[mode]);
      last_mode = mode;
    }

    solver.set_R_gimbal2world(q);

    Eigen::Vector3d ypr = tools::eulers(solver.R_gimbal2world(), 2, 1, 0);

    auto targets = tracker.track(armors, t);

    commandgener.push(targets, t, cboard.bullet_speed, ypr);  // 发送给决策线程

    /// debug
    tools::draw_text(img, fmt::format("[{}]", tracker.state()), {10, 30}, {255, 255, 255});

    nlohmann::json data;
    data["src"] = 1;  // 1 = 主循环（本处），2 = CommandGener 线程
    data["t"] = tools::delta_time(std::chrono::steady_clock::now(), t0);

    // 装甲板原始观测数据
    data["armor_num"] = armors.size();

    // tracker 状态机状态，用于量化"丢目标时间占比"
    // 0=lost 1=detecting 2=temp_lost 3=tracking
    // 均值接近 3 说明一直有目标；均值接近 0 说明长期丢目标
    {
      const auto st = tracker.state();
      data["state"] = st == "tracking"     ? 3
                      : st == "temp_lost"  ? 2
                      : st == "detecting"  ? 1
                                           : 0;
    }
    if (!armors.empty()) {
      auto min_x = 1e10;
      auto & armor = armors.front();
      for (auto & a : armors) {
        if (a.center.x < min_x) {
          min_x = a.center.x;
          armor = a;
        }
      }  //always left
      solver.solve(armor);
      data["armor_x"] = armor.xyz_in_world[0];
      data["armor_y"] = armor.xyz_in_world[1];
      data["armor_yaw"] = armor.ypr_in_world[0] * 57.3;
      data["armor_yaw_raw"] = armor.yaw_raw * 57.3;
      // 分类结果，用于排查"name/type 帧间跳变导致 tracker 匹配失败 → 掉 lost"
      data["armor_name"] = static_cast<int>(armor.name);
      data["armor_type"] = static_cast<int>(armor.type);
    }

    if (!targets.empty()) {
      auto target = targets.front();

      // 当前帧target更新后
      // 诊断用：把 4 个预测槽位的 id 标出来，并高亮本帧实际匹配上的那个
      //   黄色框 = last_id，这一块是靠观测直接定出来的，它准不准反映标定质量
      //   绿色框 = 其他槽位，是模型按 r / 角度间隔推出来的，它准不准反映结构参数
      // 这样一眼就能分清"标定问题"和"模型参数问题"
      std::vector<Eigen::Vector4d> armor_xyza_list = target.armor_xyza_list();
      const int matched_id = target.last_id;
      for (size_t i = 0; i < armor_xyza_list.size(); i++) {
        const Eigen::Vector4d & xyza = armor_xyza_list[i];
        auto image_points =
          solver.reproject_armor(xyza.head(3), xyza[3], target.armor_type, target.name);
        const bool is_matched = (static_cast<int>(i) == matched_id);
        const cv::Scalar color = is_matched ? cv::Scalar(0, 255, 255) : cv::Scalar(0, 255, 0);
        tools::draw_points(img, image_points, color);

        // 编号标在第一个角点（左上）上方
        if (!image_points.empty()) {
          cv::Point label_pos(
            static_cast<int>(image_points[0].x), static_cast<int>(image_points[0].y) - 6);
          tools::draw_text(img, std::to_string(i), label_pos, color, 0.6, 2);
        }
      }

      // aimer瞄准位置
      auto aim_point = aimer.debug_aim_point;
      Eigen::Vector4d aim_xyza = aim_point.xyza;
      auto image_points =
        solver.reproject_armor(aim_xyza.head(3), aim_xyza[3], target.armor_type, target.name);
      if (aim_point.valid)
        tools::draw_points(img, image_points, {0, 0, 255});
      else
        tools::draw_points(img, image_points, {255, 0, 0});

      // 观测器内部数据
      Eigen::VectorXd x = target.ekf_x();
      data["x"] = x[0];
      data["vx"] = x[1];
      data["y"] = x[2];
      data["vy"] = x[3];
      data["z"] = x[4];
      data["vz"] = x[5];
      data["a"] = x[6] * 57.3;
      data["w"] = x[7];
      data["r"] = x[8];
      data["l"] = x[9];
      data["h"] = x[10];
      data["last_id"] = target.last_id;

      // 卡方检验数据
      data["residual_yaw"] = target.ekf().data.at("residual_yaw");
      data["residual_pitch"] = target.ekf().data.at("residual_pitch");
      data["residual_distance"] = target.ekf().data.at("residual_distance");
      data["residual_angle"] = target.ekf().data.at("residual_angle");
      data["nis"] = target.ekf().data.at("nis");
      data["nees"] = target.ekf().data.at("nees");
      data["nis_fail"] = target.ekf().data.at("nis_fail");
      data["nees_fail"] = target.ekf().data.at("nees_fail");
      data["recent_nis_failures"] = target.ekf().data.at("recent_nis_failures");

      // 原始观测值，用于离线标定 R
      // 建目标的当帧还没有产生观测，count() 为 0 时跳过
      if (target.ekf().data.count("z_yaw")) {
        data["z_yaw"] = target.ekf().data.at("z_yaw");
        data["z_pitch"] = target.ekf().data.at("z_pitch");
        data["z_dist"] = target.ekf().data.at("z_dist");
        data["z_angle"] = target.ekf().data.at("z_angle");
      }

      // 实际预测步长，用于核对 Q 的标定条件（Q 与 dt 强耦合）
      if (target.ekf().data.count("dt")) data["dt"] = target.ekf().data.at("dt");
    }

    // 云台响应情况
    data["gimbal_yaw"] = ypr[0] * 57.3;
    data["gimbal_pitch"] = ypr[1] * 57.3;
    data["bullet_speed"] = cboard.bullet_speed;

    plotter.plot(data);

    cv::resize(img, img, {}, 0.5, 0.5);  // 显示时缩小图片尺寸
    cv::imshow("reprojection", img);
    auto key = cv::waitKey(1);
    if (key == 'q') break;
  }

  detect_thread.join();

  return 0;
}
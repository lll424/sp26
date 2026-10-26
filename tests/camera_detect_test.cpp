#include <fmt/core.h>

#include <chrono>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "tasks/auto_aim/detector.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "serial/serial.h"

#pragma pack(push, 1)
struct VisionToChassisPacket {
  uint8_t header1 = 0x5A; // 帧头1
  uint8_t header2 = 0xA5; // 帧头2
  uint8_t target_found = 0; // 1: 找到装甲板, 0: 没找到
  float x = 0.0f;        // 左右偏差 X (单位: 米)
  float z = 0.0f;        // 正前方距离 Z (单位: 米)
  float yaw = 0.0f;      // 偏航角 Yaw (单位: 度)
  uint8_t checksum = 0;  // 异或校验和
};
#pragma pack(pop)

static uint8_t calc_checksum(const uint8_t * data, size_t len) {
  uint8_t sum = 0;
  for (size_t i = 0; i < len; ++i) {
    sum ^= data[i];
  }
  return sum;
}

const std::string keys =
  "{help h usage ? |                            | 输出命令行参数说明 }"
  "{@config-path   | configs/mindvision_vm.yaml | yaml配置文件的路径}"
  "{port p         | /dev/ttyACM0               | STM32虚拟串口设备路径}"
  "{tradition t    | true                       | 是否使用传统方法识别}";

int main(int argc, char * argv[])
{
  // 读取命令行参数
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  auto config_path = cli.get<std::string>(0);
  auto port_name = cli.get<std::string>("port");
  auto use_tradition = cli.get<bool>("tradition");

  tools::Exiter exiter;

  // 初始化 USB 虚拟串口连接 STM32 CDC
  serial::Serial serial_port;
  bool serial_opened = false;
  try {
    serial_port.setPort(port_name);
    serial_port.setBaudrate(115200);
    serial_port.open();
    serial_opened = true;
    tools::logger()->info("[USB CDC Serial] Successfully opened port: {}", port_name);
  } catch (const std::exception & e) {
    tools::logger()->warn("[USB CDC Serial] Failed to open {} ({}). Transmission disabled.", port_name, e.what());
  }

  io::Camera camera(config_path);
  auto_aim::Detector detector(config_path, true);
  auto_aim::Solver solver(config_path);
  auto_aim::YOLO yolo(config_path, true);

  std::chrono::steady_clock::time_point timestamp;

  while (!exiter.exit()) {
    cv::Mat img;
    std::list<auto_aim::Armor> armors;

    camera.read(img, timestamp);

    if (img.empty()) break;

    auto last = std::chrono::steady_clock::now();

    if (use_tradition)
      armors = detector.detect(img);
    else
      armors = yolo.detect(img);

    VisionToChassisPacket packet;
    cv::Mat display_img = img.clone();

    if (!armors.empty()) {
      auto & armor = armors.front();
      solver.solve(armor);
      auto xyz = armor.xyz_in_world;
      double dist = std::sqrt(xyz[0] * xyz[0] + xyz[1] * xyz[1] + xyz[2] * xyz[2]);

      // 充填通信数据包
      packet.target_found = 1;
      packet.x = static_cast<float>(xyz[0]);
      packet.z = static_cast<float>(xyz[2]);
      packet.yaw = static_cast<float>(armor.ypr_in_world[0] * 57.3);
      packet.checksum = calc_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 1);

      tools::logger()->info(
        "[Camera PnP -> STM32] Name: {}, Type: {}, X:{:.2f}m, Z:{:.2f}m, Yaw:{:.1f}deg",
        auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type], packet.x, packet.z, packet.yaw);

      auto info = fmt::format(
        "{} ({}) Dist:{:.2f}m (X:{:.2f} Z:{:.2f})",
        auto_aim::ARMOR_NAMES[armor.name], auto_aim::ARMOR_TYPES[armor.type], dist, packet.x, packet.z);
      tools::draw_points(display_img, armor.points, {0, 255, 0});
      tools::draw_text(display_img, info, armor.left.bottom, {0, 255, 0}, 0.6, 2);
    } else {
      packet.target_found = 0;
      packet.x = 0.0f;
      packet.z = 0.0f;
      packet.yaw = 0.0f;
      packet.checksum = calc_checksum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet) - 1);
    }

    // 通过 USB CDC 发送给 STM32F427
    if (serial_opened && serial_port.isOpen()) {
      try {
        serial_port.write(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
      } catch (const std::exception & e) {
        tools::logger()->warn("[USB CDC Serial] Write failed: {}", e.what());
      }
    }

    cv::resize(display_img, display_img, {}, 0.5, 0.5);
    cv::imshow("PnP 3D Pose Result", display_img);

    auto key = cv::waitKey(33);
    if (key == 'q') break;
  }

  if (serial_opened && serial_port.isOpen()) {
    serial_port.close();
  }

  return 0;
}
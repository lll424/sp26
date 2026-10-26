#include <fmt/core.h>

#include <chrono>
#include <filesystem>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <string>

#include "io/camera.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

namespace fs = std::filesystem;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/standard3.yaml | yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }

  auto config_path = cli.get<std::string>("@config-path");

  std::cout << "============================================================\n";
  std::cout << "      装甲板 ROI 自动拍照与数据集生成工具 (支持 MindVision)\n";
  std::cout << "============================================================\n";

  // 1. 选择数据集分割 (train / val)
  std::string split = "train";
  std::cout << "\n[1/2] 请选择保存到训练集还是验证集? (1: train / 2: val) [默认: 1]: ";
  std::string input_split;
  std::getline(std::cin, input_split);
  if (input_split == "2") {
    split = "val";
  }

  // 2. 选择类别 (class0 / class1)
  std::string class_name = "class0";
  std::cout << "\n[2/2] 请选择拍摄的类别? (0: class0 / 1: class1) [默认: 0]: ";
  std::string input_class;
  std::getline(std::cin, input_class);
  if (input_class == "1") {
    class_name = "class1";
  }

  // 目标保存路径
  std::string save_dir = fmt::format("onnx/dataset/{}/{}", split, class_name);
  fs::create_directories(save_dir);

  std::cout << "\n------------------------------------------------------------\n";
  std::cout << fmt::format("✅ 配置完成！照片将自动切取 32x32 灰度 ROI 保存至:\n   👉 {}\n", save_dir);
  std::cout << "------------------------------------------------------------\n";

  std::cout << "正在初始化相机 (使用配置文件: " << config_path << ")...\n";
  
  tools::Exiter exiter;
  io::Camera camera(config_path);

  cv::Mat img;
  std::chrono::steady_clock::time_point timestamp;
  int saved_count = 0;
  std::string window_name = "MindVision ROI Capture Tool (Space/S: Save | Q/ESC: Quit)";

  std::cout << "\n操作说明:\n";
  std::cout << "  - 【空格键 Space】 或 【S 键】：裁切中心 32x32 灰度 ROI 并保存\n";
  std::cout << "  - 【Q 键】 或 【ESC 键】：退出程序\n\n";

  while (!exiter.exit()) {
    camera.read(img, timestamp);

    if (img.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }

    int h = img.rows;
    int w = img.cols;

    // 中心 ROI 矩形大小 (如 120x120 像素)
    int roi_size = 120;
    int cx = w / 2;
    int cy = h / 2;
    int x1 = std::max(cx - roi_size / 2, 0);
    int y1 = std::max(cy - roi_size / 2, 0);
    int x2 = std::min(cx + roi_size / 2, w);
    int y2 = std::min(cy + roi_size / 2, h);

    cv::Rect roi_rect(x1, y1, x2 - x1, y2 - y1);
    cv::Mat roi_bgr = img(roi_rect);

    cv::Mat display_frame = img.clone();

    // 绘制中心绿色框与红点
    cv::rectangle(display_frame, roi_rect, cv::Scalar(0, 255, 0), 2);
    cv::circle(display_frame, cv::Point(cx, cy), 3, cv::Scalar(0, 0, 255), -1);

    // 绘制状态文字提示
    std::string info_text = fmt::format("Saving to: {}/{} | Saved: {}", split, class_name, saved_count);
    tools::draw_text(display_frame, info_text, {10, 30}, {0, 255, 255});
    tools::draw_text(display_frame, "Press SPACE to Save ROI | Press Q to Exit", {10, h - 20}, {255, 255, 255});

    // 右下角显示实时 32x32 灰度 ROI 预览
    if (roi_bgr.rows > 0 && roi_bgr.cols > 0) {
      cv::Mat gray_roi, resized_gray, preview_bgr;
      cv::cvtColor(roi_bgr, gray_roi, cv::COLOR_BGR2GRAY);
      cv::resize(gray_roi, resized_gray, cv::Size(64, 64)); // 放大预览
      cv::cvtColor(resized_gray, preview_bgr, cv::COLOR_GRAY2BGR);

      cv::Rect preview_rect(w - 75, h - 75, 64, 64);
      preview_bgr.copyTo(display_frame(preview_rect));
      cv::rectangle(display_frame, preview_rect, cv::Scalar(255, 255, 0), 1);
    }

    cv::imshow(window_name, display_frame);
    int key = cv::waitKey(1);

    // 空格键 (32) 或 's'/'S'
    if (key == 32 || key == 's' || key == 'S') {
      if (roi_bgr.rows > 0 && roi_bgr.cols > 0) {
        cv::Mat final_gray_roi;
        cv::cvtColor(roi_bgr, final_gray_roi, cv::COLOR_BGR2GRAY);
        cv::resize(final_gray_roi, final_gray_roi, cv::Size(32, 32));

        auto now = std::chrono::system_clock::now();
        auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count() % 1000;
        std::string filename = fmt::format("{}/{}_{:%Y%m%d_%H%M%S}_{:03d}.jpg", save_dir, class_name, now, ms);

        cv::imwrite(filename, final_gray_roi);
        saved_count++;
        tools::logger()->info("📸 [{}] Saved ROI to {}", saved_count, filename);
      }
    } else if (key == 'q' || key == 'Q' || key == 27) {
      std::cout << "\n👋 退出拍照程序。\n";
      break;
    }
  }

  cv::destroyAllWindows();
  std::cout << fmt::format("\n🎉 采集完成！本次共保存了 {} 张 {} ({}) 图片。\n", saved_count, class_name, split);

  return 0;
}

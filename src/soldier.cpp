#include <fmt/core.h>
#include <yaml-cpp/yaml.h>

#include <chrono>
#include <limits>
#include <opencv2/opencv.hpp>

#include "io/camera.hpp"
#include "io/cboard.hpp"
#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "tools/exiter.hpp"
#include "tools/img_tools.hpp"
#include "tools/logger.hpp"

using namespace std::chrono_literals;

const std::string keys =
  "{help h usage ? |                        | 输出命令行参数说明}"
  "{@config-path   | configs/soldier.yaml | 位置参数，yaml配置文件路径 }";

int main(int argc, char * argv[])
{
  cv::CommandLineParser cli(argc, argv, keys);
  auto config_path = cli.get<std::string>(0);
  if (cli.has("help") || config_path.empty()) {
    cli.printMessage();
    return 0;
  }

  auto enemy_color =
    (YAML::LoadFile(config_path)["enemy_color"].as<std::string>() == "red") ? auto_aim::red
                                                                           : auto_aim::blue;

  tools::Exiter exiter;

  io::Camera camera(config_path);
  io::CBoard cboard(config_path);

  auto_aim::YOLO detector(config_path, false);
  auto_aim::Solver solver(config_path);

  cv::Mat img;
  Eigen::Quaterniond q;
  std::chrono::steady_clock::time_point t;

  auto last_log_time = std::chrono::steady_clock::now();

  while (!exiter.exit()) {
    camera.read(img, t);
    q = cboard.imu_at(t - 1ms);

    if (img.empty()) continue;

    // YOLO 检测不区分敌我颜色，此处按配置过滤
    auto armors = detector.detect(img);
    armors.remove_if([&enemy_color](const auto_aim::Armor & armor) {
      return armor.color != enemy_color;
    });

    // 更新云台到世界坐标系的旋转，PnP 结果据此转到世界坐标系
    solver.set_R_gimbal2world(q);

    const cv::Point2f img_center(img.cols / 2.0f, img.rows / 2.0f);
    auto target = armors.end();
    auto min_offset = std::numeric_limits<double>::max();

    for (auto it = armors.begin(); it != armors.end(); ++it) {
      // PnP 解算：由装甲板四角点和真实尺寸得到位置与姿态
      solver.solve(*it);

      tools::draw_points(img, it->points, {0, 255, 0});

      auto offset = cv::norm(it->center - img_center);
      if (offset < min_offset) {
        min_offset = offset;
        target = it;
      }
    }

    if (target != armors.end()) {
      const Eigen::Vector3d & xyz_in_gimbal = target->xyz_in_gimbal;
      const Eigen::Vector3d & xyz_in_world = target->xyz_in_world;
      auto distance = xyz_in_gimbal.norm();

      tools::draw_point(img, target->center, {0, 0, 255}, 6);
      tools::draw_text(
        img,
        fmt::format(
          "{} {} {:.2f}m", auto_aim::ARMOR_NAMES[target->name], auto_aim::ARMOR_TYPES[target->type],
          distance),
        cv::Point(target->box.x, target->box.y - 5), {0, 0, 255}, 0.6);

      auto text_gimbal = fmt::format(
        "gimbal  x {:.3f}  y {:.3f}  z {:.3f}  m", xyz_in_gimbal[0], xyz_in_gimbal[1],
        xyz_in_gimbal[2]);
      auto text_world = fmt::format(
        "world   x {:.3f}  y {:.3f}  z {:.3f}  m  yaw {:.1f} deg", xyz_in_world[0],
        xyz_in_world[1], xyz_in_world[2], target->ypr_in_world[0] * 57.3);

      tools::draw_text(img, text_gimbal, {10, 30}, {0, 255, 255}, 0.7);
      tools::draw_text(img, text_world, {10, 60}, {0, 255, 255}, 0.7);

      if (t - last_log_time > 500ms) {
        last_log_time = t;
        tools::logger()->info(
          "[{}] {:.2f} m | {} | {}", auto_aim::ARMOR_NAMES[target->name], distance, text_gimbal,
          text_world);
      }
    }

    cv::resize(img, img, {}, 0.5, 0.5);
    cv::imshow("soldier", img);
    if (cv::waitKey(1) == 'q') break;
  }

  return 0;
}

#ifndef AUTO_AIM__YOLO_HPP
#define AUTO_AIM__YOLO_HPP

#include <opencv2/opencv.hpp>

#include <mutex>

#include "armor.hpp"

namespace auto_aim
{
class YOLOBase
{
public:
  virtual std::list<Armor> detect(const cv::Mat & img, int frame_count) = 0;

  virtual std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count) = 0;
};

class YOLO
{
public:
  YOLO(const std::string & config_path, bool debug = true);

  std::list<Armor> detect(const cv::Mat & img, int frame_count = -1);

  std::list<Armor> postprocess(
    double scale, cv::Mat & output, const cv::Mat & bgr_img, int frame_count);

private:
  std::unique_ptr<YOLOBase> yolo_;
  // 同一实例会被主相机与 N 路全向相机共享（见 omniperception::Perceptron），
  // 而底层 OpenVINO 的 InferRequest 不允许并发调用，所以 detect() 默认加锁串行化。
  // 单相机场景可用配置 yolo_lock: false 关掉锁，省掉一次加解锁。
  //
  // 注意：这里必须用 unique_ptr 而不是直接放 std::mutex ——
  // std::mutex 不可拷贝也不可移动，直接作为成员会让 YOLO 变成不可移动类型，
  // 而 tools/thread_pool.hpp 里的 std::vector<auto_aim::YOLO> 需要移动构造。
  std::unique_ptr<std::mutex> mutex_ = std::make_unique<std::mutex>();
  bool lock_enabled_ = true;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__YOLO_HPP
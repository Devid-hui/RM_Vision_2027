#ifndef IO__SIM__CAMERA_SOURCE_HPP
#define IO__SIM__CAMERA_SOURCE_HPP

// 相机数据源抽象：业务算法只依赖此接口。
//   RealHikCameraSource  — 真实相机（海康/迈德威视/USB，由 io::Camera 工厂决定）
//   FakeCameraSource     — 伪帧：合成图案 / 图片序列 / 视频，可注入丢帧、重复、黑屏、过曝、噪声、延迟
//   ReplayCameraSource   — 按录制时间戳回放相机帧
// 伪帧的分辨率、像素格式、帧率、时间戳、帧号与真实相机保持一致。

#include <opencv2/opencv.hpp>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include "io/camera.hpp"
#include "io/sim/common.hpp"
#include "io/sim/recorder.hpp"
#include "tools/thread_safe_queue.hpp"

namespace io::sim
{

struct Frame
{
  TimePoint timestamp{};
  uint64_t frame_id = 0;
  int width = 0;
  int height = 0;
  int format = CV_8UC3;  // cv::Mat::type()
  cv::Mat data;
  SourceMode mode = SourceMode::Fake;
  bool injected = false;  // 是否来自注入的异常帧
};

class CameraSource
{
public:
  using Ptr = std::unique_ptr<CameraSource>;
  virtual ~CameraSource() = default;

  virtual void open() = 0;
  // timeout_ms <= 0 表示阻塞直到有帧
  virtual bool read(Frame & out, int timeout_ms = 0) = 0;
  virtual void close() = 0;
  virtual std::string name() const = 0;
  virtual SourceMode mode() const = 0;
  virtual std::string device_name() const = 0;
};

// ============================ 真实相机 ============================
class RealHikCameraSource : public CameraSource
{
public:
  RealHikCameraSource(std::string config_path, std::string sursign);

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "RealCamera"; }
  SourceMode mode() const override { return SourceMode::Real; }
  std::string device_name() const override;

private:
  std::string config_path_;
  std::string sursign_;
  std::unique_ptr<io::Camera> camera_;
  uint64_t frame_id_ = 0;
};

// ============================ 伪相机 ============================
class FakeCameraSource : public CameraSource
{
public:
  FakeCameraSource(const CameraSourceConfig & cfg, const FaultConfig & fault, int seed);
  ~FakeCameraSource() override;

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "FakeCamera"; }
  SourceMode mode() const override { return SourceMode::Fake; }
  std::string device_name() const override { return device_name_; }

private:
  void run();
  cv::Mat next_image();
  cv::Mat make_synth_frame();
  void emit(const cv::Mat & img, bool injected);

  CameraSourceConfig cfg_;
  FaultConfig fault_;
  FaultInjector injector_;
  std::string device_name_ = "fake";

  // 图片序列 / 视频
  std::vector<std::string> files_;
  size_t file_index_ = 0;
  cv::VideoCapture video_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  tools::ThreadSafeQueue<Frame, true> queue_{1};

  uint64_t frame_id_ = 0;
  long long emitted_ = 0;
  long long dropped_ = 0;
  long long duplicated_ = 0;
  long long injected_count_ = 0;
};

// ============================ 回放相机 ============================
class ReplayCameraSource : public CameraSource
{
public:
  ReplayCameraSource(const CameraSourceConfig & cfg, const FaultConfig & fault, int seed);
  ~ReplayCameraSource() override;

  void open() override;
  bool read(Frame & out, int timeout_ms) override;
  void close() override;
  std::string name() const override { return "ReplayCamera"; }
  SourceMode mode() const override { return SourceMode::Replay; }
  std::string device_name() const override { return device_name_; }

private:
  void run();

  CameraSourceConfig cfg_;
  bool loop_ = true;
  std::string device_name_ = "replay";
  CameraReplayReader reader_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  tools::ThreadSafeQueue<Frame, true> queue_{1};
  bool ready_ = false;
  long long emitted_ = 0;
};

// ============================ CameraBase 适配器 ============================
// 让 Fake/Replay 相机可以直接被现有 io::Camera 工厂返回，业务代码无需改动。
class CameraSourceAdapter : public io::CameraBase
{
public:
  CameraSourceAdapter(CameraSource::Ptr source, std::string device_name);
  ~CameraSourceAdapter() override;

  void read(cv::Mat & img, std::chrono::steady_clock::time_point & timestamp) override;
  std::string device_name() const override;

private:
  CameraSource::Ptr source_;
};

// 按配置创建相机数据源。use_force=true 时忽略配置中的 camera_mode，强制使用 force。
CameraSource::Ptr make_camera_source(
  const YAML::Node & yaml, const std::string & config_path, const std::string & sursign = "",
  SourceMode force = SourceMode::Fake, bool use_force = false);

}  // namespace io::sim

#endif  // IO__SIM__CAMERA_SOURCE_HPP

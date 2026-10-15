#include "io/sim/camera_source.hpp"

#include <algorithm>
#include <cmath>
#include <fmt/format.h>
#include <stdexcept>

#include "tools/logger.hpp"
#include "tools/yaml.hpp"

namespace io::sim
{

namespace
{
std::chrono::steady_clock::duration capped(const std::chrono::steady_clock::duration & remain)
{
  const auto cap =
    std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::milliseconds(2));
  return remain < cap ? remain : cap;
}

bool is_image_file(const std::string & path)
{
  const auto dot = path.find_last_of('.');
  if (dot == std::string::npos) return false;
  std::string ext = path.substr(dot);
  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  return ext == ".png" || ext == ".jpg" || ext == ".jpeg" || ext == ".bmp";
}

std::string read_camera_name(const std::string & config_path, const std::string & sursign)
{
  try {
    auto yaml = tools::load(config_path);
    const std::string key = sursign.empty() ? "camera_name" : "camera_name_" + sursign;
    return node_as<std::string>(yaml, key, "");
  } catch (...) {
    return "";
  }
}
}  // namespace

// ============================ RealHikCameraSource ============================
RealHikCameraSource::RealHikCameraSource(std::string config_path, std::string sursign)
: config_path_(std::move(config_path)), sursign_(std::move(sursign))
{
}

void RealHikCameraSource::open()
{
  const std::string name = read_camera_name(config_path_, sursign_);
  if (name == "fake" || name == "replay") {
    throw std::runtime_error(
      "[RealCamera] camera_name='" + name +
      "' 会导致递归；真实相机请使用 hikrobot/mindvision/Usbcamera，"
      "伪相机请通过 io::Camera 工厂的 fake/replay 分支创建。");
  }

  camera_ = std::make_unique<io::Camera>(config_path_, sursign_);
  tools::logger()->info(
    "[REAL] Camera '{}' opened (camera_name={})", camera_->device_name(), name);
}

bool RealHikCameraSource::read(Frame & out, int /*timeout_ms*/)
{
  if (!camera_) return false;

  out.data = cv::Mat();
  camera_->read(out.data, out.timestamp);
  if (out.data.empty()) return false;

  out.frame_id = frame_id_++;
  out.width = out.data.cols;
  out.height = out.data.rows;
  out.format = out.data.type();
  out.mode = SourceMode::Real;
  out.injected = false;
  return true;
}

void RealHikCameraSource::close()
{
  camera_.reset();
  tools::logger()->info("[REAL] Camera closed");
}

std::string RealHikCameraSource::device_name() const
{
  return camera_ ? camera_->device_name() : std::string("real");
}

// ============================ FakeCameraSource ============================
FakeCameraSource::FakeCameraSource(const CameraSourceConfig & cfg, const FaultConfig & fault, int seed)
: cfg_(cfg), fault_(fault), injector_(fault, seed)
{
  if (cfg_.width < 2) cfg_.width = 1440;
  if (cfg_.height < 2) cfg_.height = 1080;
  if (cfg_.fps <= 0.0) cfg_.fps = 30.0;
  device_name_ = "fake_" + cfg_.kind;
  cv::theRNG().state = static_cast<uint64_t>(seed);
}

FakeCameraSource::~FakeCameraSource() { close(); }

void FakeCameraSource::open()
{
  if (cfg_.kind == "sequence") {
    std::vector<cv::String> found;
    cv::glob(cfg_.dir + "/*", found, false);
    for (const auto & f : found)
      if (is_image_file(f)) files_.push_back(f);
    std::sort(files_.begin(), files_.end());
    if (files_.empty()) {
      tools::logger()->warn("[FAKE] No images in '{}', fallback to synth pattern", cfg_.dir);
      cfg_.kind = "synth";
    } else {
      tools::logger()->info("[FAKE] FakeCamera sequence source: {} images from '{}'", files_.size(), cfg_.dir);
    }
  } else if (cfg_.kind == "video") {
    video_.open(cfg_.video);
    if (!video_.isOpened()) {
      tools::logger()->warn("[FAKE] Cannot open video '{}', fallback to synth pattern", cfg_.video);
      cfg_.kind = "synth";
    } else {
      tools::logger()->info("[FAKE] FakeCamera video source: '{}'", cfg_.video);
    }
  }

  quit_ = false;
  thread_ = std::thread([this] { run(); });
}

cv::Mat FakeCameraSource::make_synth_frame()
{
  const int w = 640, h = 480;
  cv::Mat base(h, w, CV_8UC3);
  for (int y = 0; y < h; ++y) base.row(y).setTo(cv::Scalar(40 + y / 6, 60, 90 - y / 8));

  const int cell = 40;
  for (int y = 0; y < h; y += cell)
    for (int x = 0; x < w; x += cell)
      if (((x / cell) + (y / cell)) % 2 == 0)
        cv::rectangle(base, {x, y}, {x + cell, y + cell}, cv::Scalar(200, 200, 200), -1);

  const double t =
    std::chrono::duration<double>(SteadyClock::now() - SessionClock::start_time()).count();
  const int cx = static_cast<int>(w * 0.5 + w * 0.35 * std::sin(2.0 * M_PI * 0.2 * t));
  const int cy = static_cast<int>(h * 0.5 + h * 0.30 * std::cos(2.0 * M_PI * 0.2 * t));
  cv::rectangle(base, {cx - 30, cy - 20}, {cx + 30, cy + 20}, cv::Scalar(0, 0, 255), -1);

  cv::putText(
    base, fmt::format("FAKE f={} t={:.2f}s", frame_id_, t), {10, 24}, cv::FONT_HERSHEY_SIMPLEX, 0.6,
    cv::Scalar(0, 255, 255), 1);

  cv::Mat out;
  cv::resize(base, out, {cfg_.width, cfg_.height}, 0, 0, cv::INTER_LINEAR);
  return out;
}

cv::Mat FakeCameraSource::next_image()
{
  if (cfg_.kind == "sequence" && !files_.empty()) {
    cv::Mat img = cv::imread(files_[file_index_ % files_.size()], cv::IMREAD_COLOR);
    file_index_++;
    if (!img.empty()) return img;
    return make_synth_frame();
  }
  if (cfg_.kind == "video" && video_.isOpened()) {
    cv::Mat img;
    video_ >> img;
    if (img.empty()) {
      video_.set(cv::CAP_PROP_POS_FRAMES, 0);
      video_ >> img;
    }
    if (!img.empty()) return img;
    return make_synth_frame();
  }
  return make_synth_frame();
}

void FakeCameraSource::emit(const cv::Mat & img, bool injected)
{
  if (img.empty()) return;

  Frame f;
  f.timestamp = SteadyClock::now();
  f.frame_id = frame_id_++;
  f.width = img.cols;
  f.height = img.rows;
  f.format = img.type();
  f.data = img;
  f.mode = SourceMode::Fake;
  f.injected = injected;

  queue_.push(f);  // 队列满时丢弃旧帧，保留最新帧（与真实相机一致）
  emitted_++;
  if (injected) injected_count_++;
}

void FakeCameraSource::run()
{
  const double period = 1.0 / cfg_.fps;
  const auto t0 = SteadyClock::now();
  long long tick = 0;
  cv::Mat last;
  auto last_log = t0;

  tools::logger()->info(
    "[FAKE] FakeCamera started: {}x{} @{}fps kind={} faults={}", cfg_.width, cfg_.height, cfg_.fps,
    cfg_.kind, fault_.enabled() ? "on" : "off");

  while (!quit_) {
    const auto target =
      t0 + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
             std::chrono::duration<double>(static_cast<double>(tick) * period));
    tick++;

    while (!quit_) {
      const auto now = SteadyClock::now();
      if (now >= target) break;
      std::this_thread::sleep_for(capped(target - now));
    }
    if (quit_) break;

    if (injector_.chance(fault_.camera_drop_rate)) {
      dropped_++;
      tools::logger()->warn("[FAULT:drop] camera frame dropped (count={})", dropped_);
      continue;
    }

    bool injected = false;
    cv::Mat img;
    if (!last.empty() && injector_.chance(fault_.camera_freeze_rate)) {
      img = last;
      injected = true;
    } else {
      img = next_image();
    }
    if (img.empty()) continue;
    img = img.clone();  // 保证本帧独占缓冲，后续原地修改安全

    if (injector_.chance(fault_.camera_black_rate)) {
      img.setTo(cv::Scalar(0, 0, 0));
      injected = true;
      tools::logger()->warn("[FAULT:black] camera frame blackened");
    }
    if (injector_.chance(fault_.camera_overexpose_rate)) {
      cv::add(img, cv::Scalar(120, 120, 120), img);
      injected = true;
      tools::logger()->warn("[FAULT:expose] camera frame overexposed");
    }
    if (fault_.camera_noise_sigma > 0.0) {
      cv::Mat f, noise;
      img.convertTo(f, CV_32FC3);
      noise.create(f.size(), CV_32FC3);
      cv::randn(noise, 0.0, fault_.camera_noise_sigma);
      f += noise;
      f.convertTo(img, CV_8UC3);
      injected = true;
    }
    if (fault_.camera_delay_ms > 0.0) {
      std::this_thread::sleep_for(std::chrono::duration<double, std::milli>(fault_.camera_delay_ms));
      injected = true;
    }

    emit(img, injected);

    if (injector_.chance(fault_.camera_duplicate_rate)) {
      duplicated_++;
      emit(img, true);
      tools::logger()->warn("[FAULT:dup] camera frame duplicated (count={})", duplicated_);
    }

    last = img;

    if (std::chrono::duration<double>(SteadyClock::now() - last_log).count() >= 2.0) {
      tools::logger()->info(
        "[FAKE] FakeCamera stats: emitted={} dropped={} duplicated={} injected={}", emitted_,
        dropped_, duplicated_, injected_count_);
      last_log = SteadyClock::now();
    }
  }

  tools::logger()->info(
    "[FAKE] FakeCamera stopped: emitted={} dropped={} duplicated={} injected={}", emitted_, dropped_,
    duplicated_, injected_count_);
}

bool FakeCameraSource::read(Frame & out, int timeout_ms)
{
  if (timeout_ms > 0) return queue_.try_pop_for(out, std::chrono::milliseconds(timeout_ms));

  while (!quit_) {
    if (queue_.try_pop_for(out, std::chrono::milliseconds(100))) return true;
  }
  return false;
}

void FakeCameraSource::close()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

// ============================ ReplayCameraSource ============================
ReplayCameraSource::ReplayCameraSource(
  const CameraSourceConfig & cfg, const FaultConfig & /*fault*/, int /*seed*/)
: cfg_(cfg), reader_(cfg.replay_dir)
{
  device_name_ = "replay";
}

ReplayCameraSource::~ReplayCameraSource() { close(); }

void ReplayCameraSource::open()
{
  quit_ = false;
  thread_ = std::thread([this] { run(); });
}

void ReplayCameraSource::run()
{
  std::string error;
  if (!reader_.load(error)) {
    tools::logger()->error("[REPLAY] Camera replay load failed: {}", error);
    return;
  }

  const auto & records = reader_.records();
  if (records.empty()) return;

  ready_ = true;
  const uint64_t base = records.front().t_ns;

  tools::logger()->info(
    "[REPLAY] Camera replay started: dir={} frames={} span={:.2f}s loop={}", cfg_.replay_dir,
    records.size(), reader_.duration_ns() / 1e9, loop_);

  do {
    const auto t0 = SteadyClock::now();
    for (size_t i = 0; i < records.size(); ++i) {
      if (quit_) break;

      const auto target = t0 + std::chrono::nanoseconds(records[i].t_ns - base);
      while (!quit_) {
        const auto now = SteadyClock::now();
        if (now >= target) break;
        std::this_thread::sleep_for(capped(target - now));
      }
      if (quit_) break;

      cv::Mat img = reader_.read_image(i);
      if (img.empty())
        img = cv::Mat(records[i].height, records[i].width, CV_8UC3, cv::Scalar(0, 0, 0));

      Frame f;
      f.timestamp = SteadyClock::now();
      f.frame_id = records[i].frame_id;
      f.width = img.cols;
      f.height = img.rows;
      f.format = img.type();
      f.data = img;
      f.mode = SourceMode::Replay;

      queue_.push(f);
      emitted_++;
    }
  } while (loop_ && !quit_);

  tools::logger()->info("[REPLAY] Camera replay finished: {} frames", emitted_);
}

bool ReplayCameraSource::read(Frame & out, int timeout_ms)
{
  if (timeout_ms > 0) return queue_.try_pop_for(out, std::chrono::milliseconds(timeout_ms));

  while (!quit_) {
    if (queue_.try_pop_for(out, std::chrono::milliseconds(100))) return true;
  }
  return false;
}

void ReplayCameraSource::close()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
}

// ============================ CameraSourceAdapter ============================
CameraSourceAdapter::CameraSourceAdapter(CameraSource::Ptr source, std::string device_name)
: source_(std::move(source))
{
  device_name_ = device_name.empty() ? (source_ ? source_->device_name() : std::string("fake"))
                                     : device_name;
  if (source_) source_->open();
  tools::logger()->info(
    "[{}] CameraSourceAdapter '{}' ready (source={}, camera={})",
    source_ ? log_tag(source_->mode()) : "?", device_name_, source_ ? source_->name() : "none",
    device_name_);
}

CameraSourceAdapter::~CameraSourceAdapter()
{
  if (source_) source_->close();
}

void CameraSourceAdapter::read(
  cv::Mat & img, std::chrono::steady_clock::time_point & timestamp)
{
  if (!source_) {
    img.release();
    timestamp = SteadyClock::now();
    return;
  }

  Frame f;
  if (!source_->read(f, 0)) {
    img.release();
    timestamp = SteadyClock::now();
    return;
  }

  img = f.data;
  timestamp = f.timestamp;

  if (f.injected)
    tools::logger()->warn(
      "[FAULT] frame {} delivered with injected anomaly ({}x{})", f.frame_id, f.width, f.height);
}

std::string CameraSourceAdapter::device_name() const { return device_name_; }

// ============================ 工厂 ============================
CameraSource::Ptr make_camera_source(
  const YAML::Node & yaml, const std::string & config_path, const std::string & sursign,
  SourceMode force, bool use_force)
{
  const SimConfig sim = SimConfig::load(yaml);
  CameraSourceConfig cfg = CameraSourceConfig::load(yaml, sim.mode, sursign);
  if (use_force) cfg.mode = force;

  switch (cfg.mode) {
    case SourceMode::Real:
      return std::make_unique<RealHikCameraSource>(config_path, sursign);
    case SourceMode::Fake:
      return std::make_unique<FakeCameraSource>(cfg, sim.fault, sim.seed);
    case SourceMode::Replay:
      return std::make_unique<ReplayCameraSource>(cfg, sim.fault, sim.seed);
  }
  return std::make_unique<FakeCameraSource>(cfg, sim.fault, sim.seed);
}

}  // namespace io::sim

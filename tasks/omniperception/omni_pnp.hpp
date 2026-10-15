#ifndef OMNIPERCEPTION__OMNI_PNP_HPP
#define OMNIPERCEPTION__OMNI_PNP_HPP

// =============================================================================
//  omni_pnp.hpp —— 把主相机的 PnP 解算搬到全向相机上（全向侧胶水代码）
//
//  分工：
//    · tasks/auto_aim/pnp.*      ：与相机无关的公共内核（模型点 + solvePnP）
//    · tasks/omniperception/omni_pnp.*：本文件，全向相机特有的三件事
//        1) 没有标定内参时，用 FOV + 图像尺寸估一个近似内参（占位方案）
//        2) 没有手眼标定时，用安装角 mount_yaw/mount_pitch 构造 R_camera2gimbal（占位方案）
//        3) 解出「装甲板中心在云台系下的 yaw / pitch / 距离」
//
//  与旧做法的区别：
//    旧代码 Decider::delta_angle 用「像素到角度」的线性插值：
//        yaw = mount_yaw + fov_h/2 - center_norm.x * fov_h
//    这在视场边缘误差很大（真实关系是 arctan，不是线性），
//    现在改为：solvePnP 得到相机系 3D 点 -> 外参转到云台系 -> atan2 求角度。
//
//  【术语】下文所说的「云台系 / gimbal 系」= 本工程的 IMU 参考系 = **大yaw 系**
//         （IMU 与四个全向相机都刚性安装在大yaw上）。详见下面 mount_to_R_camera2gimbal
//         处的「大yaw + 小yaw」说明。
// =============================================================================

#include <Eigen/Dense>  // 必须在 opencv2/core/eigen.hpp 上面
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "detection.hpp"
#include "tasks/auto_aim/armor.hpp"

namespace omniperception
{

// -----------------------------------------------------------------------------
//  从 yaml 读取第 index 个全向相机的 PnP 标定参数（键名形如 omni_camera_matrix_<index>）
//
//  可识别的键（全部可选，缺省时保持 cfg 现有值，由 decider 用近似值兜底）：
//    omni_camera_matrix_N    : 9 个数，3x3 行优先内参
//    omni_distort_coeffs_N   : 5 个数，k1 k2 p1 p2 k3
//    omni_R_camera2gimbal_N  : 9 个数，3x3 行优先，相机 -> 云台 旋转
//    omni_t_camera2gimbal_N  : 3 个数，相机 -> 云台 平移（m）
//
//  R 与 t 必须成对出现才会把 cfg.has_extrinsic 置 true。
//  参数不合法（个数不对）时会打 warn 并保持原值，不会抛异常。
//  这个函数被 sentry_multithread.cpp / sentry_multithread_debug.cpp 共用，
//  避免两个入口的参数解析逻辑各写一遍后逐渐不一致。
// -----------------------------------------------------------------------------
void load_omni_pnp_config(OmniCameraConfig & cfg, const YAML::Node & yaml, int index);

// -----------------------------------------------------------------------------
//  占位方案 1：用 FOV 估计相机内参
//
//  在全向相机还没有做棋盘格标定的时候，用配置里的 fov_h/fov_v 和当前帧尺寸
//  反推出一个「理想针孔相机」内参：
//      fx = (w/2) / tan(fov_h/2),  fy = (h/2) / tan(fov_v/2)
//      cx = w/2,                   cy = h/2
//  它忽略了镜头畸变，对广角镜头只是近似；但方向是 arctan 正确的，
//  比旧的线性插值好得多。拿到标定结果后请在 yaml 里填 omni_camera_matrix_N，
//  代码会自动优先使用标定值（见 decider.cpp）。
//
//  注意：fov 必须是「全画幅」的水平/垂直视场角，单位度。
// -----------------------------------------------------------------------------
cv::Mat approximate_camera_matrix(
  const cv::Size & image_size, double fov_h_deg, double fov_v_deg);

// -----------------------------------------------------------------------------
//  占位方案 2：用安装角构造 R_camera2gimbal
//
//  坐标约定（与主相机 R_camera2gimbal 的约定保持一致，整份代码统一）：
//    · 云台系：x 向前，y 向左，z 向上（与 tools::xyz2ypd / 世界系一致）
//    · 相机系：OpenCV 约定，x 向右，y 向下，z 为光轴向前
//
//  基准姿态（mount_yaw = mount_pitch = 0）：光轴指向云台正前方，于是
//      R0 = [ 0, 0, 1;
//            -1, 0, 0;
//             0,-1, 0 ]
//  即：相机 z(前) -> 云台 x(前)，相机 x(右) -> 云台 -y(右)，相机 y(下) -> 云台 -z(下)
//
//  再叠加安装角：
//      R = Rz(mount_yaw) * Ry(+mount_pitch) * R0
//  其中 mount_yaw 为正表示相机被装成「朝向云台左侧」；
//  mount_pitch 为正表示相机「朝下低头」。后者是为了与配置里原有的语义一致：
//  旧 delta_angle 把 mount_pitch 直接加在「向下为正」的 pitch 命令上
//  （见 main 里的 Aimer：command.pitch = -(世界系抬头为正的 pitch)）。
//  如果你在标定时习惯用「抬头为正」，请在这里取负号。
//
//  ⚠ 关于哨兵的「大yaw + 小yaw」结构（重要，别搞反）：
//    · 本项目里 "gimbal"（云台）系 = **大yaw 系**：IMU 与四个全向相机都刚性
//      安装在 大yaw 上，所以 R_gimbal2world 是「大yaw -> 世界」。
//    · 因此全向相机的外参应标成「相机 -> 大yaw（即 IMU 参考系）」。标定方法
//      可直接复用 calibration/calibrate_robotworld_handeye.cpp：它用 IMU 四元数
//      构造 R_world2gimbal，解出来的正是 相机 -> 大yaw。
//    · 这样 xyz_world = R_gimbal2world * xyz_bigyaw 严格成立，
//      **全向路径完全不需要小yaw相对角 psi**（全向相机与 IMU 同在一根连杆上）。
//    · 真正需要 psi 的是主相机：它装在小yaw上，而它的 R_camera2gimbal 标到了
//      大yaw系。修正接口见 auto_aim::Solver::set_small_yaw_offset()。
//    · 必须遵守的耦合：本模块返回的角度是「大yaw系」。入口里给全向角度叠加的
//      gimbal_pos = eulers(Solver::R_gimbal2world()) 也正是大yaw的世界系 yaw，
//      两者同系，所以 "delta + gimbal_pos" 才成立。若以后把 Solver 的 "gimbal"
//      改成小yaw系，这里必须相应减去 psi，否则全向方向会整体偏一个 psi。
//
//  做完标定后请填 omni_R_camera2gimbal_N / omni_t_camera2gimbal_N
//  （名字沿用工程里 "camera2gimbal" = 相机->大yaw 的叫法）。
// -----------------------------------------------------------------------------
Eigen::Matrix3d mount_to_R_camera2gimbal(double mount_yaw_deg, double mount_pitch_deg);

// 全向相机一次 PnP 解算的结果
struct OmniPnPResult
{
  bool success = false;

  // 装甲板中心在云台系下的坐标（m）。x 前、y 左、z 上。
  Eigen::Vector3d xyz_in_gimbal = Eigen::Vector3d::Zero();

  // 相对云台正前方的角度（度），采用「数学/世界系」约定：
  //   yaw   向左为正（与 tools::xyz2ypd 一致，也与 command.yaw 一致）
  //   pitch 向上为正（与 tools::xyz2ypd 一致）
  // ⚠ 注意：下位机的 command.pitch 是「向下为正」（见 aimer.cpp 里的取负），
  //   所以 Decider::delta_angle 会在返回前把 pitch 取负，转换成命令约定。
  Eigen::Vector2d yaw_pitch_deg = Eigen::Vector2d::Zero();

  double distance = 0.0;                 // 到装甲板的直线距离（m）
  double reprojection_error = 0.0;       // 重投影误差（px），可用于过滤误检
};

// -----------------------------------------------------------------------------
//  全向相机 PnP 主入口：相机系 3D 点 -> 云台系角度
//
//  参数：
//    armor            检测到的装甲板（需要 armor.type 与 armor.points）
//    camera_matrix    该全向相机的内参（标定值或 approximate_camera_matrix 的近似值）
//    distort_coeffs   该全向相机的畸变（可为空 Mat => 无畸变）
//    R_camera2gimbal  相机 -> 云台 旋转
//    t_camera2gimbal  相机 -> 云台 平移（m）
//
//  内部会做几何合理性检查（目标必须在相机前方、距离在合理范围内），
//  不通过就返回 success = false，由调用方决定是否回退到旧方法。
// -----------------------------------------------------------------------------
OmniPnPResult solve_omni_armor(
  const auto_aim::Armor & armor, const cv::Mat & camera_matrix, const cv::Mat & distort_coeffs,
  const Eigen::Matrix3d & R_camera2gimbal, const Eigen::Vector3d & t_camera2gimbal);

}  // namespace omniperception

#endif  // OMNIPERCEPTION__OMNI_PNP_HPP

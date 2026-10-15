#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "armor.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  // =====================================================================================
  //  【新增】大小yaw结构支持：小yaw 相对 大yaw 的转角修正
  //
  //  本项目的哨兵是「大yaw + 小yaw」结构：
  //    · 大yaw：承载 IMU 与四个全向相机（IMU 测的就是大yaw的绝对姿态，
  //             也就是本类里 R_gimbal2world_ / 各处所说的 "gimbal" 系）；
  //    · 小yaw：在大yaw之上，承载主相机与枪管，可相对大yaw旋转。
  //
  //  而 R_camera2gimbal 是用 IMU 作参考系做手眼标定得到的（见
  //  calibration/calibrate_robotworld_handeye.cpp），即「相机 -> 大yaw」。
  //  主相机物理上装在小yaw上，所以标定值只在「标定当时的小yaw角」下成立；
  //  运行时小yaw一旦转动，主相机的位姿就会偏。
  //
  //  修正量 offset = psi_now - psi_calib（弧度），其中 psi 为小yaw相对大yaw的转角
  //  （与大yaw同向、向左为正）。修正方式是在手眼结果左侧乘 Rz(offset)：
  //      xyz_gimbal(真) = Rz(offset) * ( R_camera2gimbal * xyz_camera + t_camera2gimbal )
  //  推导：R_camera2bigyaw(psi) = Rz(psi) * R_camera2smallyaw，而标定值
  //        R_camera2gimbal_cal = Rz(psi_calib) * R_camera2smallyaw，
  //        两式相除即得 R_camera2bigyaw(psi) = Rz(psi - psi_calib) * R_camera2gimbal_cal。
  //  平移项同理整体左乘 Rz(offset)。
  //
  //  默认 offset = 0，即不做任何修正，行为与改动前完全一致。
  //
  //  ⚠ 一阶近似说明：上面的 t 修正是「绕大yaw原点旋转」的形式。如果小yaw的
  //    转轴相对大yaw原点有横向偏移 a（在标定文件里没有体现），严格式子应再补
  //    一项 t(真) = Rz(offset) * t_cal + (I - Rz(offset)) * a。
  //    偏移 a 在厘米量级、target 在米量级时该项可忽略（25° 时约 0.3° 量级），
  //    若实测发现 psi 大角度下仍有残差，可把 a 作为待标定量补进这里。
  //
  //  ⚠ 注意：修正后 armor.xyz_in_gimbal 表达的是「大yaw系」，这正好与全向相机
  //    （同样在大yaw上）以及入口里用 R_gimbal2world() 得到的 gimbal_pos 保持一致，
  //    所以全向分支不需要再额外减 psi。
  // =====================================================================================
  void set_small_yaw_offset(double offset_rad);

  // 当前的小yaw修正量（弧度），便于日志/调试
  double small_yaw_offset() const { return small_yaw_offset_; }

  void solve(Armor & armor) const;

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  // 【新增】小yaw相对大yaw的修正量（弧度），默认 0 表示不修正
  double small_yaw_offset_ = 0.0;

  // 绕 z 轴旋转的小工具
  Eigen::Matrix3d rot_z(double angle) const;

  // 施加了小yaw修正后的「相机 -> 大yaw」外参（等价于 Rz(offset) * 标定值）
  Eigen::Matrix3d R_camera2gimbal_eff() const;
  Eigen::Vector3d t_camera2gimbal_eff() const;

  void optimize_yaw(Armor & armor) const;

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP
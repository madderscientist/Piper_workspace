#pragma once

#include <array>
#include <map>
#include <optional>
#include <string>

#include "piper_x_driver/terminal.hpp"

namespace piper_x_driver {

struct FkResult {
  std::array<Vec3, 6> origins{};
  std::array<Vec3, 6> axes{};
  std::array<Vec3, 7> centers{};
  std::array<Mat3, 7> rots{};
};

struct ArmChainModel {
  std::string urdf_path;
  std::array<Vec3, 6> joint_xyz{};
  std::array<Vec3, 6> joint_rpy{};
  std::array<Mat3, 6> joint_rot{};
  Vec6 joint_lower{};
  Vec6 joint_upper{};
  std::array<double, 6> link_mass{};
  std::array<Vec3, 6> link_com{};
  std::array<Mat3, 6> link_inertia{};
};

std::string default_arm_urdf_path();
const ArmChainModel & load_arm_chain_model(const std::string & urdf_path = "");

double default_effector_mass_kg();
Vec3 default_effector_com_m();
const std::array<double, 7> & link_masses_default();
const std::array<Mat3, 6> & link_inertias();

FkResult fast_fk(const Vec6 & position, std::optional<Vec3> effector_com = std::nullopt);
Mat3 link6_rotation_in_base(const Vec6 & position);

Vec6 gravity_torques(
  const Vec6 & position,
  std::optional<double> mass_kg = std::nullopt,
  std::optional<Vec3> com_m = std::nullopt);

Vec6 joint_inertias(
  const Vec6 & position,
  const std::string & end_effector = "teaching_handle",
  std::optional<double> mass_kg = std::nullopt,
  std::optional<Vec3> com_m = std::nullopt,
  const Vec6 & scales = {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}},
  std::optional<Mat3> inertia_tensor = std::nullopt);

Mat6 geometric_jacobian(
  const Vec6 & position,
  const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
  const std::string & frame = "gripper");

struct IkResult {
  Vec6 joints{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  bool converged{false};
  int iterations{0};
  double position_error_m{0.0};
  double orientation_error_rad{0.0};
};

IkResult inverse_kinematics(
  const Vec3 & target_position,
  const Mat3 & target_rotation,
  const Vec6 & q_seed = {{0.0, 0.8, -1.2, 0.0, 0.4, 0.0}},
  const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
  std::optional<Vec6> lower_limits = std::nullopt,
  std::optional<Vec6> upper_limits = std::nullopt,
  double damping = 1e-3,
  int max_iters = 50,
  double pos_tol_m = 1e-4,
  double rot_tol_rad = 1e-3);

struct JointGravityModel {
  double scale{1.0};
  double angle_offset{0.0};
  double bias{0.0};
  double static_friction{0.0};

  void validate() const;
};

struct ArmGravityModel {
  std::string arm_id{"arm_1"};
  std::map<int, JointGravityModel> joints;
  std::string effector_model;
  double added_mass_kg{0.0};

  void validate() const;
  JointGravityModel joint(int index) const;
  ArmGravityModel for_effector(const std::string & model) const;
  ArmGravityModel with_added_mass(double mass_kg) const;
  ArmGravityModel with_joint(int index, const JointGravityModel & model) const;

  RigidBody effector_body(
    std::optional<double> mass_kg = std::nullopt,
    std::optional<Vec3> com_m = std::nullopt,
    const std::string & end_effector = "teaching_handle") const;

  Vec6 torques(
    const Vec6 & position,
    std::optional<double> mass_kg = std::nullopt,
    std::optional<Vec3> com_m = std::nullopt) const;

  Vec6 inertias(
    const Vec6 & position,
    const std::string & end_effector = "teaching_handle",
    std::optional<double> mass_kg = std::nullopt,
    std::optional<Vec3> com_m = std::nullopt) const;

  Mat6 jacobian(
    const Vec6 & position,
    const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
    const std::string & frame = "gripper") const;

  IkResult inverse_kinematics(
    const Vec3 & target_position,
    const Mat3 & target_rotation,
    const Vec6 & q_seed = {{0.0, 0.8, -1.2, 0.0, 0.4, 0.0}},
    const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
    std::optional<Vec6> lower_limits = std::nullopt,
    std::optional<Vec6> upper_limits = std::nullopt,
    double damping = 1e-3,
    int max_iters = 50,
    double pos_tol_m = 1e-4,
    double rot_tol_rad = 1e-3) const;

  std::array<Mat3, 7> scaled_inertia_tensors(
    const std::string & end_effector = "teaching_handle") const;
};

Vec6 max_critical_kd(
  const Vec6 & kp,
  const std::string & end_effector = "teaching_handle",
  const ArmGravityModel * gravity_model = nullptr);

Vec6 estimate_joint_torques(
  const Vec6 & position,
  const Vec6 & velocity,
  const Vec6 & target_position,
  const Vec6 & target_velocity,
  const Vec6 & kp,
  const Vec6 & kd,
  const Vec6 & torque_cmd = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  const Vec6 & acceleration = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  const std::string & end_effector = "teaching_handle",
  std::optional<double> mass_kg = std::nullopt,
  std::optional<Vec3> com_m = std::nullopt,
  const ArmGravityModel * gravity_model = nullptr);

Vec6 end_effector_wrench(
  const Vec6 & joint_torques,
  const Vec6 & position,
  const ArmGravityModel * gravity_model = nullptr,
  const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
  double damping = 1e-2,
  const std::string & frame = "gripper");

Vec6 estimate_end_wrench(
  const Vec6 & position,
  const Vec6 & velocity,
  const Vec6 & target_position,
  const Vec6 & target_velocity,
  const Vec6 & kp,
  const Vec6 & kd,
  const Vec6 & torque_cmd = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  const Vec6 & acceleration = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}},
  const std::string & end_effector = "teaching_handle",
  std::optional<double> mass_kg = std::nullopt,
  std::optional<Vec3> com_m = std::nullopt,
  const ArmGravityModel * gravity_model = nullptr,
  const Vec3 & tool_offset_m = {{0.0, 0.0, 0.0}},
  double damping = 1e-2,
  const std::string & frame = "gripper");

ArmGravityModel load_gravity_model(const std::string & path);
void save_gravity_model(const ArmGravityModel & model, const std::string & path);

}  // namespace piper_x_driver

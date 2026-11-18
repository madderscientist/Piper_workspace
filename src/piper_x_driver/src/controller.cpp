#include "piper_x_driver/controller.hpp"

#include <Eigen/Dense>

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace piper_x_driver {
namespace {

constexpr double kPi = 3.14159265358979323846;

double raised(double value, double power) noexcept {
  if (power == 1.0) return value;
  if (power == 2.0) return value * value;
  if (power == 3.0) return value * value * value;
  return std::pow(value, power);
}

bool is_valid_mode(const std::string & m) {
  return m == "standby" || m == "joint" || m == "point" || m == "linear" ||
         m == "mit" || m == "drag" || m == "hold";
}

void check_finite_6(const Vec6 & v, const std::string & name) {
  for (double x : v) {
    if (!std::isfinite(x)) {
      throw std::invalid_argument(name + ": six finite values required");
    }
  }
}

void check_finite_3(const Vec3 & v, const std::string & name) {
  for (double x : v) {
    if (!std::isfinite(x)) {
      throw std::invalid_argument(name + ": three finite values required");
    }
  }
}

}  // namespace

double compute_adaptive_stiffness_scale(
  double position_error,
  double scale,
  double b,
  double power)
{
  if (scale <= 0.0) {
    return 1.0;
  }
  const double ratio = std::abs(position_error) / b;
  return 1.0 + scale / (raised(ratio, power) + 1.0);
}

double RateMeter::hz(double now) noexcept {
  const double cutoff = now - window_;
  while (size_ > 0 && stamps_[head_] < cutoff) {
    head_ = (head_ + 1) & (kCapacity - 1);
    --size_;
  }
  if (size_ < 2) {
    return 0.0;
  }
  const size_t tail = (head_ + size_ - 1) & (kCapacity - 1);
  const double span = stamps_[tail] - stamps_[head_];
  if (span <= 1e-6) {
    return 0.0;
  }
  return std::round(((static_cast<double>(size_) - 1.0) / span) * 10.0) / 10.0;
}

Controller::Controller(
  ArmBackend & arm,
  std::function<double()> clock_fn,
  std::optional<Effector> eff,
  double g_scale,
  std::optional<ArmGravityModel> g_model,
  double f_scale,
  std::optional<ArmFrictionModel> f_model,
  double f_err_eps,
  double f_vel_eps,
  double target_vel_cutoff_hz,
  double s_a,
  double adapt_b,
  double adapt_power,
  double ramp_s,
  const Vec6 & kp_in,
  const Vec6 & kd_in)
: effector(std::move(eff)),
  gravity_model(std::move(g_model)),
  friction_model(std::move(f_model)),
  arm_(arm),
  clock_(std::move(clock_fn))
{
  gravity_scale = bounded(g_scale, "gravity_scale", 0.0, 1.0);
  gravity_scale_applied = gravity_scale;
  friction_scale = bounded(f_scale, "friction_scale", 0.0, 1.0);
  friction_scale_applied = friction_scale;
  friction_error_epsilon = bounded(f_err_eps, "friction_error_epsilon", 0.0, 10.0);
  friction_velocity_epsilon = bounded(f_vel_eps, "friction_velocity_epsilon", 0.0, 100.0);
  target_velocity_cutoff_hz = bounded(
    target_vel_cutoff_hz, "target_velocity_cutoff_hz", 0.0, 1000.0);
  if (target_velocity_cutoff_hz <= 0.0) {
    throw std::invalid_argument("target_velocity_cutoff_hz must be finite and positive");
  }
  adaptive_stiffness_scale_val = bounded(s_a, "s_a", 0.0, 20.0);
  offset_ramp_s = bounded(ramp_s, "offset_ramp_s", 0.0, 30.0);
  adaptive_stiffness_scale_applied =
    (offset_ramp_s > 0.0) ? 0.0 : adaptive_stiffness_scale_val;
  adaptive_stiffness_b = bounded(adapt_b, "adaptive_stiffness_b", 0.0, 10.0);
  if (adaptive_stiffness_b <= 0.0) {
    throw std::invalid_argument("adaptive_stiffness_b must be finite and positive");
  }
  adaptive_stiffness_power = bounded(adapt_power, "adaptive_stiffness_power", 0.0, 20.0);
  if (adaptive_stiffness_power <= 0.0) {
    throw std::invalid_argument("adaptive_stiffness_power must be finite and positive");
  }
  gravity_updated = clock_();
  kp = kp_in;
  kd = resolve_kd(kp, kd_in);
  applied_kp = kp;
  applied_kd = kd;
}

Feedback Controller::feedback(std::optional<Feedback> sample) {
  Feedback s = sample.has_value() ? std::move(*sample) : arm_.snapshot();
  if (!s.status.has_value() || !fresh(s.age)) {
    const std::string details = feedback_problem(s.age);
    throw std::invalid_argument(
      "Complete fresh joint/gripper/status/enable feedback required: " +
      (details.empty() ? std::string("arm status missing") : details));
  }
  if (!s.position.has_value()) {
    throw std::invalid_argument("Complete position feedback required");
  }
  return s;
}

Feedback Controller::healthy(bool require_enabled, std::optional<Feedback> sample) {
  Feedback s = feedback(std::move(sample));
  const int st = s.status->arm_status;
  if (!(st == 0 || st == 11 || st == 12 || st == 13) || s.status->err_code != 0) {
    throw std::invalid_argument(
      "Firmware fault: arm_status=" + std::to_string(st) +
      ", err_code=" + std::to_string(s.status->err_code));
  }
  if (require_enabled) {
    for (int i = 0; i < 6; ++i) {
      if (!s.enabled[i]) {
        throw std::invalid_argument("All six joints must be enabled");
      }
    }
  }
  return s;
}

void Controller::power(bool enabled) {
  if (enabled) {
    arm_.leave_drag();
    const Feedback s = healthy(false);
    arm_.power(true, (*s.position)[6]);
  } else {
    arm_.power(false, 0.0);
    clear_power_state();
  }
}

void Controller::reset_dynamics_state(bool clear_compensation) {
  reset_wrench_estimation();
  target_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  target_hold_time_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  previous_target_.reset();
  if (clear_compensation) {
    gravity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    friction = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    output_torque = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    last_mit_command_.reset();
    gripper_friction = 0.0;
    gripper_assist_active_ = false;
  }
}

void Controller::clear_power_state() {
  mode = "standby";
  mit_target.reset();
  cartesian_mit_target.reset();
  mit_received.reset();
  gripper_received_.reset();
  reset_dynamics_state(true);
  hold_reason.clear();
  comp_rate.clear();
  cmd_tx_rate.clear();
  mit_tx_rate.clear();
  grip_tx_rate.clear();
}

void Controller::reconfigure(const ControllerReconfigureArgs & args) {
  const auto validate = [](
    const std::optional<double> & value, const char * name, double upper, bool positive = false)
  {
    if (!value.has_value()) return;
    bounded(*value, name, 0.0, upper);
    if (positive && *value <= 0.0) {
      throw std::invalid_argument(std::string(name) + " must be finite and positive");
    }
  };
  validate(args.gravity_scale, "gravity_scale", 1.0);
  validate(args.friction_scale, "friction_scale", 1.0);
  validate(args.friction_error_epsilon, "friction_error_epsilon", 10.0);
  validate(args.friction_velocity_epsilon, "friction_velocity_epsilon", 100.0);
  validate(args.target_velocity_cutoff_hz, "target_velocity_cutoff_hz", 1000.0, true);
  validate(args.s_a, "s_a", 20.0);
  validate(args.adaptive_stiffness_b, "adaptive_stiffness_b", 10.0, true);
  validate(args.adaptive_stiffness_power, "adaptive_stiffness_power", 20.0, true);
  validate(args.offset_ramp_s, "offset_ramp_s", 30.0);

  if (args.effector.has_value() || args.update_models) {
    effector = args.effector;
    cached_auto_kd_kp_.reset();
  }
  if (args.gravity_scale.has_value()) {
    gravity_scale = *args.gravity_scale;
    if (gravity_scale == 0.0) {
      gravity_scale_applied = 0.0;
    }
  }
  if (args.friction_scale.has_value()) {
    friction_scale = *args.friction_scale;
    if (friction_scale == 0.0) {
      friction_scale_applied = 0.0;
    }
  }
  if (args.friction_error_epsilon.has_value()) {
    friction_error_epsilon = *args.friction_error_epsilon;
  }
  if (args.friction_velocity_epsilon.has_value()) {
    friction_velocity_epsilon = *args.friction_velocity_epsilon;
  }
  if (args.target_velocity_cutoff_hz.has_value()) {
    target_velocity_cutoff_hz = *args.target_velocity_cutoff_hz;
  }
  if (args.s_a.has_value()) {
    adaptive_stiffness_scale_val = *args.s_a;
    if (adaptive_stiffness_scale_val == 0.0) {
      adaptive_stiffness_scale_applied = 0.0;
    }
  }
  if (args.adaptive_stiffness_b.has_value()) {
    adaptive_stiffness_b = *args.adaptive_stiffness_b;
  }
  if (args.adaptive_stiffness_power.has_value()) {
    adaptive_stiffness_power = *args.adaptive_stiffness_power;
  }
  if (args.offset_ramp_s.has_value()) {
    offset_ramp_s = *args.offset_ramp_s;
  }
  if (args.update_models) {
    gravity_model = args.gravity_model;
    friction_model = args.friction_model;
  }
}

bool Controller::shutdown_to_drag() {
  if (mode != "drag" && (mode == "mit" || arm_.is_powered())) {
    arm_.enter_drag();
    clear_power_state();
    mode = "drag";
    return true;
  }
  return false;
}

MitTarget Controller::gravity_command(const Feedback & sample) const {
  Vec6 q{};
  for (int i = 0; i < 6; ++i) {
    q[i] = (*sample.position)[i];
  }
  MitTarget t;
  t.position = q;
  t.kp = kp;
  t.kd = kd;
  return t;
}

void Controller::start_mit() {
  power(true);
  const Feedback sample = healthy();
  if (sample.status->teach_status == 1) {
    arm_.mode("hold");
  }
  const MitTarget command = gravity_command(sample);
  gravity_scale_applied = gravity_scale;
  friction_scale_applied = friction_scale;
  adaptive_stiffness_scale_applied =
    (offset_ramp_s > 0.0) ? 0.0 : adaptive_stiffness_scale_val;
  gravity_updated = clock_();
  prev_velocity_.reset();
  vel_updated_.reset();
  target_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  target_hold_time_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  previous_target_.reset();
  mit_started = clock_();
  mit_ready = (sample.status->ctrl_mode == 1 && sample.status->mode_feedback == 6);
  send_mit(command, sample);
  arm_.mode("mit");
  mode = "mit";
  hold_reason.clear();
}

void Controller::set_mode(const std::string & next_mode, bool preload_mit) {
  if (!is_valid_mode(next_mode)) {
    throw std::invalid_argument("Unknown mode " + next_mode);
  }
  if (next_mode != "drag") {
    arm_.leave_drag();
  }
  if (next_mode != "standby" && next_mode != "drag" && next_mode != "hold") {
    const Feedback sample = healthy();
    if (next_mode == "mit") {
      if (sample.status->teach_status == 1) {
        arm_.mode("hold");
      }
      const double now = clock_();
      const bool has_fresh =
        (mit_target.has_value() || cartesian_mit_target.has_value()) &&
        mit_received.has_value() && (now - *mit_received <= 0.2);
      if (preload_mit && !has_fresh) {
        mit(gravity_command(sample));
      }
      if (!(mit_target.has_value() || cartesian_mit_target.has_value()) ||
          !mit_received.has_value() || clock_() - *mit_received > 0.2)
      {
        throw std::invalid_argument("Publish a fresh six-joint MIT target before entering MIT");
      }
      mit_started = clock_();
      mit_ready = false;
      gravity_scale_applied = gravity_scale;
      friction_scale_applied = friction_scale;
      adaptive_stiffness_scale_applied =
        (offset_ramp_s > 0.0) ? 0.0 : adaptive_stiffness_scale_val;
      gravity_updated = clock_();
      reset_dynamics_state(false);
      const MitTarget first_cmd = cartesian_mit_target.has_value()
        ? compute_cartesian_mit(*cartesian_mit_target, sample)
        : *mit_target;
      send_mit(first_cmd, sample);
    }
  }
  arm_.mode(next_mode);
  mode = next_mode;
  hold_reason.clear();
  if (next_mode == "mit") {
    mit_started = clock_();
    mit_ready = false;
    gravity_updated = clock_();
  } else {
    reset_dynamics_state(true);
  }
}

Feedback Controller::require_mode(const std::string & m1, const std::string & m2) {
  if (mode != m1 && (m2.empty() || mode != m2)) {
    throw std::invalid_argument("Select the matching mode before sending commands");
  }
  const Feedback sample = healthy();
  const int ctrl = sample.status->ctrl_mode;
  if (ctrl != 0 && ctrl != 1 && ctrl != 2) {
    throw std::invalid_argument(
      "Arm is under non-CAN control (ctrl_mode=" + std::to_string(ctrl) + ")");
  }
  if (sample.status->teach_status == 1) {
    arm_.mode("hold");
  }
  return sample;
}

void Controller::joints(const Vec6 & position, uint8_t speed_percent) {
  check_finite_6(position, "position");
  if (mode == "mit") {
    MitTarget cmd;
    cmd.position = position;
    if (mit_target.has_value()) {
      cmd.velocity = mit_target->velocity;
      cmd.kp = mit_target->kp;
      cmd.kd = mit_target->kd;
      cmd.torque = mit_target->torque;
    } else {
      cmd.kp = kp;
      cmd.kd = kd;
    }
    mit(cmd);
    return;
  }
  if (speed_percent < 1 || speed_percent > 100) {
    throw std::invalid_argument("speed_percent must be in 1..100");
  }
  const double now = clock_();
  ros_cmd_rate.mark(now);
  require_mode("joint");
  arm_.joints(position, speed_percent);
  cmd_tx_rate.mark(now);
}

void Controller::cartesian(const Vec6 & pose, uint8_t speed_percent) {
  check_finite_6(pose, "pose");
  if (speed_percent < 1 || speed_percent > 100) {
    throw std::invalid_argument("speed_percent must be in 1..100");
  }
  const double now = clock_();
  ros_cmd_rate.mark(now);
  require_mode("point", "linear");
  arm_.cartesian(pose, speed_percent, mode);
  cmd_tx_rate.mark(now);
}

Vec6 Controller::resolve_kd(const Vec6 & kp_in, const Vec6 & kd_in) const {
  bool all_nonneg = true;
  for (double v : kd_in) {
    if (v < 0.0) {
      all_nonneg = false;
      break;
    }
  }
  if (all_nonneg) {
    return kd_in;
  }
  if (!cached_auto_kd_kp_.has_value() || *cached_auto_kd_kp_ != kp_in) {
    const std::string eff_name = effector.has_value() ? effector->model : "teaching_handle";
    cached_auto_kd_ = max_critical_kd(
      kp_in, eff_name, gravity_model.has_value() ? &(*gravity_model) : nullptr);
    cached_auto_kd_kp_ = kp_in;
  }
  Vec6 out{};
  for (int i = 0; i < 6; ++i) {
    out[i] = (kd_in[i] < 0.0) ? cached_auto_kd_[i] : kd_in[i];
  }
  return out;
}

std::pair<Vec6, Vec6> Controller::resolve_command_gains(const MitTarget & cmd) const {
  Vec6 kp_vec{};
  bool all_kp_neg = true;
  bool all_kd_neg = true;
  for (int i = 0; i < 6; ++i) {
    if (cmd.kp[i] >= 0.0) all_kp_neg = false;
    if (cmd.kd[i] >= 0.0) all_kd_neg = false;
    kp_vec[i] = (cmd.kp[i] < 0.0) ? kp[i] : cmd.kp[i];
  }
  const Vec6 kd_vec = (all_kp_neg && all_kd_neg) ? kd : resolve_kd(kp_vec, cmd.kd);
  return {kp_vec, kd_vec};
}

void Controller::mit(const MitTarget & raw_cmd) {
  check_finite_6(raw_cmd.position, "position");
  check_finite_6(raw_cmd.velocity, "velocity");
  check_finite_6(raw_cmd.kp, "kp");
  check_finite_6(raw_cmd.kd, "kd");
  check_finite_6(raw_cmd.torque, "torque");

  MitTarget command = raw_cmd;
  bool has_neg_gain = false;
  for (int i = 0; i < 6; ++i) {
    if (command.kp[i] < 0.0 || command.kd[i] < 0.0) {
      has_neg_gain = true;
      break;
    }
  }
  if (has_neg_gain) {
    const auto [kp_res, kd_res] = resolve_command_gains(command);
    command.kp = kp_res;
    command.kd = kd_res;
  }
  for (int i = 0; i < 6; ++i) {
    command.position[i] = std::clamp(command.position[i], -12.5, 12.5);
    command.velocity[i] = std::clamp(command.velocity[i], -45.0, 45.0);
    command.kp[i] = std::clamp(command.kp[i], 0.0, 500.0);
    command.kd[i] = std::clamp(command.kd[i], 0.0, 100.0);
    const double t_max = 8.0 * JOINT_TORQUE_B[i] * JOINT_TORQUE_C[i];
    command.torque[i] = std::clamp(command.torque[i], -t_max, t_max);
  }
  mit_target = command;
  cartesian_mit_target.reset();
  const double now = clock_();
  mit_received = now;
  ros_cmd_rate.mark(now);
}

void Controller::cartesian_mit(const CartesianMitTarget & raw_cmd) {
  check_finite_3(raw_cmd.position, "position");
  check_finite_3(raw_cmd.tool_offset, "tool_offset");
  check_finite_6(raw_cmd.kp, "kp");
  double q_norm2 = 0.0;
  for (double c : raw_cmd.quat_wxyz) {
    if (!std::isfinite(c)) {
      throw std::invalid_argument("quat_wxyz: four finite values required");
    }
    q_norm2 += c * c;
  }
  const double q_norm = std::sqrt(q_norm2);
  if (q_norm < 1e-9) {
    throw std::invalid_argument("quat_wxyz must have non-zero norm");
  }

  CartesianMitTarget cmd = raw_cmd;
  for (double & c : cmd.quat_wxyz) {
    c /= q_norm;
  }
  for (int i = 0; i < 6; ++i) {
    if (cmd.kp[i] < 0.0) {
      throw std::invalid_argument("Cartesian MIT kp must be non-negative");
    }
  }
  cartesian_mit_target = cmd;
  mit_target.reset();
  const double now = clock_();
  mit_received = now;
  ros_cmd_rate.mark(now);
}

MitTarget Controller::compute_cartesian_mit(
  const CartesianMitTarget & cart, const Feedback & sample) const
{
  Vec6 q_meas{};
  Vec6 q_eff{};
  for (int i = 0; i < 6; ++i) {
    q_meas[i] = (*sample.position)[i];
    q_eff[i] = q_meas[i] +
      (gravity_model.has_value() ? gravity_model->joint(i + 1).angle_offset : 0.0);
  }
  MitTarget out;
  out.position = q_meas;

  const Eigen::Map<const Eigen::Matrix<double, 6, 1>> kp_vec(cart.kp.data());
  if (kp_vec.maxCoeff() <= 0.0) {
    return out;
  }

  const Vec6 dq = sample.has_velocity_torque
    ? sample.velocity
    : Vec6{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};

  const FkResult fk = fast_fk(q_eff, cart.tool_offset);
  const Vec3 & p_curr = fk.centers[6];
  Eigen::Matrix3d r_curr;
  for (int r = 0; r < 3; ++r) {
    for (int c = 0; c < 3; ++c) {
      r_curr(r, c) = fk.rots[5][r][c];
    }
  }
  const Eigen::Matrix3d r_des = Eigen::Quaterniond(
    cart.quat_wxyz[0], cart.quat_wxyz[1], cart.quat_wxyz[2], cart.quat_wxyz[3]).toRotationMatrix();
  Eigen::AngleAxisd aa(r_des * r_curr.transpose());
  double rot_angle = aa.angle();
  if (rot_angle > kPi) {
    rot_angle -= 2.0 * kPi;
  }
  const Eigen::Vector3d e_r = aa.axis() * rot_angle;

  Eigen::Matrix<double, 6, 6> j_mat;
  for (int i = 0; i < 6; ++i) {
    const Eigen::Vector3d z_i(fk.axes[i][0], fk.axes[i][1], fk.axes[i][2]);
    const Eigen::Vector3d r_i(
      p_curr[0] - fk.origins[i][0],
      p_curr[1] - fk.origins[i][1],
      p_curr[2] - fk.origins[i][2]);
    j_mat.block<3, 1>(0, i) = z_i.cross(r_i);
    j_mat.block<3, 1>(3, i) = z_i;
  }

  constexpr double kReg = 1e-4;
  constexpr double kTaskZeta = 0.28;
  constexpr double kMaxTaskRate = 55.0;
  constexpr double kModalZeta = 0.44;
  static constexpr std::array<long, 6> kCarrierLsbMult = {{2, 2, 2, 6, 6, 6}};

  const std::string eff_name = effector.has_value() ? effector->model : "teaching_handle";
  const std::optional<double> eff_mass =
    effector.has_value() ? std::optional<double>(effector->mass_kg) : std::nullopt;
  const std::optional<Vec3> eff_com =
    effector.has_value() ? std::optional<Vec3>(effector->com_m) : std::nullopt;
  const Vec6 inertias = gravity_model.has_value()
    ? gravity_model->inertias(q_meas, eff_name, eff_mass, eff_com)
    : joint_inertias(q_meas, eff_name, eff_mass, eff_com);

  Eigen::Matrix<double, 6, 1> inv_m;
  Eigen::Matrix<double, 6, 1> inv_sqrt_m;
  for (int i = 0; i < 6; ++i) {
    const double m_i = std::max(inertias[i], 1e-6);
    inv_m(i) = 1.0 / m_i;
    inv_sqrt_m(i) = 1.0 / std::sqrt(m_i);
  }

  // SO(3)-invariant harmonic-mean operational-space inertia (m_iso = 3 / tr(J_v M^-1 J_v^T))
  const Eigen::Matrix<double, 6, 1> inv_lam = j_mat.cwiseProduct(j_mat) * inv_m;
  const double m_iso = 3.0 / std::max(inv_lam.head<3>().sum(), 1e-6);
  const double i_iso = 3.0 / std::max(inv_lam.tail<3>().sum(), 1e-6);
  const Eigen::Matrix<double, 6, 1> twist =
    j_mat * Eigen::Map<const Eigen::Matrix<double, 6, 1>>(dq.data());

  Eigen::Matrix<double, 6, 1> wrench_b;
  for (int a = 0; a < 6; ++a) {
    const double err_a = (a < 3) ? (cart.position[a] - p_curr[a]) : e_r(a - 3);
    const double lam_iso = (a < 3) ? m_iso : i_iso;
    const double lam_axis = 1.0 / std::max(inv_lam(a), 1e-6);
    const double kd_task_a = std::min(
      2.0 * kTaskZeta * std::sqrt(lam_iso * cart.kp[a]),
      kMaxTaskRate * lam_axis);
    wrench_b(a) = cart.kp[a] * err_a - kd_task_a * twist(a);
  }

  const Eigen::Matrix<double, 6, 1> tau_raw = j_mat.transpose() * wrench_b;
  Vec6 tau_cart{};
  for (int i = 0; i < 6; ++i) {
    const double t_max = 8.0 * JOINT_TORQUE_B[i] * JOINT_TORQUE_C[i];
    tau_cart[i] = std::clamp(tau_raw(i), -t_max, t_max);
  }

  Eigen::Matrix<double, 6, 6> k_q = j_mat.transpose() * kp_vec.asDiagonal() * j_mat;
  k_q.diagonal().array() += kReg;
  const Eigen::LLT<Eigen::Matrix<double, 6, 6>> llt(k_q);
  Vec6 k_schur{};
  if (llt.info() == Eigen::Success) {
    const Eigen::Matrix<double, 6, 6> c_q = llt.solve(Eigen::Matrix<double, 6, 6>::Identity());
    for (int i = 0; i < 6; ++i) {
      const double c_ii = c_q(i, i);
      k_schur[i] = (c_ii > 1e-9) ? std::max(0.0, (1.0 / c_ii) - kReg) : 0.0;
    }
  }
  const Vec6 kd_schur = max_critical_kd(
    k_schur, eff_name, gravity_model.has_value() ? &(*gravity_model) : nullptr);

  Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, 6, 6>> eig(
    inv_sqrt_m.asDiagonal() * k_q * inv_sqrt_m.asDiagonal());
  Vec6 kd_modal{};
  if (eig.info() == Eigen::Success) {
    const auto & vals = eig.eigenvalues();
    const auto & vecs = eig.eigenvectors();
    for (int i = 0; i < 6; ++i) {
      double sqrt_a_ii = 0.0;
      for (int m = 0; m < 6; ++m) {
        sqrt_a_ii += vecs(i, m) * vecs(i, m) * std::sqrt(std::max(vals(m), 0.0));
      }
      kd_modal[i] = 2.0 * kModalZeta * inertias[i] * sqrt_a_ii;
    }
  }

  for (int i = 0; i < 6; ++i) {
    out.kd[i] = std::clamp(
      std::max({2.0 * MIT_KD_LSB[i], kd_schur[i], kd_modal[i]}), 0.0, 100.0);
    // Fold Cartesian spring torque into an exact quantized positive kp carrier so the
    // motor ESC 20 kHz position & velocity loops remain active with zero quantization drift.
    const double kp_foc = static_cast<double>(kCarrierLsbMult[i]) * MIT_KP_LSB[i];
    const double dq_fold = std::clamp(
      tau_cart[i] / kp_foc,
      std::max(-0.8, -11.5 - q_meas[i]),
      std::min(0.8, 11.5 - q_meas[i]));
    out.kp[i] = kp_foc;
    out.position[i] = q_meas[i] + dq_fold;
    out.torque[i] = tau_cart[i] - kp_foc * dq_fold;
  }
  return out;
}

void Controller::release_mit() {
  mit_target.reset();
  cartesian_mit_target.reset();
  mit_received.reset();
}

void Controller::reset_wrench_estimation() {
  prev_velocity_.reset();
  vel_updated_.reset();
  joint_acceleration = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  inertia_torque = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  external_torque = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  end_wrench = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
}

Vec6 Controller::update_acceleration(
  const Feedback & sample, const MitTarget & command, double now)
{
  const bool has_vel = sample.has_velocity_torque;
  const Vec6 velocity = has_vel ? sample.velocity : command.velocity;
  Vec6 accel{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (has_vel) {
    const Vec6 vel_now = sample.velocity;
    if (!prev_velocity_.has_value() || !vel_updated_.has_value()) {
      prev_velocity_ = vel_now;
      vel_updated_ = now;
    } else {
      const double dt_vel = now - *vel_updated_;
      if ((0.0 <= dt_vel && dt_vel < 0.0015) ||
          (vel_now == *prev_velocity_ && 0.0 <= dt_vel && dt_vel < 0.0045))
      {
        accel = joint_acceleration;
      } else if (0.0015 <= dt_vel && dt_vel <= 0.2) {
        const double alpha = dt_vel / (0.04 + dt_vel);
        for (int i = 0; i < 6; ++i) {
          accel[i] = (1.0 - alpha) * joint_acceleration[i] +
                     alpha * ((vel_now[i] - (*prev_velocity_)[i]) / dt_vel);
        }
        prev_velocity_ = vel_now;
        vel_updated_ = now;
      } else {
        prev_velocity_ = vel_now;
        vel_updated_ = now;
      }
    }
  } else {
    prev_velocity_.reset();
    vel_updated_.reset();
  }
  joint_acceleration = accel;
  return velocity;
}

Vec6 Controller::update_target_velocity(const Vec6 & target_position, double elapsed) {
  if (!previous_target_.has_value()) {
    previous_target_ = target_position;
    target_velocity = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    target_hold_time_ = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    return target_velocity;
  }
  if (elapsed <= 0.0) {
    return target_velocity;
  }
  const double alpha = 1.0 - std::exp(-2.0 * kPi * target_velocity_cutoff_hz * elapsed);
  for (int i = 0; i < 6; ++i) {
    target_hold_time_[i] += elapsed;
    if (target_position[i] != (*previous_target_)[i]) {
      const double raw = (target_position[i] - (*previous_target_)[i]) / target_hold_time_[i];
      target_velocity[i] += alpha * (raw - target_velocity[i]);
      (*previous_target_)[i] = target_position[i];
      target_hold_time_[i] = 0.0;
    } else if (target_hold_time_[i] > TARGET_STALE_SECONDS) {
      target_velocity[i] -= alpha * target_velocity[i];
    }
  }
  return target_velocity;
}

std::pair<Vec6, Vec6> Controller::resolve_adaptive_gains(
  const Vec6 & base_kp, const Vec6 & base_kd, const Vec6 & position_errors) const
{
  const double amount = adaptive_stiffness_scale_applied;
  if (amount <= 0.0) {
    return {base_kp, base_kd};
  }
  Vec6 kp_out{}, kd_out{};
  for (int i = 0; i < 6; ++i) {
    const double kp_f = base_kp[i];
    const double kd_f = base_kd[i];
    if (kp_f <= 0.0 || kd_f <= 0.0) {
      kp_out[i] = kp_f;
      kd_out[i] = kd_f;
      continue;
    }
    const double mult = compute_adaptive_stiffness_scale(
      position_errors[i], amount, adaptive_stiffness_b, adaptive_stiffness_power);
    const double kp_new = std::min(500.0, kp_f * mult);
    const double scale_real = kp_new / kp_f;
    const double kd_new = std::min(100.0, kd_f * std::sqrt(scale_real));
    kp_out[i] = kp_new;
    kd_out[i] = kd_new;
  }
  return {kp_out, kd_out};
}

void Controller::update_wrench_estimate(
  const MitTarget & command,
  const Feedback & sample,
  const Vec6 & resolved_kp,
  const Vec6 & resolved_kd,
  const Vec6 & velocity,
  const std::optional<Vec6> & raw_gravity,
  const std::optional<Vec6> & applied_gravity,
  const std::optional<Vec6> & raw_friction,
  const std::optional<Vec6> & applied_friction)
{
  const bool real_hw = arm_.is_real_hardware();
  Vec6 q_meas{};
  for (int i = 0; i < 6; ++i) {
    q_meas[i] = (*sample.position)[i];
  }
  for (int i = 0; i < 6; ++i) {
    const long raw_uint = (resolved_kp[i] > 0.0)
      ? std::lrint(resolved_kp[i] / MIT_KP_LSB[i])
      : 0;
    double kp_eff = 0.0;
    if (raw_uint > 0) {
      kp_eff = real_hw
        ? (static_cast<double>(std::min<long>(4095, raw_uint)) * MIT_KP_LSB[i])
        : resolved_kp[i];
    }
    const double uncomp_g = (raw_gravity.has_value() && applied_gravity.has_value())
      ? ((*raw_gravity)[i] - (*applied_gravity)[i])
      : 0.0;
    const double uncomp_f = (raw_friction.has_value() && applied_friction.has_value())
      ? ((*raw_friction)[i] - (*applied_friction)[i])
      : 0.0;
    external_torque[i] =
      kp_eff * (q_meas[i] - command.position[i]) +
      resolved_kd[i] * (velocity[i] - command.velocity[i]) -
      command.torque[i] +
      inertia_torque[i] +
      uncomp_g +
      uncomp_f;
  }
  end_wrench = end_effector_wrench(
    external_torque, q_meas, gravity_model.has_value() ? &(*gravity_model) : nullptr);
}

void Controller::send_mit(
  const MitTarget & command, const Feedback & sample, bool compute_wrench)
{
  std::optional<double> eff_mass;
  std::optional<Vec3> eff_com;
  if (effector.has_value()) {
    eff_mass = effector->mass_kg;
    eff_com = effector->com_m;
  }
  const double now = clock_();
  const double dt = std::max(0.0, now - gravity_updated);
  gravity_scale_applied += std::clamp(gravity_scale - gravity_scale_applied, -dt, dt);
  friction_scale_applied += std::clamp(friction_scale - friction_scale_applied, -dt, dt);
  if (offset_ramp_s <= 0.0) {
    adaptive_stiffness_scale_applied = adaptive_stiffness_scale_val;
  } else if (mit_ready) {
    const double sa_step = dt * std::max(1.0, std::abs(adaptive_stiffness_scale_val)) / offset_ramp_s;
    adaptive_stiffness_scale_applied += std::clamp(
      adaptive_stiffness_scale_val - adaptive_stiffness_scale_applied, -sa_step, sa_step);
  }
  gravity_updated = now;

  Vec6 q_meas{};
  Vec6 position_errors{};
  for (int i = 0; i < 6; ++i) {
    q_meas[i] = (*sample.position)[i];
    position_errors[i] = command.position[i] - q_meas[i];
  }
  const Vec6 step_target_vel = update_target_velocity(command.position, dt);
  Vec6 target_velocities{};
  for (int i = 0; i < 6; ++i) {
    target_velocities[i] =
      (std::abs(command.velocity[i]) > 1e-9) ? command.velocity[i] : step_target_vel[i];
  }
  const bool is_cart_mit = cartesian_mit_target.has_value();
  const Vec6 base_kd = resolve_kd(command.kp, command.kd);
  const auto [effective_kp, resolved_kd] = is_cart_mit
    ? std::pair<Vec6, Vec6>{command.kp, base_kd}
    : resolve_adaptive_gains(command.kp, base_kd, position_errors);
  applied_kp = effective_kp;
  applied_kd = resolved_kd;

  const Vec6 raw_gravity = gravity_model.has_value()
    ? gravity_model->torques(q_meas, eff_mass, eff_com)
    : gravity_torques(q_meas, eff_mass, eff_com);
  Vec6 g_vec{};
  for (int i = 0; i < 6; ++i) {
    g_vec[i] = gravity_scale_applied * raw_gravity[i];
  }

  const Vec6 velocity = compute_wrench
    ? update_acceleration(sample, command, now)
    : (sample.has_velocity_torque ? sample.velocity : command.velocity);

  Vec6 static_frictions{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (gravity_model.has_value()) {
    for (int i = 0; i < 6; ++i) {
      static_frictions[i] = gravity_model->joint(i + 1).static_friction;
    }
  }
  std::array<bool, 6> active_imp{};
  Vec6 stiction_errors{}, stiction_target_vels{};
  for (int i = 0; i < 6; ++i) {
    active_imp[i] =
      (resolved_kd[i] >= MIT_KD_LSB[i]) && (std::lrint(effective_kp[i] / MIT_KP_LSB[i]) > 0);
    if (!active_imp[i]) {
      stiction_errors[i] = 0.0;
      stiction_target_vels[i] = 0.0;
    } else if (is_cart_mit) {
      const double ref_kp = std::max(effective_kp[i], kp[i]);
      const double total_spring_tau = effective_kp[i] * position_errors[i] + command.torque[i];
      stiction_errors[i] = total_spring_tau / std::max(ref_kp, 1e-6);
      stiction_target_vels[i] = 0.0;
    } else {
      stiction_errors[i] = position_errors[i];
      stiction_target_vels[i] = target_velocities[i];
    }
  }
  const Vec6 raw_friction = friction_model.has_value()
    ? friction_model->torques(
        q_meas, velocity, stiction_errors, stiction_target_vels,
        raw_gravity, static_frictions,
        friction_error_epsilon, friction_velocity_epsilon, active_imp)
    : Vec6{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  Vec6 f_vec{{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
  if (friction_scale_applied != 0.0) {
    for (int i = 0; i < 6; ++i) {
      f_vec[i] = friction_scale_applied * raw_friction[i];
    }
  }

  MitTarget out;
  out.velocity = command.velocity;
  out.kp = effective_kp;
  out.kd = resolved_kd;
  for (int i = 0; i < 6; ++i) {
    const double q = command.position[i];
    const double g = g_vec[i];
    const double f = f_vec[i];
    const double kp_i = effective_kp[i];
    double offset = 0.0;
    double residual = g;
    if (active_imp[i]) {
      const double lo = std::max(-MAX_GRAVITY_OFFSET, -MIT_POSITION_LIMIT - q);
      const double hi = std::min(MAX_GRAVITY_OFFSET, MIT_POSITION_LIMIT - q);
      const double raw = g / kp_i;
      offset = std::clamp(raw, lo, hi);
      residual = (offset == raw) ? 0.0 : (g - offset * kp_i);
    }
    out.gravity_offset[i] = offset;
    out.position[i] = q + offset;
    out.torque[i] = command.torque[i] + residual + f;
  }
  arm_.mit(out);
  comp_rate.mark(now);
  mit_tx_rate.mark(now);
  cmd_tx_rate.mark(now);
  last_mit_command_ = command;
  gravity = g_vec;
  friction = f_vec;
  output_torque = out.torque;

  if (compute_wrench) {
    bool any_acc = false;
    for (double a : joint_acceleration) {
      if (a != 0.0) {
        any_acc = true;
        break;
      }
    }
    if (any_acc) {
      const std::string eff_name = effector.has_value() ? effector->model : "teaching_handle";
      const Vec6 inertias = gravity_model.has_value()
        ? gravity_model->inertias(q_meas, eff_name, eff_mass, eff_com)
        : joint_inertias(q_meas, eff_name, eff_mass, eff_com);
      for (int i = 0; i < 6; ++i) {
        inertia_torque[i] = inertias[i] * joint_acceleration[i];
      }
    } else {
      inertia_torque = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
    }
    update_wrench_estimate(
      command, sample, effective_kp, resolved_kd, velocity,
      raw_gravity, g_vec, raw_friction, f_vec);
  }

  const bool external_grip_active =
    gripper_received_.has_value() && (0.0 <= now - *gripper_received_) &&
    (now - *gripper_received_ <= 0.2);
  if (!external_grip_active) {
    if (!mit_target.has_value() && !cartesian_mit_target.has_value()) {
      apply_gripper((*sample.position)[6], 0.0, sample, false);
    } else if (gripper_assist_active_) {
      arm_.grip((*sample.position)[6], 0.0);
      gripper_friction = 0.0;
      gripper_assist_active_ = false;
    }
  }
}

void Controller::apply_gripper(
  double width_cmd, double effort_cmd, const Feedback & sample, bool explicit_cmd)
{
  double width = width_cmd;
  double effort = effort_cmd;
  const bool was_assist = gripper_assist_active_;
  if (friction_model.has_value() && friction_model->gripper.has_value() &&
      friction_scale_applied > 0.0 && sample.position.has_value())
  {
    std::tie(width, effort, gripper_friction) =
      friction_model->gripper->compensate_command(
        width, effort, (*sample.position)[6], sample.gripper_velocity, friction_scale_applied);
    gripper_assist_active_ = (effort_cmd <= 0.0 && effort > 0.0);
  } else {
    gripper_friction = 0.0;
    gripper_assist_active_ = false;
  }
  if (effort <= 0.0 && !was_assist &&
      (!explicit_cmd || (arm_.is_real_hardware() && !sample.enabled[6])))
  {
    return;
  }
  arm_.grip(width, effort, !gripper_assist_active_);
  grip_tx_rate.mark(clock_());
}

void Controller::gripper(double width, double effort) {
  if (!std::isfinite(width) || !std::isfinite(effort)) {
    throw std::invalid_argument("gripper width and effort must be finite");
  }
  const double clamped_effort = std::clamp(effort, 0.0, 5.0);
  const Feedback sample = healthy(false);
  gripper_received_ = clock_();
  apply_gripper(width, clamped_effort, sample, true);
}

Feedback Controller::watchdog(bool compute_wrench) {
  if (mode == "mit") {
    std::optional<Feedback> sample;
    try {
      sample = healthy();
      const double now = clock_();
      MitTarget command;
      const bool fresh_rx = mit_received.has_value() && (now - *mit_received <= 0.2);
      if (fresh_rx && cartesian_mit_target.has_value()) {
        command = compute_cartesian_mit(*cartesian_mit_target, *sample);
      } else if (fresh_rx && mit_target.has_value()) {
        command = *mit_target;
      } else {
        if (mit_target.has_value() || cartesian_mit_target.has_value()) {
          mit_target.reset();
          cartesian_mit_target.reset();
          arm_.grip((*sample->position)[6], 0.0);
        }
        command = gravity_command(*sample);
      }
      const bool ready = (sample->status->ctrl_mode == 1 && sample->status->mode_feedback == 6);
      if (!ready) {
        if (sample->status->ctrl_mode == 0 || sample->status->ctrl_mode == 2) {
          if (sample->status->teach_status == 1) {
            arm_.mode("hold");
            return *sample;
          }
          send_mit(command, *sample, compute_wrench);
          arm_.mode("mit");
          hold_reason.clear();
          return *sample;
        }
        if (mit_ready || now - mit_started > 0.2) {
          throw std::invalid_argument("Arm did not acknowledge or left external MIT mode");
        }
        return *sample;
      }
      mit_ready = true;
      send_mit(command, *sample, compute_wrench);
      hold_reason.clear();
    } catch (const std::exception & error) {
      hold_reason = error.what();
      try {
        arm_.retain_mit();
        const double now_ret = clock_();
        mit_tx_rate.mark(now_ret);
        cmd_tx_rate.mark(now_ret);
        if (compute_wrench && sample.has_value() && sample->position.has_value() &&
            last_mit_command_.has_value())
        {
          inertia_torque = {{0.0, 0.0, 0.0, 0.0, 0.0, 0.0}};
          std::optional<double> eff_mass;
          std::optional<Vec3> eff_com;
          if (effector.has_value()) {
            eff_mass = effector->mass_kg;
            eff_com = effector->com_m;
          }
          Vec6 q_meas{};
          for (int i = 0; i < 6; ++i) q_meas[i] = (*sample->position)[i];
          const Vec6 raw_g = gravity_model.has_value()
            ? gravity_model->torques(q_meas, eff_mass, eff_com)
            : gravity_torques(q_meas, eff_mass, eff_com);
          const Vec6 vel = sample->has_velocity_torque
            ? sample->velocity
            : last_mit_command_->velocity;
          update_wrench_estimate(
            *last_mit_command_, *sample, applied_kp, applied_kd, vel,
            raw_g, gravity, std::nullopt, std::nullopt);
        }
      } catch (const std::exception & send_error) {
        throw std::invalid_argument(
          std::string(error.what()) + "; last MIT output retained locally; retransmission failed: " +
          send_error.what() + "; no disable requested");
      }
      throw std::invalid_argument(
        std::string(error.what()) + "; retaining last MIT output; no disable requested");
    }
    return *sample;
  }
  return arm_.snapshot();
}

}  // namespace piper_x_driver

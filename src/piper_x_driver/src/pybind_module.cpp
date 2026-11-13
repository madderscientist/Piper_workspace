#include <pybind11/numpy.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "piper_x_driver/can_protocol.hpp"
#include "piper_x_driver/controller.hpp"
#include "piper_x_driver/friction.hpp"
#include "piper_x_driver/gravity.hpp"
#include "piper_x_driver/terminal.hpp"

namespace py = pybind11;
using namespace piper_x_driver;

namespace {

Vec6 seq_to_vec6(const py::sequence & seq, const char * name) {
  if (py::len(seq) < 6) {
    throw py::value_error(std::string(name) + " requires six values");
  }
  Vec6 out{};
  for (size_t i = 0; i < 6; ++i) {
    out[i] = py::cast<double>(seq[i]);
  }
  return out;
}

Vec3 seq_to_vec3(const py::sequence & seq, const char * name) {
  if (py::len(seq) < 3) {
    throw py::value_error(std::string(name) + " requires three values");
  }
  Vec3 out{};
  for (size_t i = 0; i < 3; ++i) {
    out[i] = py::cast<double>(seq[i]);
  }
  return out;
}

py::tuple vec6_to_tuple(const Vec6 & v) {
  return py::make_tuple(v[0], v[1], v[2], v[3], v[4], v[5]);
}

py::tuple vec3_to_tuple(const Vec3 & v) {
  return py::make_tuple(v[0], v[1], v[2]);
}

py::array_t<double> mat3_to_numpy(const Mat3 & m) {
  py::array_t<double> arr({3, 3});
  auto r = arr.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < 3; ++i) {
    for (py::ssize_t j = 0; j < 3; ++j) {
      r(i, j) = m[i][j];
    }
  }
  return arr;
}

py::array_t<double> mat6_to_numpy(const Mat6 & m) {
  py::array_t<double> arr({6, 6});
  auto r = arr.mutable_unchecked<2>();
  for (py::ssize_t i = 0; i < 6; ++i) {
    for (py::ssize_t j = 0; j < 6; ++j) {
      r(i, j) = m[i][j];
    }
  }
  return arr;
}

py::dict frame_to_dict(const RawCanFrame & f) {
  py::dict d;
  d["can_id"] = f.can_id;
  d["dlc"] = f.dlc;
  d["data"] = py::bytes(reinterpret_cast<const char *>(f.data.data()), f.dlc);
  return d;
}

}  // namespace

PYBIND11_MODULE(_core, m) {
  m.doc() = "Native C++ kinematics, gravity/friction dynamics, and CAN protocol core for PiPER-X.";

  m.def(
    "fast_fk",
    [](const py::sequence & position, py::object effector_com) {
      const Vec6 q = seq_to_vec6(position, "position");
      std::optional<Vec3> ec;
      if (!effector_com.is_none()) {
        ec = seq_to_vec3(effector_com.cast<py::sequence>(), "effector_com");
      }
      const FkResult res = fast_fk(q, ec);
      py::list origins, axes, centers, rots;
      for (int i = 0; i < 6; ++i) {
        origins.append(vec3_to_tuple(res.origins[i]));
        axes.append(vec3_to_tuple(res.axes[i]));
      }
      for (int i = 0; i < 7; ++i) {
        centers.append(vec3_to_tuple(res.centers[i]));
        rots.append(py::make_tuple(
          vec3_to_tuple(res.rots[i][0]),
          vec3_to_tuple(res.rots[i][1]),
          vec3_to_tuple(res.rots[i][2])));
      }
      return py::make_tuple(origins, axes, centers, rots);
    },
    py::arg("position"), py::arg("effector_com") = py::none());

  m.def(
    "link6_rotation_in_base",
    [](const py::sequence & position) {
      return mat3_to_numpy(link6_rotation_in_base(seq_to_vec6(position, "position")));
    },
    py::arg("position"));

  m.def(
    "inverse_kinematics",
    [](const py::sequence & target_position,
       const py::sequence & target_rotation,
       const py::sequence & q_seed,
       const py::sequence & tool_offset_m,
       py::object lower_limits,
       py::object upper_limits,
       double damping,
       int max_iters,
       double pos_tol_m,
       double rot_tol_rad)
    {
      const Vec3 tp = seq_to_vec3(target_position, "target_position");
      Mat3 tr{};
      for (int r = 0; r < 3; ++r) {
        py::sequence row = target_rotation[r].cast<py::sequence>();
        for (int c = 0; c < 3; ++c) {
          tr[r][c] = py::cast<double>(row[c]);
        }
      }
      const Vec6 seed = seq_to_vec6(q_seed, "q_seed");
      const Vec3 off = seq_to_vec3(tool_offset_m, "tool_offset_m");
      std::optional<Vec6> lo, hi;
      if (!lower_limits.is_none()) {
        lo = seq_to_vec6(lower_limits.cast<py::sequence>(), "lower_limits");
      }
      if (!upper_limits.is_none()) {
        hi = seq_to_vec6(upper_limits.cast<py::sequence>(), "upper_limits");
      }
      const IkResult res = inverse_kinematics(
        tp, tr, seed, off, lo, hi, damping, max_iters, pos_tol_m, rot_tol_rad);
      py::dict out;
      out["joints"] = vec6_to_tuple(res.joints);
      out["converged"] = res.converged;
      out["iterations"] = res.iterations;
      out["position_error_m"] = res.position_error_m;
      out["orientation_error_rad"] = res.orientation_error_rad;
      return out;
    },
    py::arg("target_position"),
    py::arg("target_rotation"),
    py::arg("q_seed") = py::make_tuple(0.0, 0.8, -1.2, 0.0, 0.4, 0.0),
    py::arg("tool_offset_m") = py::make_tuple(0.0, 0.0, 0.0),
    py::arg("lower_limits") = py::none(),
    py::arg("upper_limits") = py::none(),
    py::arg("damping") = 1e-3,
    py::arg("max_iters") = 50,
    py::arg("pos_tol_m") = 1e-4,
    py::arg("rot_tol_rad") = 1e-3);

  m.def(
    "gravity_torques",
    [](const py::sequence & position, py::object mass_kg, py::object com_m) {
      const Vec6 q = seq_to_vec6(position, "position");
      std::optional<double> mk;
      if (!mass_kg.is_none()) {
        mk = py::cast<double>(mass_kg);
      }
      std::optional<Vec3> cm;
      if (!com_m.is_none()) {
        cm = seq_to_vec3(com_m.cast<py::sequence>(), "com_m");
      }
      return vec6_to_tuple(gravity_torques(q, mk, cm));
    },
    py::arg("position"), py::arg("mass_kg") = py::none(), py::arg("com_m") = py::none());

  m.def(
    "joint_inertias",
    [](const py::sequence & position,
       const std::string & end_effector,
       py::object mass_kg,
       py::object com_m,
       const py::sequence & scales,
       py::object inertia_tensor)
    {
      const Vec6 q = seq_to_vec6(position, "position");
      std::optional<double> mk;
      if (!mass_kg.is_none()) {
        mk = py::cast<double>(mass_kg);
      }
      std::optional<Vec3> cm;
      if (!com_m.is_none()) {
        cm = seq_to_vec3(com_m.cast<py::sequence>(), "com_m");
      }
      const Vec6 sc = seq_to_vec6(scales, "scales");
      std::optional<Mat3> it;
      if (!inertia_tensor.is_none()) {
        py::sequence rows = inertia_tensor.cast<py::sequence>();
        Mat3 mat{};
        for (int r = 0; r < 3; ++r) {
          py::sequence row = rows[r].cast<py::sequence>();
          for (int c = 0; c < 3; ++c) {
            mat[r][c] = py::cast<double>(row[c]);
          }
        }
        it = mat;
      }
      return vec6_to_tuple(joint_inertias(q, end_effector, mk, cm, sc, it));
    },
    py::arg("position"),
    py::arg("end_effector") = "teaching_handle",
    py::arg("mass_kg") = py::none(),
    py::arg("com_m") = py::none(),
    py::arg("scales") = py::make_tuple(1.0, 1.0, 1.0, 1.0, 1.0, 1.0),
    py::arg("inertia_tensor") = py::none());

  m.def(
    "geometric_jacobian",
    [](const py::sequence & position,
       const py::sequence & tool_offset_m,
       const std::string & frame)
    {
      return mat6_to_numpy(geometric_jacobian(
        seq_to_vec6(position, "position"),
        seq_to_vec3(tool_offset_m, "tool_offset_m"),
        frame));
    },
    py::arg("position"),
    py::arg("tool_offset_m") = py::make_tuple(0.0, 0.0, 0.0),
    py::arg("frame") = "gripper");

  m.def(
    "end_effector_wrench_raw",
    [](const py::sequence & tau,
       const py::sequence & q_eff,
       const py::sequence & tool_offset_m,
       double damping,
       const std::string & frame)
    {
      return vec6_to_tuple(end_effector_wrench(
        seq_to_vec6(tau, "joint_torques"),
        seq_to_vec6(q_eff, "position"),
        nullptr,
        seq_to_vec3(tool_offset_m, "tool_offset_m"),
        damping,
        frame));
    },
    py::arg("tau"),
    py::arg("q_eff"),
    py::arg("tool_offset_m") = py::make_tuple(0.0, 0.0, 0.0),
    py::arg("damping") = 1e-2,
    py::arg("frame") = "gripper");

  m.def(
    "max_critical_kd_raw",
    [](const py::sequence & kp,
       const std::string & end_effector,
       const py::sequence & scales,
       double added_mass_kg)
    {
      ArmGravityModel gm;
      gm.effector_model = "";
      gm.added_mass_kg = added_mass_kg;
      const Vec6 sc = seq_to_vec6(scales, "scales");
      for (int i = 0; i < 6; ++i) {
        JointGravityModel jm;
        jm.scale = sc[i];
        gm.joints[i + 1] = jm;
      }
      return vec6_to_tuple(max_critical_kd(seq_to_vec6(kp, "kp"), end_effector, &gm));
    },
    py::arg("kp"),
    py::arg("end_effector"),
    py::arg("scales"),
    py::arg("added_mass_kg"));

  m.def(
    "adaptive_stiffness_scale",
    &compute_adaptive_stiffness_scale,
    py::arg("position_error"),
    py::arg("scale"),
    py::arg("b") = 0.08,
    py::arg("power") = 3.0);

  m.def(
    "joint_friction_torque",
    [](double fc, double fs, double vs, double delta, double fv, double v0, double vr,
       double load_ratio, double error_epsilon, double velocity_epsilon,
       const std::vector<std::array<double, 5>> & harmonics_raw,
       double theta, double velocity, double position_error, double target_velocity,
       double gravity_torque, double static_friction,
       py::object err_eps_override, py::object vel_eps_override,
       bool active_impedance, double cogging_scale, double friction_scale)
    {
      JointFrictionModel jm;
      jm.fc = fc;
      jm.fs = fs;
      jm.vs = vs;
      jm.delta = delta;
      jm.fv = fv;
      jm.v0 = v0;
      jm.vr = vr;
      jm.load_ratio = load_ratio;
      jm.error_epsilon = error_epsilon;
      jm.velocity_epsilon = velocity_epsilon;
      jm.harmonics.reserve(harmonics_raw.size());
      for (const auto & h : harmonics_raw) {
        jm.harmonics.push_back(SpatialHarmonic{h[0], h[1], h[2], h[3], h[4]});
      }
      std::optional<double> ee, ve;
      if (!err_eps_override.is_none()) ee = py::cast<double>(err_eps_override);
      if (!vel_eps_override.is_none()) ve = py::cast<double>(vel_eps_override);
      return jm.torque(
        theta, velocity, position_error, target_velocity,
        gravity_torque, static_friction, ee, ve,
        active_impedance, cogging_scale, friction_scale);
    });

  // Protocol encoder bindings for golden cross-validation against pyAgxArm
  m.def(
    "encode_mit_command",
    [](const py::sequence & pos,
       const py::sequence & vel,
       const py::sequence & kp,
       const py::sequence & kd,
       const py::sequence & torque,
       const py::sequence & gravity_offset)
    {
      MitTarget cmd;
      cmd.position = seq_to_vec6(pos, "position");
      cmd.velocity = seq_to_vec6(vel, "velocity");
      cmd.kp = seq_to_vec6(kp, "kp");
      cmd.kd = seq_to_vec6(kd, "kd");
      cmd.torque = seq_to_vec6(torque, "torque");
      cmd.gravity_offset = seq_to_vec6(gravity_offset, "gravity_offset");
      const auto frames = encode_mit_command(cmd);
      py::list out;
      for (const auto & f : frames) {
        out.append(frame_to_dict(f));
      }
      return out;
    },
    py::arg("position"),
    py::arg("velocity"),
    py::arg("kp"),
    py::arg("kd"),
    py::arg("torque"),
    py::arg("gravity_offset") = py::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

  m.def(
    "encode_joint_ctrl",
    [](const py::sequence & joints_rad) {
      const auto frames = encode_joint_ctrl(seq_to_vec6(joints_rad, "joints_rad"));
      py::list out;
      for (const auto & f : frames) out.append(frame_to_dict(f));
      return out;
    });

  m.def(
    "encode_end_pose_ctrl",
    [](const py::sequence & pose) {
      const auto frames = encode_end_pose_ctrl(seq_to_vec6(pose, "pose"));
      py::list out;
      for (const auto & f : frames) out.append(frame_to_dict(f));
      return out;
    });

  m.def(
    "encode_mode_ctrl",
    [](uint8_t ctrl_mode, uint8_t move_mode, uint8_t speed_percent, uint8_t mit_mode) {
      return frame_to_dict(encode_mode_ctrl(ctrl_mode, move_mode, speed_percent, mit_mode));
    });

  m.def(
    "encode_gripper_ctrl",
    [](int32_t value, uint16_t force, uint8_t status_code, uint8_t set_zero) {
      return frame_to_dict(encode_gripper_ctrl(value, force, status_code, set_zero));
    });

  m.def(
    "encode_motor_enable",
    [](uint8_t joint_index, bool enable) {
      return frame_to_dict(encode_motor_enable(joint_index, enable));
    });

  m.def(
    "encode_joint_move_batch",
    [](const py::sequence & joints_rad, uint8_t speed_percent) {
      const auto frames = encode_joint_move_batch(
        seq_to_vec6(joints_rad, "joints_rad"), speed_percent);
      py::list out;
      for (const auto & f : frames) out.append(frame_to_dict(f));
      return out;
    });

  m.def(
    "encode_cartesian_move_batch",
    [](const py::sequence & pose, uint8_t speed_percent, bool linear, uint8_t cached_mit_mode) {
      const auto frames = encode_cartesian_move_batch(
        seq_to_vec6(pose, "pose"), speed_percent, linear, cached_mit_mode);
      py::list out;
      for (const auto & f : frames) out.append(frame_to_dict(f));
      return out;
    },
    py::arg("pose"),
    py::arg("speed_percent"),
    py::arg("linear") = false,
    py::arg("cached_mit_mode") = 0);

}

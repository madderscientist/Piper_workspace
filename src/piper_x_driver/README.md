# PiPER-X 四臂驱动 (`piper_x_driver`)

`piper_x_driver` 是工作空间唯一的机械臂底层硬件驱动包与公共运动学/动力学底座，采用 **原生 C++17 (`rclcpp` + Linux SocketCAN) 实时驱动 + 无状态 C++/Python 共享数学库** 分层设计，不依赖 `pyAgxArm` 或 `python-can`，也不反向耦合任何上层业务包。

> **硬件频率上限说明**：PiPER-X 机械臂固件（`S-V1.9-0` / `S-V1.9-1`）的 CAN 状态与关节位置/速度上报频率写死为 **`200 Hz`（周期 `5 ms`）**，CAN 协议中没有任何改频指令，且受限于 `1 Mbps` CAN 2.0A 总线物理带宽（单臂仅 `200 Hz` 状态上报已达 `~3040 帧/秒`、占总线约 `39%` 容量），**无法通过软件配置将底层有效反馈与控制频率改大**。

因此，本工程以硬件 `200 Hz` 上限为基准，采用 **200 Hz（驱动 + 示教）/ 50 Hz（上层控制与状态汇总）分层控制架构**：

- **200 Hz 底层驱动层（`rate_hz=200.0`，周期 `5 ms`）**：1:1 对齐固件 200 Hz CAN 反馈，每周期完成 MIT 看门狗、重力补偿 $g(q)$、关节与夹爪摩擦力补偿 $f(q, \dot{q})$、CAN MIT 帧下发，并以 **200 Hz** 发布 `joint_states_feedback`；关节末端力估计默认关闭，启用后同频计算并发布 `end_wrench_stamped`。
- **200 Hz 示教耦合层（`piper_teaching`，`rate_hz=200.0`，周期 `5 ms`）**：主从/双向示教节点与驱动同频运行，以 **200 Hz** 订阅关节反馈并发送 `command/mit`、`command/gripper` 与参考轨迹 `target`。
- **50 Hz 上层控制与状态汇总（周期 `20 ms`）**：驱动以 **50 Hz** 发布 `control_state` JSON（含各层实时频率 `rates`）、`arm_status` 与 `end_pose_stamped`；上层策略/规划节点以 **50 Hz** 发送目标时，驱动在相邻两帧目标之间仍以 **200 Hz** 紧跟实测反馈高频刷新重力与摩擦力补偿。

## 0. 架构设计与外部包复用指南

### 0.1 模块结构与职责边界

| 层级 | 模块 | 核心类 / 函数 | 职责边界 |
|---|---|---|---|
| **C++ 实时驱动层** | [`src/socketcan_arm.cpp`](src/socketcan_arm.cpp)<br/>[`src/can_protocol.cpp`](src/can_protocol.cpp) | `SocketCanArm`, `encode_mit_command` | **原生 Linux SocketCAN 硬件与协议层**。使用内核 `CAN_RAW_FILTER` 白名单、`recvmmsg` + `SO_TIMESTAMPNS` 纳秒级硬件时间戳接收解码、`sendmmsg` 单次系统调用批量下发 6 轴 MIT 帧，管理固件校验（`0x4AF`）、使能、200 Hz 夹爪插值与零点标定（`0x476`）。 |
| **C++ 实时控制层** | [`src/controller.cpp`](src/controller.cpp)<br/>[`src/node.cpp`](src/node.cpp)<br/>[`src/main.cpp`](src/main.cpp) | `Controller`, `DriverNode` | **4 独立原生 `rclcpp` 驱动进程**。每臂独立进程隔离故障，以 200 Hz 执行看门狗、误差自适应刚度 $s_a$、重力两级守恒分配、关节/夹爪摩擦补偿与末端六维力估计，对外暴露 ROS 2 话题与服务。 |
| **C++ 共享核心库** | [`include/piper_x_driver/*.hpp`](include/piper_x_driver)<br/>[`src/gravity.cpp`](src/gravity.cpp)<br/>[`src/friction.cpp`](src/friction.cpp)<br/>[`src/configuration.cpp`](src/configuration.cpp) | `piper_x_driver_core` | **零 ROS 节点耦合的 C++17 静态核心库**。单向依赖更底层的 `piper_description` 包，启动时直接解析 `piper_description/urdf/piper_no_gripper_description.urdf` 获取 6 轴关节原点/RPY 与连杆质量、质心、3×3 惯性张量，统一提供正运动学、连杆姿态惯量、雅可比、外力螺旋、重力/摩擦模型与 `arms.yaml` 解析。 |
| **Python 公共绑定层** | [`piper_x_driver/__init__.py`](piper_x_driver/__init__.py)<br/>[`src/pybind_module.cpp`](src/pybind_module.cpp) | `_core`, `gravity.py`, `friction.py`, `configuration.py`, `protocol.py` | **供上层 Python 包单向 `import` 的无状态库**。导入时不打开任何硬件或 ROS 上下文；计算密集函数自动路由至 `pybind11` 编译的 `_core` C++ 模块，保证全工作空间 C++ 与 Python 共享唯一一份公式真值。 |

### 0.2 外部 Python 包可以 `import` 哪些接口、怎么用？

所有上层 Python 包（如 `insert`、`piper_teaching`、`piper_gravity`、`piper_friction`、`state_dashboard`、`paxini_driver`、`wrist_camera`）统一从顶层 `piper_x_driver`（或对应子模块）导入纯函数与数据模型：

1. **正/逆运动学（FK / IK）、旋转矩阵与几何雅可比（`piper_x_driver` / `piper_x_driver.gravity`）**

```python
from piper_x_driver import (
    IkResult,
    forward_kinematics,
    inverse_kinematics,
    link6_rotation_in_base,
    geometric_jacobian,
)

q = (0.0, 0.8, -1.2, 0.1, 0.3, -0.2)  # 6 轴关节角 (rad)

# 1) 计算 base_link -> link6（或加 link6 系下工具偏移 [x, y, z] m）的 4x4 齐次矩阵
T_base_link6 = forward_kinematics(q)
T_base_tip = forward_kinematics(q, tool_offset_m=(0.0, 0.0, 0.16))

# 2) 6-DOF 逆运动学：目标可传 4x4 齐次矩阵或 6D [x, y, z, roll, pitch, yaw]，自动遵守 URDF 关节限位
q_ik = inverse_kinematics(T_base_tip, q_seed=q, tool_offset_m=(0.0, 0.0, 0.16))
res = inverse_kinematics((0.28, 0.0, 0.22, 0.0, 1.4, 0.0), q_seed=q, return_result=True)  # -> IkResult(joints, converged, ...)

# 3) 仅取 base_link -> link6 的 3x3 旋转矩阵
R_base_link6 = link6_rotation_in_base(q)

# 4) 计算 6x6 几何雅可比矩阵（frame 可选 "gripper" 夹爪系 或 "base_link" 基座系）
J_grip = geometric_jacobian(q, tool_offset_m=(0.0, 0.0, 0.16), frame="gripper")
```

2. **标定重力模型、惯量、临界阻尼与末端六维力求解**

```python
from piper_x_driver import (
    ArmGravityModel,
    load_gravity_model,
    gravity_torques,
    joint_inertias,
    max_critical_kd,
    end_effector_wrench,
)

# 未标定 CAD 名义重力矩与临界阻尼
tau_cad = gravity_torques(q, mass_kg=0.685, com_m=(0.0, 0.0, 0.05))
kd_auto = max_critical_kd((17.0, 16.0, 15.0, 4.0, 3.0, 3.0), end_effector="gripper_with_wrench")

# 加载标定重力模型（自动应用各关节 angle_offset、scale 与末端等效附加质量 added_mass_kg）
g_model = load_gravity_model("src/piper_gravity/config/arm_3_gravity.yaml")
T_cal = g_model.forward_kinematics(q, tool_offset_m=(0.0, 0.0, 0.16))
q_cal = g_model.inverse_kinematics(T_cal, q_seed=q, tool_offset_m=(0.0, 0.0, 0.16))
J_cal = g_model.jacobian(q, frame="gripper")
tau_g = g_model.torques(q)
wrench = end_effector_wrench(tau_g, q, gravity_model=g_model, frame="gripper")
```

3. **关节/夹爪摩擦模型与四臂配置（`FleetConfig`）及协议常量**

```python
from piper_x_driver import (
    FleetConfig,
    default_config_path,
    load_friction_model,
    MIT_GAIN_SCALE,
    MIT_KP_LSB,
    MIT_KD_LSB,
    MIT_POSITION_LIMIT,
    MODES,
    RateMeter,
    feedback_problem,
)

fleet = FleetConfig(default_config_path())
arm4 = fleet.arm("arm_4")                      # 读取 can_port / namespace / kp / kd / s_a 等
f_model = fleet.friction_model(arm4)           # 读取已配置的 ArmFrictionModel
tau_f = f_model.torques(q, (0.05,) * 6)        # 计算 6 轴摩擦力矩 (Nm)
```

### 0.3 外部 C++ 包如何复用？

本包已通过 `ament_cmake` 导出头文件与 `piper_x_driver_core` 库。外部 C++ 包（例如 `paxini_driver`）只需在 `package.xml` 声明 `<depend>piper_x_driver</depend>`，在 `CMakeLists.txt` 中 `find_package(piper_x_driver REQUIRED)` 并加入 `ament_target_dependencies(... piper_x_driver)`，即可直接 `#include "piper_x_driver/gravity.hpp"` 调用 `piper_x_driver::fast_fk`、`piper_x_driver::inverse_kinematics`、`link6_rotation_in_base`、`gravity_torques` 等零开销 C++ 函数。

## 1. 启动与配置

[config/arms.yaml](config/arms.yaml) 定义四臂的 CAN 端口、命名空间、末端选择、补偿系数、标定文件路径和默认 `debug_kp / debug_kd`。三种末端 `teaching_handle / gripper / gripper_with_wrench` 的模型、质量属性、来源及自动阻尼所需的最大关节惯量统一保存在 [config/effectors.yaml](config/effectors.yaml)，通过同一加载和合成逻辑得到总质量、质心与惯性张量。旧末端保留原 CAD 聚合参数，带传感器的末端保留各部件的均匀密度 mesh 参数；运行时保存机械臂配置不会把这些参数复制回 `arms.yaml`。旧版内嵌末端参数的配置仍可读取。

arm3、arm4 默认使用 `gripper_with_wrench`，加载 `piper_description_with_wrench.urdf`，名义总质量为 0.685505 kg。各部件参数已变换到驱动原 `link6` 坐标系。重力标定文件中的 `added_mass_kg` 只增加夹爪底座的质量，用于等效表示未建模的相机等附件；驱动同时更新重力、惯量和自动阻尼计算，配置遥测中的 `gravity_payload` 提供应用后的总质量、质心和惯性张量。

```bash
# 启动四臂驱动（启动后自动退出可能残留的原生 Leader 模式、使能六关节并进入 MIT 补偿模式）
ros2 launch piper_x_driver four_arms.launch.py
# 可选：自定义驱动频率（默认 200.0 Hz）
ros2 launch piper_x_driver four_arms.launch.py rate_hz:=200.0

# 一键将四臂彻底断电失能
ros2 launch piper_x_driver power_off.launch.py
```

- **启动行为**：连接 CAN 后自动发送 `0x470: 0xFC` 恢复 200 Hz 周期反馈，使能六关节（夹爪零力释放），以当前实测角度为初始目标进入外部 `mit` 模式。
- **退出行为**：`Ctrl+C` 停止驱动节点时，底层自动切入固件原生零力拖动模式（`0x470: 0xFA` + `0x191: mode=0`）。
- **无外部指令回退**：当上层节点（如 `piper_teaching` 或 Dashboard MIT 调试滑块）停止发送 `command/mit` 超过 `0.2 s` 时，驱动保持 `mit` 模式，自动以当前实测位置 `sample.position[:6]` 和预设 `debug_kp / debug_kd` 维持重力与摩擦力补偿。

### 1.1 固件 CAN 上报频率约束与分层频率设置

1. **固件 CAN 反馈频率固定为 `200 Hz`（`5 ms`），不可通过软件改大**：
   - **协议无改频指令**：在 `pyAgxArm`（`V189` / 固件 `S-V1.9-0`、`S-V1.9-1`）的全部配置指令（`0x150`、`0x151`、`0x181..0x186`、`0x470..0x47D`、`0x487`）中，没有任何寄存器可修改关节位置（`0x2A5..0x2A7`）或电机高速状态（`0x251..0x256`）的上报周期，`200 Hz` 是主控板 MCU 固件固定的定时推送频率（低速驱动器状态 `0x261..0x266` 约为 `40 Hz`）。
   - **`1 Mbps` CAN 2.0A 物理带宽上限**：标准 8 字节 CAN 2.0A 报文单帧约占 `128 bit`（`0.128 ms`），整条总线物理极限仅约 `7500 ~ 7800 帧/秒`。在 `200 Hz` 下，单臂每周期主动上报 **14 帧高频报文**（3 帧关节角 `0x2A5..0x2A7` + 6 帧高速电机 `0x251..0x256` + 3 帧法兰位姿 `0x2A2..0x2A4` + 1 帧整臂状态 `0x2A1` + 1 帧夹爪 `0x2A8`）加低速驱动帧，**仅状态上报已达约 `3040 帧/秒`（占去约 39% 总线容量）**。
   - **有效控制信息率上限**：由于实测位置 $q$ 与实测速度 $\dot{q}$ 每 `5 ms`（`200 Hz`）才在 CAN 总线上更新一次，主机侧补偿解算与 CAN 下发的有效物理信息上限即为 **`200 Hz`**（两个 `5 ms` CAN 周期之间的刚度/阻尼闭环由电机驱动板内部的 **kHz 级硬件 PD + FOC 环** 自动维持）。

2. **全栈各层工作频率设置（`200 Hz` / `50 Hz`）**：

| 层级 | 默认频率（周期） | 参数 / 配置项 | 包含的计算与通信内容 |
|---|---|---|---|
| **电机驱动板硬件环** | **`> 10 kHz`** | 驱动器硬件内部闭环 | 电机侧高频执行 $K_p(p_{\text{des}} - q) + K_d(v_{\text{des}} - \dot{q}) + t_{\text{ff}}$ |
| **CAN 硬件状态上报** | **`200 Hz`** (`5 ms`) | 固件固定（`can_rx_hz`） | `0x2A5..0x2A7` 六轴角度、`0x251..0x256` 转速/电流、`0x2A8` 夹爪、`0x2A1` 状态、`0x2A2..0x2A4` 位姿（`0x261..0x266` 驱动状态约 `40 Hz`） |
| **底层驱动内环与高频遥测** (`piper_x_driver`) | **`200 Hz`** (`5 ms`) | `rate_hz:=200.0`<br/>`telemetry_hz:=200.0` | 重力补偿 $g(q)$、关节/夹爪摩擦补偿 $f(q,\dot{q})$、按需启用的末端六维力估计、CAN MIT 下发（`0x15A..0x15F`），以及发布 `joint_states_feedback`（启用估计后发布 `end_wrench_stamped`） |
| **四臂示教耦合层** (`piper_teaching`) | **`200 Hz`** (`5 ms`) | `rate_hz:=200.0` | 与驱动 1:1 同频订阅 `joint_states_feedback`，解算并下发 `command/mit`、`command/gripper` 与 `/bilateral_teaching/arm_{1..4}/target` |
| **上层控制与状态汇总层** | **`50 Hz`** (`20 ms`) | 驱动内置节流 / 上层节点 | 驱动以 **50 Hz**（及状态跳变时立即）发布 `control_state` JSON（含 `rates` 实时频率字典）、`arm_status` 与 `end_pose_stamped`；上层算法以 **50 Hz** 发送目标时，驱动在每两帧目标之间仍以 **200 Hz** 紧跟实测反馈实时更新重力与摩擦补偿 |

---

## 2. MIT 控制律与物理尺度约定

电机驱动器底层的 MIT 阻抗控制律为：

$$
\tau_{\text{joint}, i} = K_{p, i} \cdot (p_{\text{des}, i} - q_i) + K_{d, i} \cdot (v_{\text{des}, i} - \dot{q}_i) + t_{\text{ff}, i}
$$

- **统一使用减速后关节侧物理量**：
  上层所有接口（ROS `command/mit`、`debug_kp / debug_kd`、重力矩 $g_i$、摩擦力矩 $f_i$）均统一定义为**减速器输出端（关节侧）**的物理量，单位为 $\text{rad}$、$\text{rad/s}$、$\text{N}\cdot\text{m}$、$\text{N}\cdot\text{m/rad}$、$\text{N}\cdot\text{m/(rad/s)}$。
- **底层减速比自动换算**：
  CAN 报文的 `t_ff` 按关节力矩常数换算为关节侧 $\text{N}\cdot\text{m}$，但 `kp` 与 `kd` 在电机驱动器内部直接作用于电机侧。驱动在 [src/can_protocol.cpp](src/can_protocol.cpp) 的 `encode_mit_command` 中自动将关节侧 `kp`、`kd` 除以各轴放大系数 `MIT_GAIN_SCALE = (24.75, 24.75, 28.41, 2.10, 1.36, 1.0)` 换算为 CAN 线侧值。
- **阻尼推导工具 (`damping`) 与负数阻尼自动推导 (`kd < 0`)**：
  [gravity.py](piper_x_driver/gravity.py) 提供 `max_critical_kd(kp, end_effector, gravity_model)`，基于 URDF 全姿态最大对角转动惯量包络 $M_{ii,\max}$ 按 $k_{d,i} = \sqrt{2 M_{ii,\max} k_{p,i}}$ 计算各轴阻尼：
  - **命令行工具**：可通过 `ros2 run piper_x_driver damping --kp 60 60 60 12 8 8 --effector teaching_handle` 直接算出 `kd: [9.25, 9.31, 4.51, 0.6, 0.49, 0.13]` 填入配置文件；
  - **运行时负数自动推导**：当上层 `MitCommand` 或配置文件将某轴 `kd` 传为负数（如 `-1.0`）时，驱动层 `Controller` 自动根据当前机械臂末端负载与 `kp` 推导出对应阻尼下发。

---

## 3. 重力补偿及其分配机制

### 3.1 静态重力矩与标定模型计算 ([gravity.py](piper_x_driver/gravity.py))

在每个控制周期，驱动根据当前六轴实测位置 $q_{1..6}$ 与当前配置的末端质量/质心（示教手柄、标准夹爪或带 PX6D-75A 的夹爪）计算静态重力支撑力矩。若在 [config/arms.yaml](config/arms.yaml) 中配置了 `gravity_config`（如 `src/piper_gravity/config/arm_1_gravity.yaml`），驱动通过 `ArmGravityModel.torques(q)` 应用各关节标定比例因子 $\text{scale}_i$ 与零位偏置 $\text{angle\_offset}_i$，并乘以平滑过渡的重力比例系数 $\alpha_g \in [0, 1]$（运行时修改 `gravity_scale` 按最大 $1.0\text{ s}^{-1}$ 斜率渐变）：

$$
g_i(q) = \alpha_g \cdot \tau_{\text{grav}, i}^{\text{cal}}(q, \text{effector})
$$

此外，`gravity.py` 内置了 URDF 全连杆与末端执行器的 $3\times 3$ 惯性张量，支持通过 `ArmGravityModel.inertias(q)` 与 `scaled_inertia_tensors()` 按重力标定比例等比例缩放惯量而不改变张量几何形状。

### 3.2 为什么重力补偿要在“目标位置折合”与“前馈力矩偏置”之间分配？

在 MIT 控制律下，提供静态支撑力矩 $g_i$ 有两种途径：
1. **折合到目标位置（Position-Folded）**：令 $p_{\text{des}, i} = q_{\text{cmd}, i} + g_i / k_{p, i}$，由驱动器内部高频位置环 $K_{p, i}(p_{\text{des}, i} - q_i)$ 产生重力支撑。该方式在通信间隔内具有负反馈自稳性，且 CAN 16-bit 位置分辨率极高（$0.000381\text{ rad}$）。
2. **加到前馈力矩偏置（Torque-Biased）**：直接令 $t_{\text{ff}, i} = \tau_{\text{cmd}, i} + g_i$。

若只采用纯位置折合，当 $k_{p, i}$ 很小时，$g_i / k_{p, i}$ 会产生巨大的虚拟目标角偏置（甚至超出 CAN 位置编码范围 $\pm 12.5\text{ rad}$）；当 $k_{p, i} = 0$ 或 $k_{d, i} = 0$（如主从示教中主臂零刚度自由拖动）时，位置折合无法工作或缺少阻尼支撑。此外，CAN 报文中 `kp` 仅 12 bit，量化后的实际刚度 $K_{p, \text{realized}, i}$ 与请求值 $k_{p, i}$ 存在离散步进误差。

因此，驱动在 [src/controller.cpp](src/controller.cpp) 与 [src/can_protocol.cpp](src/can_protocol.cpp) 中采用**基于 CAN 量化精度的两级守恒分配**。

### 3.3 CAN 底层量化步长与阈值 ([include/piper_x_driver/can_protocol.hpp](include/piper_x_driver/can_protocol.hpp))

根据 CAN `0x15A`～`0x15F` 报文的位宽定义与 `MIT_GAIN_SCALE`：

| 参数 | CAN 位宽与线侧范围 | 关节侧 1 LSB 最小步长 | 分配阈值 / 约束 |
|---|---|---|---|
| `p_des` | 16-bit，`[-12.5, 12.5] rad` | `0.000381 rad` (`0.022°`) | 最大重力折合角 `MAX_GRAVITY_OFFSET = 1.0 rad`（且不越过 `±12.5 rad`） |
| `kp` | 12-bit，`[0, 500]` | `MIT_KP_LSB = [3.022, 3.022, 3.469, 0.256, 0.166, 0.122] Nm/rad` | `round(kp / MIT_KP_LSB[i]) <= 0` 时线侧 `kp` 量化为 `0` |
| `kd` | 12-bit，`[-5, 5]` | `MIT_KD_LSB = [0.0604, 0.0604, 0.0694, 0.0051, 0.0033, 0.0024] Nm/(rad/s)` | `kd < MIT_KD_LSB[i]` 时线侧阻尼不足 1 LSB |
| `t_ff` | 12-bit，`[-16c, 16c] Nm` | `[0.0078, 0.0078, 0.0078, 0.0084, 0.0079, 0.0064] Nm` | 范围 `±[16.0, 16.0, 16.0, 17.2, 16.3, 13.0] Nm` |

### 3.4 两级分配流程

#### 第一级：控制器层拆分（`Controller._send_mit`）

对每个关节 $i$，根据指令刚度 $k_{p, i}$ 与阻尼 $k_{d, i}$ 决定位置折合角 $\Delta p_i$ 与第一级未折合残差 $r_{1, i}$：

1. **零刚度或零阻尼（`kd < MIT_KD_LSB[i]` 或 `round(kp / MIT_KP_LSB[i]) <= 0`）**：
   不进行位置折合，重力 100% 分配到前馈力矩 `t_ff`（例如 `master_slave` 示教激活时的主臂 `kp=0, kd=0`）：
   $$
   \Delta p_i = 0, \qquad r_{1, i} = g_i
   $$
2. **有效刚度与阻尼（`kd >= MIT_KD_LSB[i]` 且 `round(kp / MIT_KP_LSB[i]) >= 1`）**：
   重力优先折算为位置偏置 $g_i / k_{p, i}$，并裁剪在 $\pm 1.0\text{ rad}$ 及 $\pm 12.5\text{ rad}$ 边界内；被裁剪掉的超出部分作为残差 $r_{1, i}$ 进入 `t_ff`：
   $$
   \Delta p_i = \operatorname{clip}\!\left(\frac{g_i}{k_{p, i}},\ \max(-1.0,\ -12.5 - q_{\text{cmd}, i}),\ \min(1.0,\ 12.5 - q_{\text{cmd}, i})\right)
   $$
   $$
   r_{1, i} = g_i - \Delta p_i \cdot k_{p, i}
   $$

#### 第二级：CAN 边界层 12 位 `kp` 量化校准（`SdkArm._write_mit`）

当 $\Delta p_i \neq 0$ 时，`kp` 经 12-bit 量化后实际生效的关节刚度为：

$$
K_{p, \text{realized}, i} = \text{MIT\_GAIN\_SCALE}_i \times \frac{\text{kp\_uint}_i \times 500}{4095}
$$

为消除 $K_{p, \text{realized}, i} \neq k_{p, i}$ 带来的重力欠补偿或过补偿，`_write_mit` 将位置偏置按实际量化刚度重标定为 $\Delta p_{\text{wire}, i}$，并将重标定或限幅余量 $r_{2, i}$ 补入 `t_ff`：

$$
\Delta p_{\text{wire}, i} = \operatorname{clip}\!\left(\Delta p_i \cdot \frac{k_{p, i}}{K_{p, \text{realized}, i}},\ \max(-1.0,\ -12.5 - q_{\text{cmd}, i}),\ \min(1.0,\ 12.5 - q_{\text{cmd}, i})\right)
$$

$$
r_{2, i} = \Delta p_i \cdot k_{p, i} - \Delta p_{\text{wire}, i} \cdot K_{p, \text{realized}, i}
$$

由此保证在目标位置处，**位置折合产生的力矩与前馈残差力矩之和恒等于总重力补偿力矩**：

$$
K_{p, \text{realized}, i} \cdot \Delta p_{\text{wire}, i} + (r_{1, i} + r_{2, i}) \equiv g_i
$$

---

## 4. 摩擦力补偿及其作用方式

### 4.1 计算公式与模型来源

驱动在启动或配置更新时，根据 [config/arms.yaml](config/arms.yaml) 中各臂的 `friction_config` 字段加载对应的六轴摩擦参数文件（如 `src/piper_friction/config/arm_1_friction.yaml`；若置空则不启用摩擦补偿）。

每轮控制周期，驱动读取当前六轴实测关节角 $q_{1..6}$ 与实测角速度 $\dot{q}_{1..6}$，调用 `ArmFrictionModel.torques(q, dq)` 计算六轴摩擦力矩 $f_i(q_i, \dot{q}_i)$（含 Stribeck 干摩擦、粘滞阻尼、方向相关的周期性啮合摩擦调制 $\tau_{\text{mod}}$ 以及方向无关的齿槽转矩 $\tau_{\text{cog}}$）。

- **静摩擦死区克服（Breakaway Stiction Feedforward）**：
  针对位置跟踪时的静摩擦死区 $2\tau_f / k_p$，以跟踪位置误差 $e_i = q_{\text{cmd}, i} - q_i$ 为主、阶梯保持时长还原的目标速度 $\dot{q}_{\text{target}, i}$ 为辅构造平滑破静驱动：
  $$\tau_{\text{stiction}, i} = \max\!\bigl(0,\ \mu_i |G_i| + \tau_{0, i} + \tau_{\text{mod}, i}(q_i)\bigr)\tanh\!\left(\frac{e_i}{\varepsilon_e} + \frac{\dot{q}_{\text{target}, i}}{\varepsilon_v}\right)$$
  当关节静止（$\dot{q}_i = 0$）但滞后于目标时输出平滑破静力矩，随 $|\dot{q}_i|$ 增长经 $\tanh(|\dot{q}_i| / v_{0,i})$ 平滑过渡到运动态 Stribeck + 齿槽补偿；当关节已到位（$e_i = 0, \dot{q}_{\text{target}, i} = 0$）或处于零刚度自由拖动（$k_{p,i}=0$）时严格为 $0$。
- **误差自适应刚度（Adaptive Stiffness）**：
  通过无量纲归一化幂律在小误差精细跟踪区提高刚度、大误差处平滑退回标称 $k_{p,i}$，阻尼按 $\sqrt{\cdot}$ 同步缩放以保持阻尼比恒定：
  $$k_{p, i}' = k_{p, i}\left(1 + \frac{s_a}{(|e_i| / b)^p + 1}\right), \qquad k_{d, i}' = k_{d, i}\sqrt{\frac{k_{p, i}'}{k_{p, i}}}$$
  每次周期均从指令标称增益起算（不复乘发散），且重力折合 $\Delta p_i = g_i / k_{p, i}'$ 与末端力估计均使用实际下发的 $k_{p, i}'$。可通过 `arms.yaml` 或 `ros2 param set` 运行时在线调节 `adaptive_stiffness_scale`（$s_a$，默认 `0.0`）、`adaptive_stiffness_b`（默认 `0.08 rad`）、`adaptive_stiffness_power`（默认 `3.0`）以及 `friction_error_epsilon`（默认 `0.05 rad`）、`friction_velocity_epsilon`（默认 `0.02 rad/s`）。
- **完整数学公式、物理推导与标定流程**：详见 [`piper_friction` README](../piper_friction/README.md#12-总摩擦力矩公式全景拆解)。

### 4.2 作用通道（直接叠加至 `t_ff`）

摩擦力矩 $f_i(q_i, \dot{q}_i)$ 与运动速度及空间相位强相关，**不进行目标位置折合**，而是直接叠加到 MIT 帧的前馈力矩 `t_ff` 中。

结合第 3 节的重力补偿分配，驱动每周期最终下发给 SDK `move_mit` 的各轴目标位置 $p_{\text{des}, i}$ 与前馈力矩 $t_{\text{ff}, i}$ 为：

$$
\begin{aligned}
p_{\text{des}, i} &= q_{\text{cmd}, i} + \Delta p_{\text{wire}, i} \\
t_{\text{ff}, i} &= \tau_{\text{cmd}, i} + \underbrace{(r_{1, i} + r_{2, i})}_{\text{重力未折合残差}} + \underbrace{f_i(q_i, \dot{q}_i)}_{\text{摩擦力前馈补偿}}
\end{aligned}
$$

其中 $\tau_{\text{cmd}, i}$ 为上层 `MitCommand.torque` 传入的外部期望前馈力矩（默认 `0`）。

---

## 5. 末端六轴力与力矩估计

四臂启动时可用 `wrench_estimation_enabled:=true` 显式开启全部臂；运行中可调用 `/arm_4/set_wrench_estimation`（`std_srvs/srv/SetBool`），或设置对应节点的同名 ROS 参数。Insert 控制台根据选中的力来源自动开关所选臂的估计。重力与对角惯量使用完整数值输入缓存，不对关节角取整；姿态或末端模型改变时重新计算。

关节末端力估计默认关闭（ROS 参数 `wrench_estimation_enabled=false`），重力与摩擦补偿照常执行。开启后，驱动在每个 200 Hz MIT 周期内同步估计**作用在机械臂上的六轴外力矩**与**夹爪局部坐标系下的末端六维力螺旋（三维力 + 三维力矩）**：

1. **夹爪局部坐标系定义（`frame='gripper'`）**：
   原点位于法兰/夹爪基准点，采用右手正交系（$\hat{x} \times \hat{y} = \hat{z}$）：

   - **$+Z$ 轴**：沿夹爪朝前（指向夹爪尖端 / 接近方向）
   - **$+Y$ 轴**：垂直夹爪平面朝上
   - **$+X$ 轴**：顺着夹爪向前看朝左

2. **关节外力矩估计（作用在机械臂上的外力矩 $\hat{\tau}_{\text{ext}}$）**：
   由相邻周期实测角速度差分经一阶低通滤波（时间常数 $0.04\text{ s}$，有效时间步窗口 $0.004 \le \Delta t \le 0.2\text{ s}$）得到角加速度 $\ddot{q}_i$，结合随姿态变化的标定连杆惯量 $I_i(q)$（`ArmGravityModel.inertias` / `joint_inertias`）与未补偿重力/摩擦余量，计算外部施加在各关节上的负载力矩：

   $$
   \hat{\tau}_{\text{ext}, i} = K_{p, i} \left(q_{\text{meas}, i} - q_{\text{cmd}, i}\right) + K_{d, i} \left(\dot{q}_{\text{meas}, i} - v_{\text{des}, i}\right) - \tau_{\text{cmd}, i} + I_i(q)\ddot{q}_i + (1 - \alpha_g)\tau_{\text{grav}, i}^{\text{cal}}(q) + (1 - \alpha_f)\tau_{\text{fric}, i}(q, \dot{q})
   $$

   当人为调节 `gravity_scale` 或 `friction_scale` 小于 $1.0$ 时，$(1 - \alpha_g)\tau_{\text{grav}}^{\text{cal}}$ 与 $(1 - \alpha_f)\tau_{\text{fric}}$ 自动补齐未补偿的自重与摩擦项，避免机械臂自身重量被误判为外部接触力。

3. **末端六维力螺旋映射（三维力 $\text{N}$ + 三维力矩 $\text{N}\cdot\text{m}$）**：
   在 [gravity.py](piper_x_driver/gravity.py) 中通过 `geometric_jacobian(q, tool_offset_m, frame='gripper')` 计算夹爪局部系下的 $6 \times 6$ 几何雅可比矩阵 $J(q) = [J_v(q)^T,\ J_\omega(q)^T]^T$，并用带阻尼（默认 $\lambda = 0.01$）的转置伪逆求解作用在夹爪上的末端六维力螺旋 $\hat{\mathcal{F}} = [f_x, f_y, f_z, m_x, m_y, m_z]^T$：

   $$
   \hat{\mathcal{F}} = \left(J(q) J(q)^T + \lambda^2 I_6\right)^{-1} J(q)\,\hat{\tau}_{\text{ext}}
   $$

---

## 6. 对外暴露的 ROS 2 接口与完整字段契约

每个驱动节点在对应机械臂命名空间（默认 `/arm_1`～`/arm_4`，对应 `can0`～`can3`）下暴露以下话题与服务。使能、模式与目标彼此独立（使能不自动切模式，发位置/夹爪目标不隐式切模式；底层逐字节 CAN 报文映射见 [CAN_PROTOCOL.md](CAN_PROTOCOL.md)）。

### 6.1 发布的反馈话题（关节/六维力 200 Hz，状态/位姿/JSON 50 Hz）

| 相对话题名 | 消息类型 | 发布条件与内容 |
|---|---|---|
| `joint_states_feedback` | `sensor_msgs/msg/JointState` | **200 Hz**：`name` 固定为 `joint1..joint6, gripper`；`position[7]` 为六轴 `rad` + 夹爪开度 `m`（来自 `0x2A5..0x2A8`，任一超过 `1.0 s` 则整帧不发）；高速反馈 `0x251..0x256` 均新鲜时附带 6 轴 `velocity`（`rad/s`）与 `effort`（电流估计力矩 `Nm`） |
| `end_wrench_stamped` | `geometry_msgs/msg/WrenchStamped` | **200 Hz**：`header.frame_id = gripper`（夹爪局部系：`+X` 左、`+Y` 上、`+Z` 前），在估计启用、`mode == 'mit'`、`mit_ready`、未保留过期输出且关节位置新鲜时发布作用在夹爪上的三维外力 `wrench.force.{x,y,z}`（`N`）与三维外力矩 `wrench.torque.{x,y,z}`（`Nm`） |
| `arm_status` | `piper_x_interfaces/msg/ArmStatus` | **50 Hz**（及状态跳变时立即）：来自 `0x2A1` 的 `ctrl_mode/arm_status/mode_feedback/teach_status/motion_status/trajectory_num/err_code`、六轴限位与通信异常位、`motor_enabled[6]` / `motor_enable_valid[6]`、`gripper_enabled` / `gripper_valid`、11 段 CAN 报文时效 `feedback_age[11]`（索引 `0..2` 为 `0x2A5..2A7`，`3` 为 `0x2A8` 夹爪，`4` 为 `0x2A1` 状态，`5..10` 为 `0x261..266` 驱动器，单位 `s`，缺失为 `-1`）、`teaching_active`、`last_error` |
| `end_pose_stamped` | `geometry_msgs/msg/PoseStamped` | **50 Hz**：`header.frame_id = base_link`，来自 `0x2A2..0x2A4` 的基座到法兰末端位置（`m`）与四元数姿态 |
| `control_state` | `std_msgs/msg/String` (JSON) | **50 Hz**（及状态跳变时立即）：主机控制、补偿、力估计与各层实时频率字典（完整键名见下表） |

#### `control_state` JSON 字段契约

| JSON 键 | 内容与单位 |
|---|---|
| `configuration` | 当前臂 `id/name/namespace/can_port`、`end_effector`、`gravity_scale`、`friction_scale`、`gravity_config`、`friction_config`、`debug_kp/debug_kd`、`config_file`、当前 `effector` 详情及全部可选 `effectors` |
| `mode`, `mit_ready` | 驱动内部模式（`standby/joint/point/linear/mit/drag/hold`）及外部 MIT 就绪标志（`ctrl_mode==1 && mode_feedback==6`） |
| `mit_target` | 当前活动的外部 MIT 6 轴目标角（`rad`，无外部目标或回退重力补偿时为 `null`） |
| `joint_lower`, `joint_upper` | 前端滑条推荐初始显示跨度（`rad`，非驱动命令限位） |
| `gravity_torque`, `friction_torque` | 当前 6 轴重力补偿力矩 $g_i(q)$ 与旋转关节摩擦补偿力矩 $f_i(q,\dot{q})$（`Nm`） |
| `gripper_friction`, `gripper_velocity`, `gripper_force` | 夹爪非周期摩擦与偏置补偿力（`N`）、`30 ms` 滑窗实测开合速度（`m/s`）、`0x2A8` 实测力 |
| `output_torque` | 实际随 MIT 帧提交给 SDK 的 6 轴前馈力矩 $t_{\text{ff}, i} = \tau_{\text{cmd}, i} + r_{1,i} + f_i$（`Nm`） |
| `wrench_estimation_enabled`, `wrench_estimate_available` | 估计开关及当前估计是否有效；关闭或暂不可用时，下列估计字段为 `null`，不发布 `end_wrench_stamped` |
| `inertia_torque`, `external_torque` | 6 轴连杆惯性力矩 $I_i(q)\ddot{q}_i$ 与受载外力矩估计 $\hat{\tau}_{\text{ext}, i}$（`Nm`） |
| `end_wrench`, `end_force`, `end_torque` | 夹爪局部系（`+X` 左、`+Y` 上、`+Z` 前）下的末端六维外力螺旋 `[fx, fy, fz, tx, ty, tz]`、三维力 `[fx, fy, fz]`（`N`）、三维力矩 `[tx, ty, tz]`（`Nm`） |
| `gravity_scale_applied`, `friction_scale_applied` | 按最大 `1.0/s` 斜率渐变后的当前生效重力/摩擦补偿比例（`0..1`） |
| `holding`, `hold_reason` | MIT 看门狗是否因输入/反馈/模式异常保留上一帧输出及原因说明 |
| `calibration_active`, `calibration_blocked_reason` | 末端闭合零位标定进行中标志及当前不可标零的原因说明 |
| `last_error` | 节点最近一次错误文本 |
| `rates` | 各层实时频率字典（`Hz`）：`target_hz`、`telemetry_target_hz`、`can_rx_hz`、`loop_hz`、`comp_hz`、`cmd_tx_hz`、`mit_tx_hz`、`grip_tx_hz`、`ros_cmd_hz`、`joints_pub_hz`、`wrench_pub_hz`、`pose_pub_hz`、`status_pub_hz`、`control_pub_hz` |

### 6.2 订阅的控制命令话题

| 相对话题名 | 消息类型 | QoS | 功能说明 |
|---|---|---|---|
| `command/joints` | [`JointCommand`](../piper_x_interfaces/msg/JointCommand.msg) | Reliable, Depth 10 | `position[6]`（`rad`）+ `speed_percent`（`1..100`）。`joint` 模式下发 `0x151` + `0x155..157`；`mit` 模式下仅更新 MIT 目标位置 `position[6]`（保留当前或预设 `kp/kd/velocity/torque`） |
| `command/cartesian` | [`CartesianCommand`](../piper_x_interfaces/msg/CartesianCommand.msg) | Reliable, Depth 10 | `pose[6]`（`[x,y,z,roll,pitch,yaw]`，`m/rad`）+ `speed_percent`（`1..100`）；需先通过 `set_mode` 选择 `point`（MoveP）或 `linear`（MoveL），每次下发时驱动将 `0x151`（速度与模式）与 `0x152..154` 目标帧合并批量发送 |
| `command/mit` | [`MitCommand`](../piper_x_interfaces/msg/MitCommand.msg) | Best-Effort, Depth 5 | 六轴关节侧 `position[6]`（`±12.5 rad`）、`velocity[6]`（`±45 rad/s`）、`kp[6]`（`0..500 Nm/rad`）、`kd[6]`（传入 `< 0` 如 `-1.0` 时由驱动自动推导最大临界阻尼）、`torque[6]`（额外前馈 `Nm`）；输入超时窗口 `0.2 s`，每周期驱动将 `0x151` MIT 保活帧与 `0x15A..15F` 六轴帧合并批量下发 |
| `command/gripper` | [`GripperCommand`](../piper_x_interfaces/msg/GripperCommand.msg) | Best-Effort, Depth 5 | 夹爪总开度 `width`（`0..0.1 m`）与力控参数 `effort`（`0..5`；配置了夹爪摩擦模型时自动叠加非周期摩擦补偿；`effort <= 0` 为释放或被动捏合摩擦辅助），下发 `0x159` |

### 6.3 提供的 ROS 2 服务

| 相对服务名 | 服务类型 | 请求字段 | 返回与语义说明 |
|---|---|---|---|
| `enable_srv` | [`Enable`](../piper_x_interfaces/srv/Enable.srv) | `bool enable_request` | `enable_response=true` 表示在 `3.0 s` 内收到了请求之后的新鲜使能位确认（`true` 使能六关节并保持夹爪零力释放；`false` 七轴全失能，不要求先有正常反馈） |
| `set_wrench_estimation` | `std_srvs/srv/SetBool` | `bool data` | 按臂启停关节末端力估计，只影响计算与发布；关闭时清除估计历史，不改变使能、模式或控制目标 |
| `set_mode` | [`SetMode`](../piper_x_interfaces/srv/SetMode.srv) | `string mode` | 切换驱动模式并发送对应模式请求（不阻塞等待固件到位；部分固件在收到首组 J/P/L 目标帧前不锁存 `ctrl_mode=1`）：<br/>• `standby`：`0x151.ctrl_mode=0` 待机（不等于失能）<br/>• `joint`：选择 J 模式（后续 `command/joints` 同轮连发 `0x151` + `0x155..157` 激活 `ctrl_mode=1, mode_feedback=1`）<br/>• `point`：选择 P 模式（后续 `command/cartesian` 同轮连发 `0x151` + `0x152..154` 激活 `ctrl_mode=1, mode_feedback=0`）<br/>• `linear`：选择 L 模式（后续 `command/cartesian` 同轮连发 `0x151` + `0x152..154` 激活 `ctrl_mode=1, mode_feedback=2`）<br/>• `mit`：以带完整重力/摩擦补偿的六轴 MIT 帧预装并切入 `0x151` MIT（已在 `mit` 就绪态再次调用则释放外部目标、立即回到零重力浮动）<br/>• `drag`：固件原生零力拖动（`0x470: 0xFA` + `0x191: mode=0`）<br/>• `hold`：结束原生拖动记录请求（`0x150.B2=2`） |
| `configure_arm` | [`ConfigureArm`](../piper_x_interfaces/srv/ConfigureArm.srv) | `string end_effector`, `float64 gravity_scale`, `float64 friction_scale` | 原子保存并应用末端键（`teaching_handle` / `gripper` / `gripper_with_wrench`，切换末端需七轴失能且非 MIT/示教）、`gravity_scale`（`0..1`）与 `friction_scale`（`0..1` 持久化写盘；`-1.0` 保持原值；`< -1.0` 按 `-(2.0 + s)` 临时覆盖内存摩擦比例供标定使用而不写盘） |
| `calibrate_effector` | [`CalibrateEffector`](../piper_x_interfaces/srv/CalibrateEffector.srv) | `bool confirm_closed` | 要求 `confirm_closed=true`、待机模式、七轴全失能且反馈新鲜；经 CAN `0x159` 写入夹爪零点，等待 `0x476` ACK（上限 `1.0 s`）并在 `0.5 s` 内校验新鲜零位回读（`0..0.1 mm`） |

---

## 7. 相关文档

- [CAN 层能力与逐条报文参考 (`CAN_PROTOCOL.md`)](CAN_PROTOCOL.md)
- [关节摩擦力标定与模型详解 (`piper_friction`)](../piper_friction/README.md)
- [四臂主从与双向示教 (`piper_teaching`)](../piper_teaching/README.md)
- [关节重力与静摩擦标定 (`piper_gravity`)](../piper_gravity/README.md)
- [四臂 Web 控制台 (`state_dashboard`)](../state_dashboard/README.md)

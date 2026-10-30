# PiPER-X 四臂驱动 (`piper_x_driver`)

`piper_x_driver` 是工作空间中唯一直接打开 SocketCAN 并持有 `pyAgxArm` SDK 实例的底层驱动包。每个机械臂运行一个独立驱动节点，在同一控制周期内复用同一份新鲜硬件反馈完成 **MIT 监督、重力补偿、摩擦力补偿与状态发布**。

## 1. 启动与配置

配置文件默认位于 [config/arms.yaml](config/arms.yaml)，定义四臂的 CAN 端口、命名空间、末端负载模型（`teaching_handle` / `gripper`）、重力补偿系数 `gravity_scale`、摩擦参数文件路径 `friction_config` 以及默认 `debug_kp / debug_kd`。

```bash
# 启动四臂驱动（启动后自动退出可能残留的原生 Leader 模式、使能六关节并进入 MIT 补偿模式）
ros2 launch piper_x_driver four_arms.launch.py

# 一键将四臂彻底断电失能
ros2 launch piper_x_driver power_off.launch.py
```

- **启动行为**：连接 CAN 后自动发送 `0x470: 0xFC` 恢复 200 Hz 周期反馈，使能六关节（夹爪零力释放），以当前实测角度为初始目标进入外部 `mit` 模式。
- **退出行为**：`Ctrl+C` 停止驱动节点时，底层自动切入固件原生零力拖动模式（`0x470: 0xFA` + `0x191: mode=0`）。
- **无外部指令回退**：当上层节点（如 `piper_teaching` 或 Dashboard MIT 调试滑块）停止发送 `command/mit` 超过 `0.2 s` 时，驱动保持 `mit` 模式，自动以当前实测位置 `sample.position[:6]` 和预设 `debug_kp / debug_kd` 维持重力与摩擦力补偿。

---

## 2. MIT 控制律与物理尺度约定

电机驱动器底层的 MIT 阻抗控制律为：

$$
\tau_{\text{joint}, i} = K_{p, i} \cdot (p_{\text{des}, i} - q_i) + K_{d, i} \cdot (v_{\text{des}, i} - \dot{q}_i) + t_{\text{ff}, i}
$$

- **统一使用减速后关节侧物理量**：
  上层所有接口（ROS `command/mit`、`debug_kp / debug_kd`、重力矩 $g_i$、摩擦力矩 $f_i$）均统一定义为**减速器输出端（关节侧）**的物理量，单位为 $\text{rad}$、$\text{rad/s}$、$\text{N}\cdot\text{m}$、$\text{N}\cdot\text{m/rad}$、$\text{N}\cdot\text{m/(rad/s)}$。
- **底层减速比自动换算**：
  SDK 的 `t_ff` 已在内部按关节力矩常数换算为关节侧 $\text{N}\cdot\text{m}$，但 `kp` 与 `kd` 在驱动器内部直接作用于电机侧。驱动在 [backend.py](piper_x_driver/backend.py) 的 `SdkArm._write_mit` 中自动将关节侧 `kp`、`kd` 除以各轴放大系数 `MIT_GAIN_SCALE = (24.75, 24.75, 28.41, 2.10, 1.36, 1.0)` 换算为 CAN 线侧值。
- **阻尼推导工具 (`damping`) 与负数阻尼自动推导 (`kd < 0`)**：
  [gravity.py](piper_x_driver/gravity.py) 提供 `max_critical_kd(kp, end_effector, gravity_model)`，基于 URDF 全姿态最大对角转动惯量包络 $M_{ii,\max}$ 按 $k_{d,i} = \sqrt{2 M_{ii,\max} k_{p,i}}$ 计算各轴阻尼：
  - **命令行工具**：可通过 `ros2 run piper_x_driver damping --kp 60 60 60 12 8 8 --effector teaching_handle` 直接算出 `kd: [9.25, 9.31, 4.51, 0.6, 0.49, 0.13]` 填入配置文件；
  - **运行时负数自动推导**：当上层 `MitCommand` 或配置文件将某轴 `kd` 传为负数（如 `-1.0`）时，驱动层 `Controller` 自动根据当前机械臂末端负载与 `kp` 推导出对应阻尼下发。

---

## 3. 重力补偿及其分配机制

### 3.1 静态重力矩与标定模型计算 ([gravity.py](piper_x_driver/gravity.py))

在每个控制周期，驱动根据当前六轴实测位置 $q_{1..6}$ 与当前配置的末端质量/质心（`teaching_handle` 示教手柄或 `gripper` 标准夹爪）计算静态重力支撑力矩。若在 [config/arms.yaml](config/arms.yaml) 中配置了 `gravity_config`（如 `src/piper_gravity/config/arm_1_gravity.yaml`），驱动通过 `ArmGravityModel.torques(q)` 应用各关节标定比例因子 $\text{scale}_i$ 与零位偏置 $\text{angle\_offset}_i$，并乘以平滑过渡的重力比例系数 $\alpha_g \in [0, 1]$（运行时修改 `gravity_scale` 按最大 $1.0\text{ s}^{-1}$ 斜率渐变）：

$$
g_i(q) = \alpha_g \cdot \tau_{\text{grav}, i}^{\text{cal}}(q, \text{effector})
$$

此外，`gravity.py` 内置了 URDF 全连杆与末端执行器的 $3\times 3$ 惯性张量，支持通过 `ArmGravityModel.inertias(q)` 与 `scaled_inertia_tensors()` 按重力标定比例等比例缩放惯量而不改变张量几何形状。

### 3.2 为什么重力补偿要在“目标位置折合”与“前馈力矩偏置”之间分配？

在 MIT 控制律下，提供静态支撑力矩 $g_i$ 有两种途径：
1. **折合到目标位置（Position-Folded）**：令 $p_{\text{des}, i} = q_{\text{cmd}, i} + g_i / k_{p, i}$，由驱动器内部高频位置环 $K_{p, i}(p_{\text{des}, i} - q_i)$ 产生重力支撑。该方式在通信间隔内具有负反馈自稳性，且 CAN 16-bit 位置分辨率极高（$0.000381\text{ rad}$）。
2. **加到前馈力矩偏置（Torque-Biased）**：直接令 $t_{\text{ff}, i} = \tau_{\text{cmd}, i} + g_i$。

若只采用纯位置折合，当 $k_{p, i}$ 很小时，$g_i / k_{p, i}$ 会产生巨大的虚拟目标角偏置（甚至超出 CAN 位置编码范围 $\pm 12.5\text{ rad}$）；当 $k_{p, i} = 0$ 或 $k_{d, i} = 0$（如主从示教中主臂零刚度自由拖动）时，位置折合无法工作或缺少阻尼支撑。此外，CAN 报文中 `kp` 仅 12 bit，量化后的实际刚度 $K_{p, \text{realized}, i}$ 与请求值 $k_{p, i}$ 存在离散步进误差。

因此，驱动在 [controller.py](piper_x_driver/controller.py) 与 [backend.py](piper_x_driver/backend.py) 中采用**基于 CAN 量化精度的两级守恒分配**。

### 3.3 CAN 底层量化步长与阈值 ([backend.py](piper_x_driver/backend.py))

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

## 5. 相关文档

- [控制层次与模式](../../docs/01-concepts.md)
- [ROS 已开放能力与接口](../../docs/02-ros-api.md)
- [CAN 层能力与逐条报文](../../docs/03-sdk-can.md)
- [关节摩擦力标定与模型详解 (`piper_friction`)](../piper_friction/README.md)
- [四臂主从与双向示教 (`piper_teaching`)](../piper_teaching/README.md)

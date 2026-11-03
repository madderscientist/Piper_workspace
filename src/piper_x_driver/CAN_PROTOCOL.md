# CAN 层能力与逐条报文参考

本文回答：**机械臂在 CAN 层能做什么、每条消息发什么、反馈从哪里来，以及本工程 ROS 是否开放。**
不是只列常用 SDK 函数。驱动结构与完整 ROS 消息/服务字段见 [驱动与 ROS 接口文档 (README.md)](README.md)。

## 0. 适用版本、单位约定与报文总览

- 硬件：PiPER-X；SDK 配置：`PiperFW.V189`；本工程连接时接受的固件字符串：`S-V1.9-0`、`S-V1.9-1`。
- SDK 固定提交：`d90e234ba8e408cfa964b38aa9120b3131fc9f85`，见 [依赖声明](../../requirements.txt)。
- **固件周期反馈频率固定为 `200 Hz`（`5 ms`），不可通过 CAN 指令改大**：在标准 Follower 模式（`0x470: 0xFC`）下，`0x2A1`（整臂状态）、`0x2A2..0x2A4`（末端位姿）、`0x2A5..0x2A7`（六轴角度）、`0x2A8`（夹爪状态）与 `0x251..0x256`（高速电机速度/电流）均按 **`200 Hz`** 定时推送；`0x261..0x266`（低速驱动器状态）约按 **`40 Hz`** 推送，单臂 RX 合计约 `3040 帧/秒`（约占 `1 Mbps` CAN 2.0A 总线 39% 带宽）。
- 下文 **TX = 主机发给设备，RX = 设备发给主机**。Leader 输出控制帧时，同一 ID 也可能作为主机 RX。
- `B0`～`B7` 是 CAN 数据区第 0～7 字节；`B0..3` 表示连续四字节；
  `i16/i32` 是有符号整数，`u8/u16` 是无符号整数。多字节整数均为**大端**。
- 除明确标出的固件查询和 CPV 外，下文固定 SDK 的发送帧均为 8 字节；
  TX 表中标为保留的字节由编码器填 0。MIT 是位打包，不按普通整数表解释。
- “CAN/SDK 支持”指固定版本存在消息定义、编解码或调用入口，**不等于已在本机带动力验收**。
  “ROS 已开放”只指自有 `piper_x_driver`；[官方 ROS 源码](../vendor/agx_arm_ros)
  是参考，不是默认启动的另一个接口集合。

### 单位与转换

|量|CAN 原始值的单位|SDK / 自有 ROS|
|---|---|---|
|普通关节角、末端 RPY|`i32 × 0.001°`|rad，发送时 `round(rad × 180/π × 1000)`|
|末端 XYZ、夹爪总开度|`i32 × 1e-6 m`|m，发送时 `round(m × 1e6)`|
|高速反馈速度、电流|`i16 × 0.001 rad/s`、`i16 × 0.001 A`|rad/s、A|
|角度上下限|`i16 × 0.1°`|rad|
|关节最大速度，本固定 SDK|`u16 × 0.01 rad/s`|rad/s|
|关节最大加速度|`u16 × 0.01 rad/s²`|rad/s²|
|末端最大线速度/线加速度|`u16 × 0.001 m/s`、`u16 × 0.001 m/s²`|m/s、m/s²|
|末端最大角速度/角加速度|`u16 × 0.001 rad/s`、`u16 × 0.001 rad/s²`|rad/s、rad/s²|

`0x473/0x474` 的旧消息注释仍有最大速度 `0.001 rad/s` 的说法；
这里采用固定 SDK 实际读写的 `0.01 rad/s`，不要把旧注释和当前编码混用。
编码范围不是机械允许运动范围，也不是本工程额外的软件限位。

### 实现依据

以下报文按实际编码器核对，而不是只按类注释抄表：

- [默认报文编解码器及 TX/RX 映射](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/piper/default/parser.py)
- [V188+ 编解码覆盖：MIT、IK 反馈](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/piper/versions/v188/parser.py)
- [通用 SDK 调用实现](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/piper/default/driver.py)
- [V188+ MIT 调用实现](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/piper/versions/v188/driver.py)
- [PiPER-X V189 型号覆盖](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/piper_x/versions/v189/driver.py)
- [夹爪编解码器](https://github.com/agilexrobotics/pyAgxArm/blob/d90e234ba8e408cfa964b38aa9120b3131fc9f85/pyAgxArm/protocols/can_protocol/drivers/effector/agx_gripper/default/parser.py)
- [自有 CAN 适配器](piper_x_driver/backend.py)、
  [控制语义](piper_x_driver/controller.py)、
  [ROS 节点](piper_x_driver/node.py)。

### 报文层级总览

本文按**顶层入口 → 子模式/子功能报文**组织：

|层级模块|顶层入口报文|下属子模式 / 具体报文|
|---|---|---|
|[1. 整臂状态、模式切换与电机使能](#arm-state-enable)|`0x151` 模式控制、`0x2A1` 整臂状态|[`0x471` 电机使能、`0x261`～`0x266` 驱动器反馈](#motor-enable)|
|[2. 运动模式 (`move_mode`) 与各模式报文](#move-mode)|`0x151.B1/B3` 选择 7 种运动模式|[2.1 `j`/`js` 关节报文](#mode-j-js)、[2.2 `p`/`l` 笛卡尔报文](#mode-p-l)、[2.3 `c` 圆弧报文](#mode-c)、[2.4 `mit` 报文](#mode-mit)、[2.5 `cpv` 报文](#mode-cpv)|
|[3. 急停、轨迹控制与原生示教](#motion-ctrl)|`0x150` 运动控制（急停/轨迹/拖动记录）|[3.2 原生 Leader/Follower 与回零 (`0x470`、`0x191`、`0x155`～`0x157`)](#leader-follower)|
|[4. 夹爪控制与反馈](#gripper-ctrl)|`0x159` 夹爪控制|`0x2A8` 夹爪状态与实测开度|
|[5. 设备参数查询、配置与设置确认](#device-params)|`0x477` 参数查询/恢复、`0x476` 设置确认|[5.2 关节限位/设零/清错](#joint-limits-config)、[5.3 末端限速](#end-vel-acc-limits)、[5.4 碰撞/助力等级](#crash-assistance-rating)、[5.5 夹爪/示教器参数](#gripper-pendant-params)|
|[6. 固件身份查询与主机计算边界](#firmware-and-host)|`0x4AF` 固件版本查询|主机本地计算与 CAN 报文的边界|

---

<a id="arm-state-enable"></a>

## 1. 整臂状态、模式切换与电机使能

<a id="mode-ctrl"></a>

### 1.1 TX `0x151`：模式控制 `ArmMsgModeCtrlV188`

|字节|字段|发什么|
|---|---|---|
|B0|`ctrl_mode`|`0x00` 待机；`0x01` CAN 控制；定义还接受 `0x03` 以太网、`0x04` Wi-Fi、`0x07` 离线轨迹控制|
|B1|[`move_mode`](#move-mode)|`0` P 点位；`1` J 关节；`2` L 直线；`3` C 圆弧；`5` CPV；**`6` MIT**（7 种模式总表与报文见 [第 2 节](#move-mode)）|
|B2|`move_spd_rate_ctrl`|运动速度百分比 `0..100`（SDK 缓存初值 `50`）；用于 `p / j / l / c` 位置速度规划模式，自有 ROS `command/joints` 与 `command/cartesian` 接受 `1..100`|
|B3|[`mit_mode`](#move-mode)|`0x00` 位置速度；`0xAD` MIT；`0xFF` 无效（与 `move_mode` 组合见 [第 2 节](#move-mode)）|
|B4|`residence_time`|离线轨迹点停留秒数 `0..254`；`255` 轨迹终止|
|B5|`installation_pos`|`0` 无效；`1` 水平正装；`2` 左侧装；`3` 右侧装|
|B6..7|保留|0|

SDK `set_motion_mode('j'/'p'/'l'/'c'/'mit'/'js'/'cpv')` 选择运动方式；
`mit` 使用 `B1=6, B3=0xAD`，`js` 使用 J 目标帧和 MIT 子模式，不是同一入口。
`set_speed_percent()` 仍发 `0x151`，但临时以 `B1=0xFF` 表示只更新 `move_spd_rate_ctrl`、不改运动方式，并缓存在 SDK 的 `_msg_mode` 中供后续 `0x151` 帧复用。
由于 `p / j / l / c` 的目标帧（`0x152`～`0x157`）只含位置/姿态、不含速度字段，这些模式的运动速度由 `move_spd_rate_ctrl` 控制；`mit`（帧内自带 `v_des`/`kd`）、`js`（无轨迹规划快速响应）、`cpv`（通过 `0x181`～`0x186` 设 `sp`/`vv`）则各自管理响应或速度参数。
`set_installation_pos()` 也复用该消息。字段接受某种传输模式不等于本工程实现了对应网络控制。

**ROS：部分开放。** `set_mode` 开放 standby/joint/point/linear/mit；
drag/hold 走 [第 3 节 `0x150`](#motion-ctrl)，不是新的 `0x151` 模式码。
安装方向、离线停留、圆弧、CPV、JS 没有自有 ROS 设置入口。
SDK 自动切模式被关闭；发目标不会隐式选择模式。`set_mode` 成功只表示请求已发送。

<a id="arm-status"></a>

### 1.2 RX `0x2A1`：整臂状态 `ArmMsgFeedbackStatusV188`

|字节|字段|返回什么|
|---|---|---|
|B0|`ctrl_mode`|`0` 待机、`1` CAN、`2` 示教、`3` 以太网、`4` Wi-Fi、`5` 遥控、`6` 联动示教输入、`7` 离线轨迹|
|B1|[`arm_status`](#arm-status-enum)|整臂运行/故障枚举，见下表|
|B2|[`mode_feedback`](#move-mode)|`0` P、`1` J、`2` L、`3` C、`5` CPV、**`6` MIT**|
|B3|[`teach_status`](#motion-ctrl)|`0` 关闭、`1` 开始记录、`2` 结束记录、`3` 回放、`4` 暂停、`5` 继续、`6` 终止、`7` 去起点（对应 [TX `0x150.B2`](#motion-ctrl)）|
|B4|`motion_status`|`0` 已到指定点位；`1` 未到达|
|B5|`trajectory_num`|离线轨迹点序号 `0..255`|
|B6..7|`err_code:u16`|整数 bit 8..13 是 J1..J6 角度超限；bit 0..5 是 J1..J6 通信异常，其余保留|

<a id="arm-status-enum"></a>

|`arm_status`|含义|`arm_status`|含义|
|---|---|---|---|
|`0x00`|正常|`0x08`|拖动超速|
|`0x01`|急停|`0x09`|关节状态异常|
|`0x02`|无解|`0x0A`|其他异常|
|`0x03`|奇异点|`0x0B`|示教记录|
|`0x04`|目标超限|`0x0C`|示教执行|
|`0x05`|关节通信异常|`0x0D`|示教暂停|
|`0x06`|抱闸未释放|`0x0E`|主控 NTC 过温|
|`0x07`|碰撞|`0x0F`|释放电阻 NTC 过温|

SDK：`get_arm_status()`。**ROS：已开放**于 `arm_status`，还拆出角度超限与通信异常布尔位。
此帧没有六关节使能位；不能用 `ctrl_mode` 或 `mode_feedback` 判断电机是否失能。

<a id="motor-enable"></a>

### 1.3 电机使能、失能与驱动器反馈

#### TX `0x471`：使能/失能 `ArmMsgMotorEnableDisableConfig`

|字节|发什么|
|---|---|
|B0|关节编号；本工程明确逐条发送 `1..6`|
|B1|`0x01` 失能；`0x02` 使能|
|B2..7|0|

SDK：`enable(i)` / `disable(i)`。SDK 把调用参数 `255` 转为线侧编号 `7`，
但消息注释对 `7` 是广播还是夹爪存在歧义；本工程不用这个广播入口。
夹爪通过 [`0x159`](#gripper-ctrl) 独立控制。

**ROS：已开放**于 `enable_srv`，但不是 CAN 原样透传：
一次请求操作六关节与夹爪，使能夹爪时目标为实测开度、SDK effort=0.3，不设零、不清错。
MIT 失能请求先由主机逐步释放，再发 `0x471` 与夹爪失能；这不是固件专用的“软停报文”。
成功必须收到请求之后的新鲜七路使能位确认；失能不附带发送模式切换。

#### RX `0x261`～`0x266`：低速驱动器信息 `ArmMsgFeedbackLowSpdN`

|CAN ID|关节|CAN ID|关节|CAN ID|关节|
|---|---|---|---|---|---|
|`0x261`|J1|`0x263`|J3|`0x265`|J5|
|`0x262`|J2|`0x264`|J4|`0x266`|J6|

每帧结构相同：

|字节|返回什么|
|---|---|
|B0..1|`u16 × 0.1 V`，驱动器电压|
|B2..3|`i16 × 1 °C`，驱动器温度|
|B4|`i8 × 1 °C`，电机温度|
|B5|FOC 状态：bit 0 低电压、1 电机过温、2 驱动器过流、3 驱动器过温、4 碰撞、5 驱动器错误、**6 使能**、7 堵转|
|B6..7|`u16 × 0.001 A`，母线电流；不具备采样的电机可能固定返回 0，不能视为实测零电流|

SDK：`get_driver_states(i)`。**ROS：仅部分开放**：`arm_status.motor_enabled[6]`、
`motor_enable_valid[6]`、对应 `feedback_age`。电压、温度、母线电流和逐轴完整 FOC
错误位目前没有专用自有 ROS 字段。

---

<a id="move-mode"></a>

## 2. 运动模式 (`move_mode`) 与各模式报文

SDK `set_motion_mode()` 支持以下 7 种运动模式，通过 [`0x151`](#mode-ctrl) 的 `B1 (move_mode)` 与 `B3 (mit_mode)` 组合选择：

|模式|含义|输入什么|运动特点|`0x151` 编码 (`B1` / `B3`)|对应报文章节|
|---|---|---|---|---|---|
|**`p`**|笛卡尔点到点运动|目标 `XYZ + RPY`|指定末端目标位姿，不保证末端沿直线到达|`move_mode=0, mit_mode=0x00`|[2.2 `p` / `l` 笛卡尔报文](#mode-p-l)|
|**`j`**|关节空间运动|六个关节目标角度|在关节空间运动到目标，末端轨迹一般不是直线|`move_mode=1, mit_mode=0x00`|[2.1 `j` / `js` 关节报文](#mode-j-js)|
|**`l`**|笛卡尔直线运动|目标 `XYZ + RPY`|末端位置沿直线到目标|`move_mode=2, mit_mode=0x00`|[2.2 `p` / `l` 笛卡尔报文](#mode-p-l)|
|**`c`**|笛卡尔圆弧运动|起点、中间点、终点三个位姿|用三个点确定圆弧轨迹|`move_mode=3, mit_mode=0x00`|[2.3 `c` 圆弧报文](#mode-c)|
|**`mit`**|关节 MIT 控制|每轴目标位置、目标速度、`kp`、`kd`、前馈力矩|可调刚度、阻尼，并叠加前馈力矩|`move_mode=6, mit_mode=0xAD`|[2.4 `mit` 报文](#mode-mit)|
|**`js`**|关节快速响应模式|六个关节目标角度|不做平滑和轨迹规划，快速响应关节目标|`move_mode=1, mit_mode=0xAD`|[2.1 `j` / `js` 关节报文](#mode-j-js)|
|**`cpv`**|单关节位置／速度控制|关节编号及位置或速度目标|可另设加减速度、轮廓速度和控制环参数|`move_mode=5, mit_mode=0x00`|[2.5 `cpv` 报文](#mode-cpv)|

<a id="mode-j-js"></a>

### 2.1 `j` / `js`：关节空间运动、实测位置与高速反馈

#### TX `0x155`～`0x157`：六关节位置目标

|CAN ID / 消息类|B0..3|B4..7|
|---|---|---|
|`0x155` / `ArmMsgJointCtrl12`|J1 目标角 `i32 × 0.001°`|J2 目标角，同单位|
|`0x156` / `ArmMsgJointCtrl34`|J3 目标角|J4 目标角|
|`0x157` / `ArmMsgJointCtrl56`|J5 目标角|J6 目标角|

SDK：`move_j(joints)`，JS 使用 `move_js(joints)`，都用这三帧，控制模式不同。
一条六关节目标拆成三帧，不是单帧原子更新；每帧本身不带速度、夹爪或模式字段。

**ROS：J 位置控制已开放**于 `command/joints`：
先用 `set_mode joint` 并等 `ctrl_mode=1, mode_feedback=1`；
命令的 `speed_percent` 先通过 `0x151` 发送，再发上述三帧。JS 未开放。
关节角使用设备原始值，无软件零偏、中心对齐或默认模型范围裁剪。
不过固定 SDK 在关闭型号限位后，普通关节目标仍经过通用 ±2π 裁剪；
这不同于 CAN 的 int32 毫度编码范围，不能把驱动验证通过等同于 SDK 原样发送。

#### RX `0x2A5`～`0x2A7`：六关节实测角

|CAN ID / 消息类|B0..3|B4..7|
|---|---|---|
|`0x2A5` / `ArmMsgFeedbackJointStates12`|J1 实测角 `i32 × 0.001°`|J2 实测角|
|`0x2A6` / `ArmMsgFeedbackJointStates34`|J3 实测角|J4 实测角|
|`0x2A7` / `ArmMsgFeedbackJointStates56`|J5 实测角|J6 实测角|

SDK：`get_joint_angles()`。**ROS：已开放**于 `joint_states_feedback.position[0..5]`，转换成 rad。
SDK 聚合 getter 在部分分段缺失时可能补零；自有适配器先检查三份原始缓存，
再合并夹爪反馈，避免把缺失当作真实零角度。

#### RX `0x251`～`0x256`：高速电机信息 `ArmMsgFeedbackHighSpdN`

|CAN ID|关节|CAN ID|关节|CAN ID|关节|
|---|---|---|---|---|---|
|`0x251`|J1|`0x253`|J3|`0x255`|J5|
|`0x252`|J2|`0x254`|J4|`0x256`|J6|

每帧：`B0..1 = i16 × 0.001 rad/s` 速度；
`B2..3 = i16 × 0.001 A` 电流；`B4..7 = i32 × 0.001 rad` 电机位置字段。
注意这里的位置编码不是 `0x2A5`～`0x2A7` 的毫度编码。

SDK `get_motor_states(i)` 还按 `torque = current × k × b × c` 估算力矩。
PiPER-X 系数按 J1..J6 顺序：

```text
k = [1.18125, 1.65375, 1.18125, 0.9576, 0.9576, 0.9576]
b = [4,       2.5,     4,       1.2,    0.8,    1]
c = [1,       1,       1,       1.0723, 1.016565, 0.813252]
```

**ROS：部分开放**。六帧都新鲜时发布 `joint_states_feedback.velocity[6]` 与 `effort[6]`，
否则这两个数组为空；position 仍是六关节加夹爪共 7 项。
不发布原始电流或高速位置字段，`effort` 不是已扣除重力、摩擦的外力矩。

<a id="mode-p-l"></a>

### 2.2 `p` / `l`：笛卡尔点位、直线目标与位姿反馈

#### TX `0x152`～`0x154`：基座到法兰的目标位姿

|CAN ID / 消息类|B0..3|B4..7|
|---|---|---|
|`0x152` / `ArmMsgEndPoseCtrlXY`|X，`i32 × 1e-6 m`|Y，同单位|
|`0x153` / `ArmMsgEndPoseCtrlZRX`|Z，`i32 × 1e-6 m`|Roll，`i32 × 0.001°`|
|`0x154` / `ArmMsgEndPoseCtrlRYRZ`|Pitch，`i32 × 0.001°`|Yaw，同单位|

SDK：`move_p(pose)` / `move_l(pose)`；两者发送相同三帧，区别在 `0x151.B1` 的 P/L 模式。
SDK 位姿入口还会处理姿态角范围；本工程没有另加工作空间几何裁剪。

**ROS：已开放**于 `command/cartesian`，消息为 XYZ m + RPY rad 及 `speed_percent`（同样先通过 `0x151` 发送速度比例，再发上述三帧）；
需先选择 point 或 linear 并等实际模式反馈。目标不自动包含任意工具偏移。

#### RX `0x2A2`～`0x2A4`：设备反馈的基座到法兰位姿

|CAN ID / 消息类|B0..3|B4..7|
|---|---|---|
|`0x2A2` / `ArmMsgFeedbackEndPoseXY`|X，`i32 × 1e-6 m`|Y|
|`0x2A3` / `ArmMsgFeedbackEndPoseZRX`|Z，`i32 × 1e-6 m`|Roll，`i32 × 0.001°`|
|`0x2A4` / `ArmMsgFeedbackEndPoseRYRZ`|Pitch，`i32 × 0.001°`|Yaw|

SDK：`get_flange_pose()`。**ROS：已开放**于 `end_pose_stamped`，
`frame_id=base_link`，驱动把 RPY 按 `xyz` 欧拉角转成四元数；三帧不完整/过期时不发布。
这不是自动应用末端工具偏移后的 TCP 位姿。

#### RX `0x2AA`～`0x2AC`：IK 计算结果（非实测角）

|CAN ID / 消息类|B0..3|B4..7|
|---|---|---|
|`0x2AA` / `ArmMsgFeedbackIKJointStates12`|IK J1，`i32 × 0.001°`|IK J2|
|`0x2AB` / `ArmMsgFeedbackIKJointStates34`|IK J3|IK J4|
|`0x2AC` / `ArmMsgFeedbackIKJointStates56`|IK J5|IK J6|

SDK：`get_ik_joint_angles()`。**ROS：未开放**，不能用它替代 `joint_states_feedback`。

<a id="mode-c"></a>

### 2.3 `c`：圆弧运动

#### TX `0x158`：圆弧点编号 `ArmMsgCircularPatternCoordNumUpdateCtrl`

`B0=1/2/3` 分别标记起点/中点/终点；`0` 无效；`B1..7=0`。
它不含坐标，坐标仍通过 [`0x152/0x153/0x154`](#mode-p-l) 发送。

SDK `move_c(start_pose, mid_pose, end_pose)` 的顺序是：

```text
0x151 选择 C 模式（SDK 自动切模式开启时）
0x152 + 0x153 + 0x154 起点 → 0x158.B0=1
0x152 + 0x153 + 0x154 中点 → 0x158.B0=2
0x152 + 0x153 + 0x154 终点 → 0x158.B0=3
```

**ROS：未开放。** `CartesianCommand` 只有一个 pose，不支持用三次发布冒充圆弧指令。

<a id="mode-mit"></a>

### 2.4 `mit`：位置、速度、刚度、阻尼、前馈

#### TX `0x15A`～`0x15F`：逐轴 MIT `ArmMsgJointMitCtrlN`

|CAN ID|目标轴|CAN ID|目标轴|CAN ID|目标轴|
|---|---|---|---|---|---|
|`0x15A`|J1|`0x15C`|J3|`0x15E`|J5|
|`0x15B`|J2|`0x15D`|J4|`0x15F`|J6|

六帧布局相同；V188+ 为 **16 + 12 + 12 + 12 + 12 bit，共 8 字节，无旧应用层 CRC**：

|字节|发什么|
|---|---|
|B0..1|位置 `p_des[15:0]`|
|B2|速度 `v_des[11:4]`|
|B3|高半字节 `v_des[3:0]`，低半字节 `kp[11:8]`|
|B4|`kp[7:0]`|
|B5|`kd[11:4]`|
|B6|高半字节 `kd[3:0]`，低半字节 `t_ff[11:8]`|
|B7|`t_ff[7:0]`|

量化是区间到无符号整数的线性映射，不是 IEEE 浮点或有符号补码：
`u = int((value-min) × (2^bits-1)/(max-min))`。

|字段|位宽|线侧量化范围|
|---|---|---|
|`p_des`|16|`-12.5..12.5 rad`|
|`v_des`|12|`-45..45 rad/s`|
|`kp`|12|`0..500`|
|`kd`|12|`-5..5`；自有 ROS 只接受 `0..5`|
|`t_ff`|12|`-16..16`，SDK 先将关节侧请求除以上文的 `c[i]`|

SDK：`move_mit(i, p_des, v_des, kp, kd, t_ff)`，V189 不再对 MIT 的 J4/J5 额外取反。
SDK 该入口的前馈允许值是 `±16×c[i]`；自有 ROS 使用的检查值由
`±8×b[i]×c[i]` 计算，**两者不是同一个范围**，超出 SDK 入口范围时仍可能被 SDK 裁剪。
不能把其中任一个范围称为机械安全力矩或保证准确的输出能力。

固件概念上的控制量：

```text
T_ref = kp × (p_des - p) + kd × (v_des - v) + t_ff
```

**`kp / kd` 与 `t_ff` 的减速比尺度差异及本工程屏蔽约定：**

- **SDK / 驱动器原始尺度差异**：
  - `t_ff`：SDK（先除以 `c[i]`）与固件（结合 `k[i]×b[i]`）已做电机侧到关节输出侧的力矩系数换算，因此 `move_mit` 的 `t_ff` 输入直接对应**减速后整个臂在关节端实际展现的力矩（Nm）**。
  - `kp` 与 `kd`：SDK 在 `move_mit` 中未做减速比或力矩系数换算，直接将 `[0, 500]` 与 `[-5, 5]` 线性量化为 12 位无符号整数发给电机驱动器；驱动器内部 $k_{p,\text{raw}}(p_{\text{des}} - p) + k_{d,\text{raw}}(v_{\text{des}} - v)$ 作用于未经过减速器放大的电机侧尺度，经减速器输出到关节后，**关节实际展现出来的刚度 $K_{p,\text{joint}}$ 与阻尼 $K_{d,\text{joint}}$ 会同步放大 $\alpha_i$ 倍**：
    $$\tau_{\text{joint}} = (\alpha_i \cdot k_{p,\text{raw}})(p_{\text{des}} - p) + (\alpha_i \cdot k_{d,\text{raw}})(v_{\text{des}} - v) + t_{\text{ff}}$$
    其中 J1..J6 的关节刚度/阻尼放大系数为 `MIT_GAIN_SCALE = [24.75, 24.75, 28.41, 2.10, 1.36, 1.0]`。
- **本工程统一以减速后物理量为准**：
  - 实际使用中，本工程在 [`backend.py`](piper_x_driver/backend.py) 的 `SdkArm` 边界层屏蔽减速比放大问题，**所有上层接口（ROS `command/mit`、`debug_mit`、`Controller`、示教配置 `kp/kd` 以及重力折合 `offset = gravity / kp`）统一直接以减速后关节端展现的刚度（Nm/rad）与阻尼（Nm/(rad/s)）为准**。
  - `SdkArm._write_mit` 在调用 `move_mit` 下发前，自动将减速后的 `kp`、`kd` 除以各轴 `MIT_GAIN_SCALE[i]` 换算为线侧原始值，并按 12 位 `kp` 量化后的实际关节刚度校准重力位置偏置，使关节端实际产生的补偿力矩与上层减速后物理量严格一致。

**ROS：已开放**于 `command/mit`，每条 ROS 消息必须含六轴完整五组数组；
驱动每轮拆成六帧发送，不是六轴原子 CAN 消息。切模式走 `0x151`，使能仍独立。
`debug_mit` 与四臂示教也是复用这条链路，没有另一套专用 MIT CAN 协议。
主机重力补偿如何作用于目标、输入超时如何保留输出、显式停止如何渐变，
见 [驱动与 ROS MIT 语义](README.md#3-重力补偿及其分配机制)；它们不是上述 8 字节之外的固件功能。

<a id="mode-cpv"></a>

### 2.5 `cpv`：单关节位置/速度及控制参数

#### TX/RX `0x181`～`0x186`：`ArmMsgCPVSettingsAndQueriesN` / `ArmMsgFeedbackCPVResponseN`

|CAN ID|关节|CAN ID|关节|CAN ID|关节|
|---|---|---|---|---|---|
|`0x181`|J1|`0x183`|J3|`0x185`|J5|
|`0x182`|J2|`0x184`|J4|`0x186`|J6|

这是同一 ID 上的请求/应答，普通参数帧 **DLC=7**：

|字节|发什么 / 回什么|
|---|---|
|B0|ASCII `w` (`0x77`) 写；`r` (`0x72`) 读；RX `a` (`0x61`) 参数应答|
|B1..2|两个 ASCII 字符的参数类型，见下表|
|B3..6|`i32` 参数值；查询请求填 0|

写确认另有 **DLC=1、数据 `[0xAC]`** 的帧，不要按 7 字节参数应答解码。

|类型|能力|每个 raw count 的量|SDK 入口|
|---|---|---|---|
|`po`|位置目标/查询|`0.001°`，SDK 对外 rad|`move_cpv_pos` / `get_cpv_pos`|
|`sp`|速度目标/查询|`0.001 rad/s`|`move_cpv_vel` / `get_cpv_vel`|
|`ac`|加速度|`0.01 rad/s²`|`set_cpv_acc` / `get_cpv_acc`|
|`dc`|减速度|`0.01 rad/s²`|`set_cpv_dcc` / `get_cpv_dcc`|
|`vv`|轮廓速度|`0.001 rad/s`|`set_cpv_cv` / `get_cpv_cv`|
|`pp`|位置环 Kp|`0.01`|`set_cpv_pp` / `get_cpv_pp`|
|`kp`|速度环 Kp|`0.01`|`set_cpv_kp` / `get_cpv_kp`|
|`ki`|速度环 Ki|`0.01`|`set_cpv_ki` / `get_cpv_ki`|

PiPER-X V189 的 SDK 对 **CPV 位置 J4/J5** 取反、对 **CPV 速度 J2/J3** 取反。
这是该型号调用入口的补丁，不应复制到普通 J 目标、MIT 或所有 CPV 读回。
自动切模式开启时，SDK 会选择 `0x151.B1=5`；自有适配器已关闭自动切模式。

**ROS：未开放**，没有 `command/cpv` 或 CPV 参数服务。

---

<a id="motion-ctrl"></a>

## 3. 急停、轨迹控制与原生示教

### 3.1 TX `0x150`：运动控制 `ArmMsgMotionCtrl`

|字节|发什么|
|---|---|
|B0|`emergency_stop`：`0` 无效、`1` 电子快速急停、`2` 恢复|
|B1|`track_ctrl`：`0` 关闭、`1` 暂停规划、`2` 继续、`3` 清当前轨迹、`4` 清全部、`5` 获取当前规划、`6` 终止、`7` 传输、`8` 传输结束|
|B2|`grag_teach_ctrl`：`0` 关闭、`1` 开始拖动记录、`2` 结束记录、`3` 回放、`4` 暂停、`5` 继续、`6` 终止、`7` 去轨迹起点|
|B3..7|**当前编码器固定发 0**|

SDK `electronic_emergency_stop()` 发 `B0=1`，`reset()` 发 `B0=2`；
拖动/轨迹字段可由底层消息构造发送。结果要看 [`0x2A1`](#arm-status)，发送成功不等于设备已执行。
电子急停报文不是物理急停，也不是本工程 MIT 软停或简单撤销使能。

消息类的历史注释还描述 B3 轨迹点序号、B4..5 名称索引、B6..7 CRC16，
但固定版本构造器/编码器没有实现这些字段；
`0x476` 解码器也没有解码历史注释中的轨迹点确认字段。
因此不能据此宣称当前 SDK 已提供完整轨迹上传/下载协议。

**ROS：仅开放 B2 的开始/结束**：
`set_mode drag` 发 `B2=1`，`set_mode hold` 发 `B2=2`，B0/B1 为 0。
`hold` 是本工程命名，不是厂商独立的 HOLD 模式码。
电子急停/恢复、固件轨迹管理、回放、暂停/继续和去起点均没有自有 ROS 服务。
四臂 MIT 双向示教是主机算法，不使用这套固件录制/回放。

<a id="leader-follower"></a>

### 3.2 固件原生 Leader/Follower 与回零

#### TX `0x470`：`ArmMsgLeaderFollowerModeConfig`

|字节|发什么|
|---|---|
|B0|`0x00` 无效；`0xFA` Leader 示教输入臂；`0xFC` Follower 运动输出臂|
|B1|历史反馈 ID 偏移|
|B2|历史控制 ID 偏移|
|B3|历史联动目标 ID 偏移|
|B4..7|0|

固定 SDK 的 `set_leader_mode()` / `set_follower_mode()` 将 B1..3 发 0；
这些偏移字段已标记废弃，不据此提供多臂地址规划承诺。

#### TX `0x191`：`ArmMsgLeaderArmMoveToHome`

|SDK 入口|发出的 B0..7|含义|
|---|---|---|
|`restore_leader_drag_mode()`|`01 00 00 00 00 00 00 00`|恢复原生 Leader/Follower 拖动|
|`move_leader_to_home()`|`01 01 01 00 00 00 00 00`|Leader 回零|
|`move_leader_follower_to_home()`|`01 00 01 00 00 00 00 00`|Leader/Follower 一起回零|

这是请求移动到原生 home，不是通过 [`0x475`](#joint-limits-config) 重设关节硬件零点。

#### RX `0x155/0x156/0x157`：Leader 输出目标

布局与 [2.1 关节控制帧](#mode-j-js) 相同，SDK 以 `leader_joint_12/34/56` 接收，
通过 `get_leader_joint_angles()` 聚合。它不是 `0x2A5`～`0x2A7` 的实测反馈。

**ROS：以上原生 Leader/Follower 能力均未开放。**
本工程 1↔3、2↔4 的 MIT 配对不调用 `0x470/0x191`，两者不要混为一种示教模式。

---

<a id="gripper-ctrl"></a>

## 4. 夹爪控制与反馈

### 4.1 TX `0x159`：`ArmMsgGripperCtrl`

|字节|发什么|
|---|---|
|B0..3|`i32` 位置；宽度模式为 `1e-6 m`，角度模式为 `0.001°`|
|B4..5|力度字段；编码器按 `u16` 发送，SDK `force × 1000`；消息文字称 mN，反馈解码按 `i16`|
|B6|`0` 失能/宽度；`1` 使能/宽度；`2` 失能+清错/宽度；`3` 使能+清错/宽度；`4..7` 分别为上述四种操作的角度模式|
|B7|`0` 不设零；`0xAE` 把当前位置设为零|

SDK：

- `move_gripper_m(value, force)`：宽度转换为 µm，B6=1，B7=0。
- `move_gripper_deg(value, force)`：角度转换为毫度，B6=5，B7=0。
- `disable_gripper()`：在本工程要求的宽度反馈下发 B6=0；`reset_gripper()` 使用清错状态码。
- `calibrate_gripper()`：默认位置/力度/状态均 0，B7=`0xAE`，
  等待 [`0x476`](#param-ack) 的指令索引 **`0x75`** 和设零结果，不是简单等待 `0x59`。

SDK 虽把 force 称作 N，本工程尚未标定与真实接触力的关系，ROS `effort`
只表示该 SDK 数值尺度，不宣称是已校准的夹持力。
夹爪/示教器行程与摩擦参数见 [5.5 `0x47D`/`0x47E`](#gripper-pendant-params)。

### 4.2 RX `0x2A8`：`ArmMsgFeedbackGripper`

|字节|返回什么|
|---|---|
|B0..3|`i32` 位置；宽度模式乘 `1e-6 m`，角度模式乘 `0.001°`|
|B4..5|`i16 × 0.001`，SDK 力度反馈字段|
|B6|bit 0 低电压、1 电机过温、2 过流、3 驱动器过温、4 传感器异常、5 驱动器错误、**6 使能**、7 已设零/曾设零|
|B7|`0` 宽度模式；`1` 角度模式；固定解码器把非 1 也按宽度处理|

SDK：`get_gripper_status()`；还可接收 `0x159` 控制帧作为
`get_gripper_ctrl_states()` 的数据源，但控制目标/回显不能替代 `0x2A8` 的实测开度。

**ROS：部分开放。**

- `command/gripper`：总开度 `0..0.1 m`、effort `0..5`，独立于运动模式，不设零、不清错。
- `joint_states_feedback.position[6]`：实测总开度；`arm_status`：夹爪使能与有效性。
- `enable_srv`：夹爪使能/失能与六关节一起调度。
- `calibrate_effector`：显式确认物理闭合后写设备零点；需 ACK 后再验证新鲜失能与
  `0..0.1 mm` 零位回读。断电保持性未验证。
- 角度模式、夹爪完整错误位、力度反馈、清错及控制帧回显没有自有 ROS 接口。

---

<a id="device-params"></a>

## 5. 设备参数查询、配置与设置确认

### 5.1 参数查询/恢复总入口 (`0x477`) 与设置确认 (`0x476`)

#### TX `0x477`：`ArmMsgParamEnquiryAndConfig`

这条消息复用多个互相独立的功能，不能只按 ID 判断它在做什么。

|字节|发什么|
|---|---|
|B0|查询：`0` 无；`1` [末端速度/加速度→`0x478`](#end-vel-acc-limits)；`2` [碰撞等级→`0x47B`](#crash-assistance-rating)；`3` 当前轨迹索引（无完整专用解析链）；`4` [夹爪/示教器参数→`0x47E`](#gripper-pendant-params)；`5` [关节助力→`0x488`](#crash-assistance-rating)|
|B1|参数恢复：`0` 无；`1` [末端速度/加速度恢复默认](#end-vel-acc-limits)；`2` [全关节角度/最大速度/加速度恢复默认](#joint-limits-config)|
|B2|历史 `0x48x` 周期反馈：`0` 无效；`1` 开启；`2` 关闭|
|B3|`0xAE` 使固件负载档位设置生效；0 不设置|
|B4|负载档位代码，见下文的实现差异|
|B5..7|0|

SDK `set_flange_vel_acc_limits_to_default()`、
`set_joint_angle_vel_acc_limits_to_default()`、
`set_links_vel_acc_period_feedback()`、`set_payload()` 均通过此帧发送。
这些设置通常等待 `0x476` 的指令 `0x77`；收到确认不等于做了物理回读验证。

**两个不能省略的边界：**

1. SDK 明确把 `0x481`～`0x486` 周期链节速度/加速度标为废弃、全零无意义，
   当前 PiPER 解码映射也没有这些帧；这里不将其写成可用的实时遥测能力。
2. 固定版本 `set_payload('empty'/'half'/'full')` 的枚举实际发送 B4=`1/2/3`，
   但同版本消息注释写的是 `0/1/2`，`3` 无效。这里记录差异，
   **不宣称该档位语义已由真机验证**，也不把它当作 kg 或负载辨识接口。

**ROS：上述设置未开放。** `configure_arm` 保存的末端质量/质心与 `gravity_scale`
是主机模型配置，不发 `0x477`，不改固件负载档位或固件重力参数。

<a id="param-ack"></a>

#### RX `0x476`：`ArmMsgFeedbackRespSetInstruction`

|字节|返回什么|
|---|---|
|B0|`instruction_index`，用于匹配设置请求：[`0x74` 角度/速度](#joint-limits-config)、[`0x75` 关节配置](#joint-limits-config)/[夹爪设零](#gripper-ctrl)、[`0x77` 通用参数](#device-params)、[`0x79` 末端限制](#end-vel-acc-limits)、[`0x7A` 碰撞](#crash-assistance-rating)、[`0x7D` 末端参数](#gripper-pendant-params)、[`0x87` 助力](#crash-assistance-rating)|
|B1|`is_set_zero_successfully`，设零结果，1 成功；不能泛化成所有设置的通用成功码|
|B2..7|当前解码器不使用|

ACK-only 的 SDK 返回 true 只说明匹配到指令索引；有些设置还额外查询实际配置。
自有 ROS 不发布此原始确认帧，只在 `calibrate_effector` 内部使用对应确认结果。
不要拿任何旧 ACK 当成运动到位、使能成功或设备参数已生效的证据。

<a id="joint-limits-config"></a>

### 5.2 关节角度/速度/加速度参数、关节设零与清错

|方向 / CAN ID / 消息|每条消息的内容|
|---|---|
|TX `0x472` / `ArmMsgSearchMotorMaxAngleSpdAccLimit`|B0=关节 `1..6`；B1=`1` 查角度/最大速度、`2` 查最大加速度；B2..7=0|
|RX `0x473` / `ArmMsgFeedbackAllCurrentMotorAngleLimitMaxSpd`|B0=关节；B1..2=`i16` 最大角，B3..4=`i16` 最小角，均 `0.1°`；B5..6=`u16 × 0.01 rad/s` 最大速度；B7 保留|
|TX `0x474` / `ArmMsgMotorAngleLimitMaxSpdSet`|B0=关节；B1..2=最大角；B3..4=最小角；B5..6=最大速度，类型/单位同 `0x473`；B7=0|
|TX `0x475` / `ArmMsgJointConfig`|B0=关节；B1=`0xAE` 将当前位置设零；B2=`0xAE` 使加速度设置生效；B3..4=`u16 × 0.01 rad/s²` 最大加速度；B5=`0xAE` 清关节错误；B6..7=0|
|RX `0x47C` / `ArmMsgFeedbackAllCurrentMotorMaxAccLimit`|B0=关节；B1..2=`u16 × 0.01 rad/s²` 最大加速度；其余未由当前解码器使用|

`0x474` 的三项数值和 `0x475` 的最大加速度可用 `0x7FFF` 表示该项不设置；
不要当作实际角度、速度或加速度。`0x475` 的 `0xAE` 是各操作的有效标志，
不需要的操作保持 0。该消息把 SDK 编号 `255` 转成线侧 `7` 表示全部关节，
与 `0x471` 的历史编号歧义应分开看。

SDK 对应入口：

- `get_joint_angle_vel_limits(i)` / `set_joint_angle_vel_limits(i, ...)`：
  查询 `0x472 → 0x473`；设置 `0x474 → 0x476`，并回查实际数值。
- `get_joint_acc_limits(i)` / `set_joint_acc_limits(i, ...)`：
  查询 `0x472 → 0x47C`；设置 `0x475 → 0x476`，并回查实际数值。
- `calibrate_joint(i)`：`0x475.B1=0xAE`，等待 `0x476` 的指令 `0x75` 和设零结果。
- `clear_joint_error(i)`：`0x475.B5=0xAE`，等待指令确认，不自动证明错误已经消失。

**ROS：均未开放。** 没有六关节硬件标零服务、关节零偏 YAML、自动清错或设备限位写服务。
`calibrate_effector` 只标定末端，不是这里的 `calibrate_joint`。

<a id="end-vel-acc-limits"></a>

### 5.3 末端速度/加速度限制

|方向 / CAN ID / 消息|B0..1|B2..3|B4..5|B6..7|
|---|---|---|---|---|
|TX `0x479` / `ArmMsgEndVelAccParamConfig`|最大线速度|最大角速度|最大线加速度|最大角加速度|
|RX `0x478` / `ArmMsgFeedbackCurrentEndVelAccParam`|当前配置最大线速度|当前配置最大角速度|当前配置最大线加速度|当前配置最大角加速度|

四项都是 `u16`，分别乘 `0.001 m/s`、`0.001 rad/s`、`0.001 m/s²`、`0.001 rad/s²`。
它们是**配置上限**，不是末端瞬时实测速度。

SDK：`get_flange_vel_acc_limits()` 发送 `0x477.B0=1`；
`set_flange_vel_acc_limits(...)` 发送 `0x479`，等待确认并回查。
**ROS：未开放**。ROS 的 `speed_percent` 是 `0x151` 的速度比例，不是这四项限制。

<a id="crash-assistance-rating"></a>

### 5.4 碰撞防护等级与拖动助力等级

|方向 / CAN ID / 消息|每条消息的内容|
|---|---|
|TX `0x47A` / `ArmMsgCrashProtectionRatingConfig`|B0..5=J1..J6 碰撞等级，每轴 `u8`、`0..8`，0 关闭碰撞检测；B6..7=0|
|RX `0x47B` / `ArmMsgFeedbackCrashProtectionRating`|B0..5=J1..J6 当前碰撞等级；B6..7 未使用|
|TX `0x487` / `ArmMsgJointAssistanceRatingConfig`|B0..5=J1..J6 助力等级，每轴 `u8`、`0..10`，0 不助力；B6..7=0|
|RX `0x488` / `ArmMsgFeedbackJointAssistanceRating`|B0..5=J1..J6 当前助力等级；B6..7 未使用|

SDK `get_crash_protection_rating()` 用 `0x477.B0=2` 查询，
`set_crash_protection_rating(...)` 用 `0x47A` 写；
`get_joint_assistance_rating()` 用 `0x477.B0=5` 查询，
`set_joint_assistance_rating(...)` 用 `0x487` 写。设置入口还会等待确认并回查。

**ROS：均未开放。** 助力等级不是 `MitCommand.kp/kd`，
也不是 `configure_arm.gravity_scale`；碰撞等级没有已标定的 Nm/N 阈值换算。

<a id="gripper-pendant-params"></a>

### 5.5 夹爪/示教器行程、倍率与摩擦参数

|方向 / CAN ID / 消息|每条消息的内容|
|---|---|
|TX `0x47D` / `ArmMsgGripperTeachingPendantParamConfig`|B0=示教行程比例 `100..200`；B1=最大行程代码 `0/70/100`（mm 尺度）；B2=示教摩擦 `1..10`；B3..7=0|
|RX `0x47E` / `ArmMsgFeedbackGripperTeachingPendantParam`|B0=行程比例；B1=最大行程，SDK 乘 `0.001` 转为 m；B2=摩擦等级；其余未使用|

SDK 末端驱动：`get_gripper_teaching_pendant_param()` 用 `0x477.B0=4` 查询；
`set_gripper_teaching_pendant_param(...)` 写 `0x47D`，等待 `0x7D` 确认并回查。
**ROS：未开放。** 切换 YAML 的末端配置不是把这些参数写入示教器/夹爪。

---

<a id="firmware-and-host"></a>

## 6. 固件身份查询与主机计算边界

### 6.1 TX/RX `0x4AF`：`ArmMsgReqFirmware` / `ArmMsgFeedbackFirmware`

- **TX：DLC=1，只发 `[0x01]`**，不是所有请求都填满 8 字节。
- **RX：每段 8 字节，共 11 段 / 88 字节**。SDK 按接收顺序拼接，以 `H-V` 开头检查完整性。
- SDK `get_firmware()` 从拼接数据提取：硬件版本 `[0:8]`、减速比/批次 `[16:18]`、
  节点类型 `[32:38]`、软件版本 `[60:68]`、生产日期 `[68:74]`、节点号 `[76:78]`；
  此处区间按 Python 的左闭右开表示。

**ROS：仅启动内部使用。** 适配器等待接收流就绪后查询并核对 `S-V1.9-0`，
不匹配即连接失败；目前没有固件查询 ROS 服务或完整身份话题。

### 6.2 SDK 的本地计算不等于新的 CAN 消息

SDK 的正运动学、法兰/TCP 坐标变换、工具偏移等可在主机计算，
不应虚构成“查询某条 CAN ID 得到任意 TCP”的协议。
同样，主机的重力模型、MIT 目标缓存、调试会话与四臂耦合，
都复用前面的目标/使能/反馈帧，不是额外的 CAN 控制模式。

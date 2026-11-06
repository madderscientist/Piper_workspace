# paxini_driver — 帕西尼 PX6D 六维力传感器 ROS 2 驱动

依据《PX6D 6 Axis Force/Torque Sensor Instruction Manual》第 4、5 章实现，支持 RS485、USB、CANFD、CAN 2.0 通信接口，具体指令见下表。

当传感器尚未接入主机时，节点默认保持待机并按 reconnect_sec 周期自动重连，在 sensor_status 话题上报离线状态；接入硬件后自动读取版本号并进入采集状态。

## 1. 协议支持对照（官方手册第 5 章）

| 接口 | 物理层默认参数 | 命令与应答格式 | 传感器六轴数据报文 (Fx, Fy, Fz, Mx, My, Mz，float32 小端) |
| --- | --- | --- | --- |
| RS485 (5.1) | 921600 8N1，无流控 | 0xAA 0x55 + ID + CMD + DATA + CRC8 | 单帧读取 0x05 返回 29 字节帧（CMD=0x03，24 字节负载 + CRC8） |
| USB (5.2) | 921600 8N1，无流控 | 0xAA 0x55 + ID + CMD + DATA + CRC8 | 连续 1 kHz 传输 0x03 或单帧 0x05，29 字节帧 |
| CANFD (5.3) | 仲裁域/数据域默认 1 Mbps | 发送 0x200+ID，应答 0x180+ID | 0x280+ID 单帧 DLC=24（24 字节） |
| CAN 2.0 (5.4) | 默认 1 Mbps | 发送 0x200+ID，应答 0x180+ID | 连续三帧 DLC=8：0x280+ID (Fx,Fy)、0x380+ID (Fz,Mx)、0x480+ID (My,Mz) |

- CRC-8 校验：poly=0x07, init=0x00, refin=False, refout=False, xorout=0x00，覆盖从帧头 0xAA 0x55 到数据域末尾的全部字节，与手册第 5.1.3、5.2.2 节全部示例字节逐位一致。
- 支持指令：
  - 0x01 设置设备 ID（CAN/CANFD 下断电重启生效）
  - 0x02 设置回传频率（数据位 = Freq / 4）
  - 0x03 启动 1 kHz 自动连续回传
  - 0x04 停止自动连续回传
  - 0x05 单次请求一帧六维力数据
  - 0x07 读取固件版本号（如 v0.1.1）
  - 0x10 传感器硬件校零（Calibration）
  - 0x11 CAN / CANFD 协议模式切换（手册 5.5 节）

## 2. 启动与使用

`config/px6d.yaml` 默认配置了双臂力传感器，工作在 `interface: rs485`、`poll_rate_hz: 1000`（通过 0x05 请求采集，实际接收频率见 `rx_hz`），节点收到 `Ctrl+C`（`SIGINT` / `SIGTERM`）退出时会自动下发 `0x04` 指令并停止采集，恢复传感器被动模式：
- `arm_3`（左手，固件 `v1.0.3`）：命名空间 `/arm_3`，默认串口 `/dev/ttyUSB1`（若串口顺序互换会自动按固件版本匹配）
- `arm_4`（右手，固件 `v1.0.1`）：命名空间 `/arm_4`，默认串口 `/dev/ttyUSB0`（若串口顺序互换会自动按固件版本匹配）

```bash
source ./setup_env.bash
colcon build --packages-select paxini_driver --symlink-install
source install/local_setup.bash

# 默认同时启动左手 arm_3 (/arm_3/wrench_stamped) 与右手 arm_4 (/arm_4/wrench_stamped)
ros2 launch paxini_driver px6d.launch.py

# 只启动单臂传感器（支持 arm_3 / left / 左手 或 arm_4 / right / 右手）
ros2 launch paxini_driver px6d.launch.py arm:=arm_3
ros2 launch paxini_driver px6d.launch.py arm:=arm_4
```

## 3. 驱动职责与接口

驱动负责硬件通信，以及使用已有标定结果进行零偏和工具重力补偿。
驱动不控制机械臂，不采集多姿态标定点，不拟合参数，也不写标定配置。
正常采集及重连成功不打印 INFO；通信异常及数据断流才报警，同一异常持续期间只报一次，
恢复收到有效六维力后重置告警。`stream_timeout_sec` 默认 1 秒，仅用于断流告警。

- 发布 `wrench_raw_stamped`：未经零偏、重力补偿或滤波的传感器六维读数。
- 发布 `wrench_stamped`：按已加载参数补偿后的六维力，单位 N / Nm，参考点仍为传感器原始参考点。
- 发布 `sensor_status`：连接、固件版本、实际 `rx_hz`、已加载参数和拟合质量。
- 订阅 `joint_states_feedback`：取得实际姿态，用于计算工具重力。
- `reload_config`（`std_srvs/srv/Trigger`）：读取 config 中本传感器的零偏、质量和重心，
  原子更新补偿参数并重置滤波器，无需重启驱动。标定程序保存结果后自动调用。
- 硬件服务 `calibrate`（0x10 硬件指令）、`start_stream`、`stop_stream`、`read_once`、`get_version`。
- `command` 支持 `reload_config`、`calibrate`、`start`、`stop`、`read_once`、`version`、
  `set_freq:<hz>`、`set_id:<id>`、`set_mode:can`、`set_mode:canfd`。

手动重新加载参数：

```bash
ros2 service call /arm_3/reload_config std_srvs/srv/Trigger '{}'
```

## 4. 独立标定程序

先启动机械臂驱动及 paxini_driver，再运行 `ros2 run paxini_driver calibrate`。
这是独立的 ROS 客户端程序，只通过话题和服务获取数据、控制机械臂；不打开传感器串口。
标定完成后写 config，再请求所选传感器驱动的 `reload_config`，驱动立即应用新结果。

所有臂共用配置顶层的 `calibration_poses`，只需任选一只臂记录一套。
当前公共列表是 arm4 记录的五个姿态。`record` 追加公共姿态，`clear` 清空公共姿态；
`run` 每次读取最新公共列表，控制所选臂运动并保存该传感器独立的零偏和工具重力参数。

每个姿态直接设置六个关节的最终目标，不在标定程序中插值、不逐关节等待到位。
持续保持目标，根据原始六维力读数动态等待稳定：
连续 `calib_settle_sec`（默认 0.8 秒）窗口内，各通道峰峰值均不超过
`calib_stability_range` 才开始采样。前 3 个阈值单位为 N，后 3 个为 Nm。
阈值来自 arm4 静止约 10 秒、6125 条原始读数的实测，各通道峰峰值放大 2 倍；
实测记录保存在 `calibration_stability_measurement`，不使用滤波结果判定稳定。
随后连续采集默认 `calib_sample_sec: 0.5` 秒的原始六维力和对应实际关节位置，
分别取均值代表该姿态。采样中若力读数再次波动，丢弃整段并重新等待稳定。
关节位置用于重力拟合，不作为稳定门槛。采集完所有姿态后再统一求解并保存校准参数。
采样结束后，同样直接设置校准开始前的六个关节目标，保持至力读数稳定，再退出校准控制。
重力方向使用 `piper_x_driver` 中与 SDK 一致的运动学函数，传感器通道方向已用
实测重力数据核对；不将显示 URDF 的关节变换直接用于 SDK 关节反馈。
配置选择与读写集中在 `configuration.py`；保存时串行合并各臂结果并原子替换文件，
避免两只臂同时标定时覆盖彼此的结果，或驱动重载时读到未写完的 YAML。
各传感器条目保存 `calibration_samples`（各姿态的位置、原始力和重力方向均值）及
`calibration_quality`（力/力矩逐分量 RMS 残差），方便复算和检查实际校准误差。

```bash
# 1. 手动或零力拖动将机械臂摆到不同姿态（建议 >= 3 个不同腕部朝向），每摆好一个姿态执行一次 record：
ros2 run paxini_driver calibrate --arm 左手 record

# 查看已记录的校准姿态与当前校准参数
ros2 run paxini_driver calibrate --arm 左手 list

# 2. 启动自动校准：机械臂依次运动到已记录的各个姿态采样，解算零偏并写回 src/paxini_driver/config/px6d.yaml
ros2 run paxini_driver calibrate --arm 左手 run

# 如需重新录制姿态，可先清空
ros2 run paxini_driver calibrate --arm 左手 clear
```

## 5. 求解模型与可辨识性

在传感器坐标系中，设 d 是单位向下重力方向，W=m·9.81，c 是相对于
传感器原始力矩参考点的重心，p=W·c：

```text
F_obs = b_F + W d
M_obs = b_M + c × (W d) = b_M - [d]× p
```

每个采样窗口保存位置均值和力均值；求解时使用逐样本计算后平均的 d，
因为 R(mean(q)) 一般不等于 mean(R(q))。求解是带 W>=0 的力最小二乘及
无额外正则化的力矩最小二乘。补偿保持同一坐标和同一力矩参考点，不重复平移力矩。
`calibration_quality` 报告残差、力/力矩矩阵秩，以及质量和重心是否可辨识。
参数不可辨识时报告这一事实，不将某个最小范数解描述为唯一标定值。
数学模型验证不能替代真实的无接触读数验证，传感器漂移或外部负载仍会形成残差。

# 四臂示教（主从模式 / 双向示教）

`piper_teaching` 通过 ROS 调用已有驱动，支持**主从模式**（`master_slave`）和**双向耦合模式**（`bilateral`）：

| 配对 | 机械臂 | 默认 CAN / 末端 | 主从模式 (`master_slave`) | 双向模式 (`bilateral`) |
|---|---|---|---|---|
| 1 → 3 / 1 ↔ 3 | `arm_1` / `arm_3` | `can0` 示教手柄 / `can2` 夹爪 | `arm_1` 主臂（重力补偿）控制 `arm_3` 从臂（重力补偿+MIT） | 六关节和末端开度双向跟随 |
| 2 → 4 / 2 ↔ 4 | `arm_2` / `arm_4` | `can1` 示教手柄 / `can3` 夹爪 | `arm_2` 主臂（重力补偿）控制 `arm_4` 从臂（重力补偿+MIT） | 六关节和末端开度双向跟随 |

示教节点不打开 CAN，不直接创建 SDK 对象；重力补偿统一在底层驱动中折合到 `target position`（`p_des = position + gravity / kp`），示教不重复计算。

## 1. 预览与启动

先确保 `ros2 launch piper_x_driver four_arms.launch.py` 已在同一 `ROS_DOMAIN_ID` 下运行：

```bash
# 预览模式（默认 dry_run:=true，仅发布配对参考，不操作电机）
ros2 launch piper_teaching teaching.launch.py

# 正式启动主从示教（1、2 主臂重力补偿，分别控制 3、4 从臂）
ros2 launch piper_teaching teaching.launch.py dry_run:=false mode:=master_slave

# 正式启动双向耦合示教（1↔3、2↔4 双向跟随）
ros2 launch piper_teaching teaching.launch.py dry_run:=false mode:=bilateral
```

可通过 `ros2 topic echo /bilateral_teaching/state` 查看状态，各臂参考目标同时发布在 `/bilateral_teaching/arm_1/target`～`arm_4/target`。

正式模式流程（`idle → enabling → preloading → starting → engaging → active`）：

1. 等待四臂反馈正常并确认使能。
2. 保持配置的 `kp / kd` 进入 **engaging** 对齐阶段：
   - `master_slave`：主臂 `arm_1`、`arm_2` 锁定在初始位置，从臂 `arm_3`、`arm_4` 以 **24°/s** 靠近主臂位置。
   - `bilateral`：配对臂 `1↔3`、`2↔4` 以 **24°/s** 相互靠近至初始位置中点。
3. 对齐完成后自动进入 **active**：
   - `master_slave`：主臂 `velocity=0, kd=0` 且夹爪零力释放，经驱动重力补偿实现自由拖动；从臂跟随主臂位置 `p`、速度 `dp` 与夹爪开度。
   - `bilateral`：配对臂按 `1↔3`、`2↔4` 双向交换关节位置、速度与夹爪开度。

运行过程中在示教终端或驱动终端按 **回车（Enter）**，可重新执行完整初始化流程。
也可显式关闭自动开始并手动触发：

```bash
ros2 launch piper_teaching teaching.launch.py dry_run:=false auto_start:=false
ros2 service call /bilateral_teaching/start std_srvs/srv/Trigger '{}'
```

## 2. 停止与关闭

```bash
ros2 service call /bilateral_teaching/stop std_srvs/srv/Trigger '{}'
```

- **上层停止（调用 `/bilateral_teaching/stop` 或 `Ctrl+C` 退出 `teaching.launch.py`）**：停止发布耦合指令，四臂驱动保持在 `mit` 重力补偿模式。
- **驱动退出（`Ctrl+C` 退出 `four_arms.launch.py`）**：底层自动切换到固件原生零力拖动（`drag`）模式。
- **彻底断电**：运行 `ros2 launch piper_x_driver power_off.launch.py` 或调用 `enable_srv(False)`。

## 3. 配置参数

配对耦合参数位于 [bilateral_teaching.yaml](config/bilateral_teaching.yaml)，四臂硬件与重力参数位于 [arms.yaml](../piper_x_driver/config/arms.yaml)：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `mode` | `master_slave` | `master_slave` 或 `bilateral`（非活动阶段也可调 `/bilateral_teaching/set_mode` 切换） |
| `kp` | `[15, 15, 15, 3, 2, 2]` | 六轴位置耦合与重力折合增益 |
| `kd` | `[4.62, 4.65, 2.25, 0.30, 0.24, 0.06]` | 六轴阻尼增益 |
| `gripper_effort` | `2.0` | 夹爪跟随力度（SDK 标度 `0..5`） |
| `feedback_timeout` | `1.0 s` | 反馈时效窗口 |
| `engage_speed_deg_s` | `24 °/s` | engage 阶段最大关节目标变化率 |

更多接口说明见 [ROS API](../../docs/02-ros-api.md#5-四臂双向示教)。

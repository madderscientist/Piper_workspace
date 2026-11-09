# camera_calibration

PiPER-X 腕部相机内参标定与单臂 Eye-in-Hand 手眼外参标定（ChArUco + Web Dashboard）。

- **仅 `arm_3`（IP `192.168.123.97`）和 `arm_4`（IP `192.168.123.98`）装有腕部相机**。
- **启动标定 Dashboard 必须显式指定 `arm` 和 `camera`**，一次只加载并标定所选的单条机械臂及其腕部相机。
- **当前标定板配置**：`12×9` 格，`checker size = 15 mm` (`0.015 m`)，`marker size = 11.25 mm` (`0.01125 m`)，`ArUco DICT_5X5_100`；Dashboard 页面顶部可直接填写并保存其他标定板规格。
- **当前相机与末端硬件配置**：
  - **子码流 `/stream1`（默认档位）**：原生 `720×480`，H.265，**30 fps**。
  - **主码流 `/stream0`（高清档位）**：原生 `1920×1080`，H.264，**30 fps**。
  - **外参参考系 `gripper_base_top`**：`arm_3` 与 `arm_4` 均使用 `piper_description_with_wrench.urdf`（含 PX6D-75A 力传感器与双侧法兰），`link6 -> gripper_base_top` 的 +Z 偏移为 **`92.8 mm` (`0.0928 m`)**。

## 启动方式

```bash
source ./setup_env.bash

# 方式一：分别启动单臂相机与标定 Dashboard（以 arm_3 为例）
ros2 launch camera_node camera.launch.py arm:=arm_3
ros2 launch camera_calibration calibration.launch.py arm:=arm_3 camera:=camera_arm_3

# 方式二：启动标定 Dashboard 时加 start_camera:=true 一并拉起对应相机节点（以 arm_4 为例）
ros2 launch camera_calibration calibration.launch.py arm:=arm_4 camera:=camera_arm_4 start_camera:=true
```

打开浏览器访问 `http://127.0.0.1:8300`：
1. **标定板规格设置**：当前配置为 `12×9 / 15 mm / 11.25 mm / DICT_5X5_100`，可在页面顶部直接修改并点击「应用板规格」。
2. **① 内参标定**：选择分辨率档位（默认 `720×480 @ 30fps` 或 `1920×1080 @ 30fps`），拍摄多角度图后点击「求解」与「保存」。
3. **② 外参标定（单臂手眼 Eye-in-Hand）**：固定标定板，移动所选机械臂至不同姿态（≥ 3 组且旋转轴不平行），停稳后依次点击「采集一组」→「求解」→「保存这一组」。
4. **3D 视锥可视化**：标定保存后，运行 `ros2 run state_dashboard state_dashboard` 打开 `http://127.0.0.1:8089`，展开对应机械臂的 3D 视图即可查看相机光心、光学坐标轴与 FOV 视锥体。

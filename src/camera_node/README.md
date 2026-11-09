# `camera_node`

PiPER-X 腕部 IP 相机 RTSP → `sensor_msgs/Image` 与 `sensor_msgs/CameraInfo`。**没人要图的时候不拉流**。

配置文件位于 [`config/cameras.yaml`](config/cameras.yaml)，统一维护相机名、所属机械臂、别名（`aliases`）与 IP / RTSP 流的映射关系：

| 相机键名 | 所属机械臂 | IP | 常用别名（`aliases`） | 默认子码流 RTSP | 发布话题 |
|---|---|---|---|---|---|
| `camera_arm_3` | `arm_3` (`/arm_3`) | `192.168.123.97` | `arm_3`, `arm3`, `arm3_camera`, `cam3`, `192.168.123.97` | `rtsp://admin:123456@192.168.123.97/stream1` | `/camera_arm_3/image_raw` |
| `camera_arm_4` | `arm_4` (`/arm_4`) | `192.168.123.98` | `arm_4`, `arm4`, `arm4_camera`, `cam4`, `192.168.123.98` | `rtsp://admin:123456@192.168.123.98/stream1` | `/camera_arm_4/image_raw` |

## 当前相机硬件与编码配置

两台腕部 IP 相机（`192.168.123.97` / `192.168.123.98`）均接入主机专用相机网卡 `enp2s0`（静态 IP `192.168.123.100/24`），末端结构为 `piper_description_with_wrench.urdf`（臂侧法兰 `7.5 mm` + PX6D-75A 力传感器 `24.0 mm` + 夹爪侧法兰 `8.8 mm` + 去底法兰夹爪底座 `gripper_base_without_flange.STL` `52.5 mm`，`link6 -> gripper_base_top` 高度 `+92.8 mm`）。

相机固件内的双码流已统一持久化配置为 **30 fps**：

| 码流 | RTSP 路径 | 原生分辨率 | 编码格式 | 帧率 | 码率控制 | 用途 |
|---|---|---|---|---|---|---|
| **子码流（默认）** | `/stream1` | **`720×480`** | H.265 (HEVC) | **30 fps** | VBR 500 kbps | `camera_node` 与标定默认档位，低延迟、低 CPU 占用 |
| **主码流** | `/stream0` | **`1920×1080` (1080P)** | H.264 | **30 fps** | CBR 3000 kbps | 高清采集与 1080p 档位标定（避免 4K H.265 软解掉帧花屏） |

## 启动方式

```bash
source ./setup_env.bash

# 1. 两个臂的腕部相机一起启动（默认不传 arm/camera，或传 arm:=all）
ros2 launch camera_node camera.launch.py
# 等价别名入口：
ros2 launch camera_node cameras.launch.py

# 2. 只选择单个机械臂或相机别名启动
ros2 launch camera_node camera.launch.py arm:=arm_3
ros2 launch camera_node camera.launch.py arm:=arm_4
ros2 launch camera_node camera.launch.py camera:=cam3
ros2 launch camera_node camera.launch.py camera:=192.168.123.98

# 3. 附带标定文件（同步发布 ~/camera_info）与可选 MJPEG 预览页
ros2 launch camera_node camera.launch.py arm:=arm_3 \
  calib_file:=/home/a/piper_x_ws/src/camera_calibration/config/calibration.yaml \
  server_port:=8010
```

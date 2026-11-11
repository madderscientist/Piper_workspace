# `camera_node`

PiPER-X 腕部 IP 相机驱动：同时提供**零 ROS 依赖的纯 Python RTSP 取图接口**（供上层 Agent / 视觉推理脚本直接 `import`，无需启动任何节点）以及**按需拉流的 ROS 2 节点**（供手眼标定 `camera_calibration` 与 ROS 工具链订阅）。

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

## Python 直接取图（无需启动 ROS 节点）

顶层 `import camera_node` 不依赖 `rclpy`，返回标准 OpenCV 格式的 `BGR uint8 (H, W, 3)` `numpy.ndarray`。`camera` 参数支持 `'arm_3'`、`'arm_4'`、`'cam3'`、`'192.168.123.97'` 等任意别名或完整 `rtsp://` 地址；`stream` 可选 `'stream1'`（默认 `720×480`）或 `'stream0'`（高清 `1920×1080`）。

```python
import cv2
from camera_node import WristCamera, grab_frame, load_camera_info

# 1. 单次抓图：临时连接 RTSP 截 1 帧后立即断开，平时零后台 CPU / 带宽开销（实测约 0.45 s）
bgr = grab_frame('arm_4', stream='stream0')
cv2.imwrite('/tmp/arm4_1080p.jpg', bgr)

# 2. 一直拉流、即刻截图：Agent 启动时初始化一次，后台持续抽帧并自动断线重连
cam = WristCamera('arm_4', stream='stream0')
# ... Agent 运行期间任意时刻截图：
instant_bgr = cam.grab(fresh=False)  # < 1 ms 直接取流内最新一帧（若刚启动尚未出帧会自动等到首帧）
post_move_bgr = cam.grab()           # ~16-33 ms 等调用之后到达的下一帧（机械臂刚停稳时防读旧图）
# 程序退出时关闭（也可用 with WristCamera(...) as cam: 自动关闭）
cam.close()

# 3. 读取对应分辨率档位的标定内参（同样不依赖 ROS）
calib = load_camera_info(
    '/home/a/piper_x_ws/src/camera_calibration/config/calibration.yaml',
    'arm_4', width=1920, height=1080,
)
```

环境准备：

- 在工作空间环境中：`source /home/a/piper_x_ws/setup_env.bash` 后即可直接 `import camera_node`。
- 在独立的非 ROS 虚拟环境（如 PyTorch / Agent 专用 venv 或 conda，仅需 `numpy`、`pyyaml` 和系统 `ffmpeg`）中：执行一次 `pip install --no-build-isolation --no-deps -e /home/a/piper_x_ws/src/camera_node`，或设 `PYTHONPATH=/home/a/piper_x_ws/src/camera_node` 即可直接导入。

## ROS 2 节点启动方式（用于标定面板或话题订阅）

平时运行上层脚本不需要启动 ROS 节点；需要运行 `camera_calibration` 或用 ROS 工具订阅图像时再启动（没人订阅时不拉流）：

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

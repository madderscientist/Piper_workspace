# `wrist_camera`

PiPER-X 腕部 IP 相机一体化纯 Python 工具包（无相机 ROS 节点、无图像话题转发）：统一管理双腕相机硬件配置、**零 ROS 依赖的纯 Python 取图与内外参/位姿库**、**按需拉流的网页实时预览**，以及 **ChArUco 内参 + Eye-in-Hand 手眼外参标定面板**。

## 1. 硬件与码流配置

本机腕部相机配置存放在 `$PIPER_CONFIG_DIR/wrist_camera/<相机名>.yaml`（每台相机一个文件，不进 git，模板见根目录 `config/examples/wrist_camera/`）；各相机的标定结果存放在对应 `calibration` 字段指向的 YAML（通常为 `../calibration/<相机名>.yaml`，缺失或无效视为未标定）。包内只保留 [config/board.yaml](config/board.yaml)（ChArUco 标定板尺寸）。

在工作空间外（未设置 `PIPER_CONFIG_DIR`）调用时，需显式传入 `config_file` 参数指向相机配置目录或单个 YAML 文件，否则报错。

## 2. Python 直接 `import` 库（取图 + 内外参 + 基座系位姿）

顶层 `import wrist_camera` 不依赖 `rclpy` 或 `cv2`（仅需 `numpy`、`pyyaml` 和系统 `ffmpeg`），在工作空间（`source ./setup_env.bash`）或外部独立虚拟环境（`pip install --no-build-isolation --no-deps -e <workspace>/src/wrist_camera`）中均可直接导入：

```python
from wrist_camera import (
    WristCamera,
    camera_pose_in_base,
    extrinsic_matrix,
    get_extrinsic,
    get_intrinsics,
    grab_frame,
)

# 1. 单次抓拍：临时连 RTSP 截 1 帧后立即断开，平时零后台 CPU / 带宽开销
bgr_once = grab_frame('arm_4', stream='stream0')

# 2. 常驻热待机拉流（自动断线重连）+ 直接读取该档位内参、外参与基座系位姿
with WristCamera('arm_4', stream='stream0') as cam:
    instant_bgr = cam.grab(fresh=False)      # < 1 ms 直接取流内最新帧
    post_move_bgr = cam.grab(fresh=True)     # ~16-33 ms 等调用之后到达的下一帧（停稳后防读旧图）
    intr = cam.intrinsics()                  # 当前 1920x1080 内参字典 (camera_matrix, distortion_coefficients, ...)
    ext = cam.extrinsic()                    # 手眼外参字典 (gripper_base_top -> camera_arm_4)
    t_top_cam = cam.extrinsic_matrix()       # (4, 4) T_top_cam 齐次变换矩阵
    t_base_cam = cam.pose_in_base(q_rad_6)   # (4, 4) 结合六轴关节角正运动学算出 T_base_cam

# 3. 不开相机流时也可随时单独查询任意档位内参、外参与位姿（自动读取该相机的 calibration 文件）
intr_480 = get_intrinsics('arm_4', stream='stream1')
intr_1080 = get_intrinsics('arm_4', width=1920, height=1080)
ext_dict = get_extrinsic('arm_4')
t_top_cam = extrinsic_matrix('arm_4')
t_base_cam = camera_pose_in_base('arm_4', q_rad_6)
```

## 3. 网页实时预览（`preview`，无人看时零开销）

仅在浏览器打开预览页期间拉取 RTSP 流，关闭页面自动停流：

```bash
source ./setup_env.bash

# 默认同时挂载 arm_3 与 arm_4 双臂预览（浏览器打开 http://127.0.0.1:8010，页面内可切 stream1 / stream0）
ros2 run wrist_camera preview

# 只预览单台腕部相机或指定默认码流 / 端口（也可直接 python -m wrist_camera.preview ...）
ros2 run wrist_camera preview --arm arm_4 --stream stream0 --port 8010
```

## 4. 内参标定与单臂手眼外参标定（`calibrate`）

当前默认标定板规格为 `12×9` 格、方格 `15 mm`（`0.015 m`）、Marker `11.25 mm`（`0.01125 m`）、`DICT_5X5_100`：

```bash
source ./setup_env.bash

# 启动单臂标定 Web 面板（http://127.0.0.1:8300；外参阶段会自动读取驱动的 /arm_X/joint_states_feedback）
ros2 run wrist_camera calibrate --arm arm_4
# 或标定 arm_3：
ros2 run wrist_camera calibrate --arm arm_3
```

生成可打印标定板图：`ros2 run wrist_camera make_board --out /tmp/charuco_board.png`。

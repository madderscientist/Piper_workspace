# head_camera

**Azure Kinect DK 头部相机** 工具包：通过每台相机出厂烧录的 **12 位唯一硬件序列号（USB Serial Number，作用等同于网卡 MAC 地址）** 固定绑定多台相机，默认以 **V4L2 纯 RGB 硬件直通 MJPG** 模式单进程同时管理一台或多台头部相机，提供低延迟 HTTP 单帧抓拍、实时 MJPEG 视频流与按序列号隔离的 **ChArUco 内参标定**。

## 1. 硬件序列号绑定与当前双相机配置

`/dev/video0`、`/dev/video2` 的编号会随插拔顺序变化，但相机 USB 描述符中的 **12 位硬件序列号（`serial`）** 固定跟定物理相机。[config/head_camera.yaml](config/head_camera.yaml) 已绑定当前接入的两台头部相机：

| 相机别名 | 12 位硬件序列号（`serial`） | 说明 |
|---|---|---|
| `head_1` | `000314605112` | 通过序列号自动匹配对应 `/dev/videoN` 与独立内参 |
| `head_2` | `000456614112` | 通过序列号自动匹配对应 `/dev/videoN` 与独立内参 |

## 2. 查询设备、启动全部或只启动指定/新接入相机

```bash
cd /home/a/piper_x_ws
source ./setup_env.bash

# 1. 列出所有已连接相机（已绑定相机显示 [head_1]/[head_2]，新插入未绑定相机显示 [new] 及 12 位 serial）
ros2 run head_camera head_camera list

# 2. 不传 --camera：默认一次同时启动当前接入的所有头部相机（共用 8310 端口，1080p @ 30fps）
ros2 run head_camera head_camera serve

# 3. 传 --camera：只启动指定的那一台（支持别名、12 位硬件序列号或逗号分隔组合）
ros2 run head_camera head_camera serve --camera head_1
ros2 run head_camera head_camera serve --camera 000456614112

# 4. 插入新相机后：先用 list 查出 [new] 的 serial，再单独启动这台新相机（若已有 serve 占用 8310 可加 --port 8311）
ros2 run head_camera head_camera serve --camera <新相机12位serial>
```

远程浏览器通过单端口转发 `ssh -N -L 8310:127.0.0.1:8310 <用户名>@<机器地址>` 打开 `http://127.0.0.1:8310/` 即可同屏查看所有已启动相机的实时画面并分别标定内参。（注意：请避免将两台相机接在同一个外置 USB 2.0 集线器下，以免触发单集线器 USB 2.0 等时带宽上限 `ENOSPC`；分别接入主板不同 USB 口即可双路 `1080p @ 30fps` 同时运行。）

## 3. 如何通过 HTTP 拉取指定相机的图片、视频流与内参

所有接口均支持通过 `?camera=<别名|序列号|序号>` 或路径前缀 `/camera/<别名|序列号>/...` 指定相机（单相机或不传 `camera` 时默认取第一台）：

| 路径示例 | 用途与延迟特性 |
|---|---|
| `GET /frame/color.jpg?camera=head_1` | **即刻拿最新帧**：零拷贝返回内存最新 JPEG（本机 1080p Keep-Alive 实测约 **1 ms**），响应头含 `X-Camera-Serial` 与 `X-Frame-Sequence` |
| `GET /frame/color.jpg?camera=head_2&fresh=1` | **停稳后新帧同步**：等待请求到达**之后**该相机产出的下一帧再返回（30 fps 下约 **16~33 ms**） |
| `GET /stream/color.mjpg?camera=head_1` | **低延迟实时视频流**：事件驱动推送 `multipart/x-mixed-replace`，慢网自动跳帧不积压 |
| `GET /api/intrinsic?camera=head_1` | **读取该相机内参**：按该相机 12 位序列号返回对应分辨率的 `camera_matrix` 与畸变系数 |
| `GET /api/status` | 返回 `cameras` 数组（每台相机的 `name`、`serial`、分辨率、在线状态与内参状态） |

### Python 拉取两台相机图片与内参示例

```python
import cv2
import numpy as np
import requests

session = requests.Session()  # 复用本机 TCP 长连接，每次拉取 1080p 原图约 1 ms
BASE = 'http://127.0.0.1:8310'

def grab_head_bgr(camera: str = 'head_1', *, fresh: bool = False) -> np.ndarray:
    params = {'camera': camera, **({'fresh': '1'} if fresh else {})}
    resp = session.get(f'{BASE}/frame/color.jpg', params=params, timeout=3.0)
    resp.raise_for_status()
    bgr = cv2.imdecode(np.frombuffer(resp.content, dtype=np.uint8), cv2.IMREAD_COLOR)
    if bgr is None:
        raise RuntimeError('解码头部相机 JPEG 失败')
    return bgr

def get_head_intrinsic(camera: str = 'head_1') -> dict:
    info = session.get(f'{BASE}/api/intrinsic', params={'camera': camera}, timeout=2.0).json()
    if not info.get('available'):
        raise RuntimeError(f'相机 {camera} 当前分辨率尚未标定内参')
    return info

# 按别名或 12 位硬件序列号均可拉取：
img1 = grab_head_bgr('head_1')                       # 即刻取 head_1 (000314605112)
img2 = grab_head_bgr('000456614112', fresh=True)     # 等 head_2 停稳后下一帧
```

命令行抓图与低延迟视频流播放：

```bash
curl -fsS 'http://127.0.0.1:8310/frame/color.jpg?camera=head_1' -o head1.jpg
curl -fsS 'http://127.0.0.1:8310/frame/color.jpg?camera=head_2&fresh=1' -o head2.jpg
ffplay -fflags nobuffer -flags low_delay 'http://127.0.0.1:8310/stream/color.mjpg?camera=head_1'
```

## 4. ChArUco 内参标定（按序列号独立保存）

默认标定板规格为 `12 × 9` 格、方格 `15 mm`（`0.015 m`）、Marker `11.25 mm`（`0.01125 m`）、`DICT_5X5_100`。标定结果按相机 12 位硬件序列号与分辨率写入 [config/intrinsic.yaml](config/intrinsic.yaml) 的 `cameras.<serial>.profiles.<WxH>` 下，互不覆盖：

1. **网页交互标定**：浏览器打开 `http://127.0.0.1:8310/` 切到 **「内参标定」**，在下拉框选择 `head_1` 或 `head_2`，手持标定板采集多角度视图后点击 **「求解并保存内参」**。
2. **离线目录标定**：
   ```bash
   ros2 run head_camera head_camera calibrate --camera head_1 --input ~/head_camera_data/capture_...
   ```

## 5. 构建与测试

```bash
cd /home/a/piper_x_ws
source ./setup_env.bash
.venv/bin/python /usr/bin/colcon build --symlink-install --packages-select head_camera
.venv/bin/python /usr/bin/colcon test --packages-select head_camera --event-handlers console_direct+
.venv/bin/python /usr/bin/colcon test-result --test-result-base build/head_camera --verbose
```

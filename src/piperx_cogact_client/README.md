# piperx_cogact_client

本地采集三路图像和双臂反馈，请求云端 CogACT，使用驱动的 C++ IK 将 30 步世界系 TCP 目标转换为关节角，再通过 ROS 2 MIT 下发；不使用固件笛卡尔控制。请求/响应 schema 为 `piperx_inference_request_v1` / `piperx_inference_response_v1`。

## 启动

```bash
cd /home/a/piper_x_ws
source ./setup_env.bash
CMAKE_BUILD_PARALLEL_LEVEL=2 .venv/bin/python /usr/bin/colcon build --symlink-install --packages-select piperx_cogact_client
source ./setup_env.bash
ros2 launch piperx_cogact_client dashboard.launch.py
```

访问 <http://127.0.0.1:8095/>。默认左臂 `arm_3`、右臂 `arm_4`，头相机 `head_2`，另有两路腕相机。

- 运行配置：[config/default.yaml](config/default.yaml)，包含服务器、相机、话题、IK 和执行参数。
- 自定义启动：`ros2 run piperx_cogact_client dashboard --config <配置路径> --port 8095`。
- 离线 UI：`ros2 run piperx_cogact_client dashboard --no-ros --port 8095`，不创建 ROS 节点；未注入关节反馈时不能完成观测请求。

## 执行模式

| 模式 | 行为 |
|---|---|
| 单步 | “请求”获取并求解新批次，“执行一步”下发当前批次的一步；无缓存时执行一步仍会自动请求。 |
| 整批 | 按 `execute_hz` 执行新批次或缓存批次，默认 30 Hz。 |
| 串行闭环 | 采集、推理、IK、执行整批、维持末目标并等待 `settle_delay_s`，默认等待 1 s。 |
| 异步重叠 | 推理与下发并行，按延迟跳步；重叠部分在关节空间线性混合。 |

长度为 $L$ 的重叠段使用 $w_j=(j+1)/(L+1)$、$a_j=(1-w_j)a_j^{old}+w_j a_j^{new}$，同时混合夹爪开度。

“请求”仅在单步模式显示，只计算、不下发该批动作；暂停保留 MIT 目标维持，停止可释放维持。切换模式会停止旧任务、清空预测和进度；迟到结果被丢弃，相机、关节反馈与标定保留，新模式需手动启动。

请求或执行期间按钮显示忙碌样式并禁用；单步在本次操作结束后恢复，整批在全部下发完成或停止后恢复。三路相机固定实时预览，观测采集在推理请求时自动完成。

## 仿真预览

顶部“仿真窗口”打开独立的 URDF/mesh 三维视图，也可访问 `/simulator`。模型原样读取 `piper_description`，保留其固有关节零位和夹爪安装角；双臂按基座标定摆放，关节角加入驱动零偏。模型与驱动 Link6 的差异仅用于 TCP 标记换算，不修改 URDF 或执行目标。

每次“请求预览”都重新采集当前观测、请求模型并求解 IK，不复用或替换真实执行批次。得到新 chunk 后自动播放，进度条显示 `0…30` 步；支持暂停、逐步、拖动进度和速度调整，停止可取消等待或播放。该窗口不下发机械臂命令，仅作运动学回放，不包含接触/碰撞动力学。

浏览器依赖已本地打包，运行时无需 CDN。修改仿真脚本后，在 `piperx_cogact_client/web` 目录运行 `npm ci && npm run build`，再刷新页面。

## 坐标与标定

- 长度用米、关节角用弧度，位姿为 `[x,y,z,qx,qy,qz,qw]`；动作 6D 旋转取矩阵前两行，不是前两列。
- 世界系：`+X` 朝工作区、`+Y` 朝左臂、`+Z` 朝上。TCP 是闭合手指内侧接触面中心，`+Z` 朝指尖、`±X` 沿开合轴，正负轴须与模型约定一致。
- 未落盘时，Link6 到 TCP 默认沿 `+Z` 偏移：普通夹爪 `0.13503 m`，带力传感器夹爪 `0.16483 m`；实际值可在面板调整。
- ChArUco 板平放、打印面朝上并保持不动，可分步采集 `head/left/right`；结合腕相机手眼外参和 FK 求基座外参，再由双基座连线与板面法向建立世界系。

变换使用列向量，工具偏移随姿态旋转：

```text
world_from_tcp = world_from_base @ FK(q) @ link6_from_tcp
base_from_link6_target = inverse(world_from_base) @ world_from_tcp_target @ inverse(link6_from_tcp)
```

FK/IK 使用驱动的关节零偏。IK 首步以实测关节角为初值，后续沿用上一步解；整批求解后再按所选模式下发 `MitCommand`，夹爪使用独立的 `GripperCommand`。

外参、工具偏移和起始位保存于 [config/calibration.yaml](config/calibration.yaml)，启动自动加载。手工修改后需重启；重新标定会重算外参。“记录当前双臂位置”保存关节角与开度，“平滑回到标定起始位”采用五次插值。

## Python 调用

`CameraInput` 接收 BGR 图像或 JPEG/PNG 字节及对应 K/D、world→camera 光学外参；缩放时同步内参，有非零畸变时先去畸变。输入字段见 [cogact_api.py](piperx_cogact_client/cogact_api.py)。

```python
from piperx_cogact_client import request_cogact

chunk, raw_req, raw_resp = request_cogact(
    url="http://10.209.224.190:8332",
    instruction=instruction,
    observation_monotonic_ns=mono_ns,
    observation_unix_ns=unix_ns,
    arms=arms_by_role,     # left/right：六关节、总开口、世界系 TCP 位姿
    cameras=cameras,      # head/wrist_left/wrist_right：CameraInput
    calibration_id="calib_v1",
)
```

此接口仅请求与解析，不执行 IK 或下发动作；HTTP 请求直连服务器，不使用环境代理。

## 测试

```bash
source ./setup_env.bash
ROS_DOMAIN_ID=232 ROS_LOCALHOST_ONLY=1 .venv/bin/python -m pytest -q src/piperx_cogact_client/test
```


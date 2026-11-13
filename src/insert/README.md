# InsertAnything 插入策略部署与控制台 (`insert`)

基于 [`actor.onnx`](actor.onnx) 的六自由度力控插孔策略 ROS 2 部署包与 Web 控制台（默认绑定 `can4` / `arm_4` / `/arm_4`）。

## 1. 启动方式

```bash
# 终端 1：启动驱动
ros2 launch piper_x_driver four_arms.launch.py

# 终端 2：启动 InsertAnything 控制台（默认监听 http://127.0.0.1:8092/）
ros2 run insert web --port 8092
```

也可通过 `ros2 launch` 启动（可选 `start_driver:=true` 一起拉起单臂驱动）：

```bash
ros2 launch insert insert.launch.py port:=8092
```

默认使用 Paxini 力传感器，需要同时启动对应臂的 `paxini_driver`。页面切换到关节估计力时，通过该臂的 `set_wrench_estimation` 服务开启估计；切回 Paxini 时关闭。也可用 `--wrench-source estimated` 或 launch 参数 `wrench_source:=estimated` 选择估计力。

策略推理与逆解在状态锁之外执行，关节和力反馈可以继续接收。独立 50 Hz 定时器发送最近完成的目标，避免慢逆解导致 MIT 输入断流；停止策略后，迟到的计算结果不会继续发送。页面耗时包含推理与逆解，未选中的关节估计数据不进行作用点求解。

## 2. 操作流程

1. **夹爪开关与零力拖动**：在前端页面点击 **“打开夹爪”** / **“闭合夹爪（夹紧插头）”** 控制夹爪夹紧插头，点击 **“进入零力拖动模式”** 用手将夹着插头的末端插入孔中。
2. **设置力作用点与记录孔中位置**：确认插头末端相对于默认力作用点的位置偏移（默认 `z + 19.5 cm`，可在页面直接修改 `X / Y / Z (cm)`），点击 **“记录当前孔中位置”** 记录孔坐标系 `H` 基准位姿。
3. **移出孔外并执行策略**：将拿着插头的末端从孔中拿起来（移出孔），点击 **“执行插入策略”**，节点以 50 Hz 运行 [`actor.onnx`](actor.onnx) 完成闭环力控插孔。

估计力和 Paxini 力传感器分别维护滤波状态。默认时间常数为 15 ms，
每帧根据反馈时间戳间隔 Δt 计算 `α = 1 - exp(-Δt / τ)`，没有时间戳时使用接收间隔。
切换力来源时使用对应数据流的系数，频率变化后自动调整，保持相同的时间响应。
页面显示当前来源的时间常数和实际滤波系数；驱动频率与 insert 接收频率分别显示。

## 3. Agent 视觉引导全流程控制台 (`agent_web`)

位于 [`agent_web/`](agent_web/) 与 [`insert/agent_flow/`](insert/agent_flow/)，串联四阶段完整流程（默认监听 `http://127.0.0.1:8093/`）：

```bash
# 确保双外部相机服务与本地 copilot-api (127.0.0.1:4141) 已运行
ros2 run head_camera head_camera serve

# 启动全流程网页（默认绑定 arm_4，调用 gpt-6.1-sol · max）
ros2 run insert agent_web --port 8093
# 或使用 launch：
ros2 launch insert agent.launch.py port:=8093
```

1. **准备与 Float**：机械臂默认保持零力拖动（Float），右侧实时显示 `head_1`、`head_2`、`camera_arm_4` 三视角画面与系统状态。
2. **板块 1 · 相机位置标定**：空夹爪拖动机械臂使腕部相机与双外部相机同看 `12×9` ChArUco 标定板，一键解算并持久化 `T_base_head_1`、`T_base_head_2`。
3. **板块 2 · 双图 Agent 识别**：前端提供两段式可编辑 Prompt，单次调用 `gpt-6.1-sol (max)` 同时识别两张外部相机图，输出 `arm_4` 坐标系下孔口位置、插入朝向、对齐增量并绘制三维重投影标记图。
4. **板块 3 · 初步移动**：平滑控制插头姿态对齐孔轴，并将插头末端移至孔口正前方 `0.5 cm` 处悬停锁位。
5. **板块 4 · 执行插入**：与原 `insert` 一致选择 `Rule-based` 或 `Policy-based` 完成 50 Hz 闭环力控插入。

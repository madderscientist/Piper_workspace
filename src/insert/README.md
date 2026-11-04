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

## 2. 操作流程

1. **夹爪开关与零力拖动**：在前端页面点击 **“打开夹爪”** / **“闭合夹爪（夹紧插头）”** 控制夹爪夹紧插头，点击 **“进入零力拖动模式”** 用手将夹着插头的末端插入孔中。
2. **设置力作用点与记录孔中位置**：确认插头末端相对于默认力作用点的位置偏移（默认 `z + 19.5 cm`，可在页面直接修改 `X / Y / Z (cm)`），点击 **“记录当前孔中位置”** 记录孔坐标系 `H` 基准位姿。
3. **移出孔外并执行策略**：将拿着插头的末端从孔中拿起来（移出孔），点击 **“执行插入策略”**，节点以 50 Hz 运行 [`actor.onnx`](actor.onnx) 完成闭环力控插孔。

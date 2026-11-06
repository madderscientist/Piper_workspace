# PiPER-X 关节重力补偿与静摩擦标定 (`piper_gravity`)

针对 URDF CAD 理论重力模型与真实机械臂（含线束、减速器、电机转子及力矩常数偏差）之间存在不一致的问题，本包通过**开启基础重力支撑、关闭摩擦补偿、双向准静态 S 曲线阶梯逼近与正反半和/半差解耦**，同时标定各关节的**重力与惯量比例系数 `scale`** 及**准静态静摩擦 $F_s$**。

---

## 1. 物理原理与标定顺序

### 1.1 为什么要“先重力标定，再摩擦力标定”？
在摩擦力标定（`piper_friction`）中，残余力矩由实测力矩减去重力矩得到：$\tilde{\tau} = \tau_{\text{meas}} - \tau_g(q)$。如果 $\tau_g(q)$ 本身存在 $5\% \sim 12\%$ 的比例偏差，重力残差会直接混入残余力矩。因此标准流程是：
1. **先用 `piper_gravity` 标定出精确的重力模型 `arm_X_gravity.yaml`（并测得静摩擦 $F_s$ 与惯量缩放比例）**；
2. **再基于校准后的重力模型运行 `piper_friction` 标定动摩擦与空间周期谐波**。

### 1.2 重力比例与 $3\times 3$ 惯性张量的等比例缩放
- **等比例缩放不改变惯性张量几何形状**：各连杆惯性张量保存在 [gravity.py](../piper_x_driver/piper_x_driver/gravity.py)，三种末端的质量属性统一保存在 [effectors.yaml](../piper_x_driver/config/effectors.yaml)，通过同一加载逻辑得到完整 $3\times 3$ 惯性张量 $\mathbf{I}_k$。标定出重力比例因子 $s_j$ 后，`ArmGravityModel.inertias(q)` 与 `scaled_inertia_tensors()` 按比例缩放转动惯量而不改变张量主轴方向与形状，供 `piper_friction` 扣除低加速度过渡段的惯性力矩 $J_j^{\text{cal}}\ddot{q}_j$。
- **为什么仍需限制大加速度段**：电机转子反射惯量 $N_i^2 I_{\text{rotor}, i}$ 关于转轴对称，对静态重力无贡献，但占高速角加速惯量的显著比例；因此摩擦标定仅对小加速度过渡段做惯量修正与余弦降权，大加速度段直接硬截断剔除。

### 1.3 带力传感器、法兰和相机的复合末端

arm3、arm4 的 `gripper_with_wrench` 使用六个部件的 mesh 体积积分计算质心和完整惯性张量。每个部件内部按均匀密度处理，质量采用 URDF 中的已知值；通过安装变换和平行轴定理合成到驱动原 `link6` 坐标系。传感器、两片法兰、夹爪底座和两片手指的名义总质量为 **0.685505 kg**。

该末端的重力辨识保留已有机械臂各轴 `scale / angle_offset / bias`，将相机等未建模附件统一辨识为一个全臂共享的 `added_mass_kg`，只加到 `gripper_base_without_flange.STL`，不改变传感器、法兰和手指的质量。每次拟合共同使用已采集的各轴双向半和数据；J1、关闭重力补偿的 J6 不提供质量信息，仍可测量静摩擦。保存的 `arm_X_gravity.yaml` 包含 `effector_model` 和 `added_mass_kg`，驱动据此更新总质量、总质心及惯量。

附加质量的重力矩对质量呈线性关系：`g(q) = g_nominal(q) + added_mass_kg * g_gripper_per_kg(q)`。相机的等效质心与单位质量惯量采用夹爪底座 mesh；准静态重力数据只能辨识这里的等效质量，不能独立测出相机真实的惯性张量。CSV 必须包含实际采样姿态 `q1..q6`。Web 工作台会显示附加质量和末端总质量；写入驱动当前使用的标定文件后会请求驱动重新加载。

更新统一配置中的 `gripper_with_wrench` mesh 质量属性，保留其余末端参数：

```bash
python3 -m piper_x_driver.mesh_inertia \
  src/piper_description/urdf/piper_description_with_wrench.urdf \
  --reference-urdf src/piper_description/urdf/piper_description.urdf \
  --effector gripper_with_wrench \
  --output src/piper_x_driver/config/effectors.yaml
```

---

## 2. 双向准静态逼近数学模型

标定期间保持驱动基础重力支撑开启（`gravity_scale = 1.0`）、关闭摩擦补偿（`friction_scale = 0.0`），先把非测试关节平滑移至安全抬起锚点 `SAFE_ANCHOR_POSE = (0.0, 0.90, -1.50, 0.0, 0.0, 0.0)`（测试 J2～J6 时保持 J1 当前偏航角不变，防止打到桌面），再在 MIT 刚度 $k_{p,\text{realized}}$ 下以余弦 S 曲线（`0.05 rad/s`）让待测关节分别从正方向（$+$）和反方向（$-$）缓慢逼近目标点 $q^*$ 并停稳（$|\dot{q}| \le 0.03\text{ rad/s}$）。

直接从驱动发布的 `/arm_X/control_state` 读取实发重力支撑力矩 $\tau_{g,\text{applied}}$，重构正反向准静态平衡力矩：
- **正向逼近停稳力矩**：$\tau^+(q) = \tau_{g,\text{applied}}(q^+) + k_{p,\text{realized}}(q^* - q^+) - k_{d,\text{realized}}\dot{q}^+ = \tau_g(q) + F_s(q)$
- **反向逼近停稳力矩**：$\tau^-(q) = \tau_{g,\text{applied}}(q^-) + k_{p,\text{realized}}(q^* - q^-) - k_{d,\text{realized}}\dot{q}^- = \tau_g(q) - F_s(q)$

对同一角度网格上的 $\tau^+(\theta)$ 与 $\tau^-(\theta)$ 求**半和**与**半差**：
- **半和（提取真实重力矩，静摩擦正负抵消）**：
  $$\tau_{g,\text{meas}}(\theta) = \frac{\tau^+(\theta) + \tau^-(\theta)}{2}$$
- **半差（提取静摩擦，重力完全抵消）**：
  $$F_s(\theta) = \frac{|\tau^+(\theta) - \tau^-(\theta)|}{2}$$

标定后的单关节重力模型形式为（详见 [gravity.py](../piper_x_driver/piper_x_driver/gravity.py)）：
$$\tau_{g, j}^{\text{cal}}(q) = \text{scale}_j \cdot \tau_{g, j}^{\text{CAD}}(q + \text{angle\_offset}), \qquad \text{bias}_j \equiv 0$$
- **直立轴 `J1`**：不受重力影响，锁定 `scale = 1.0, angle_offset = 0.0, bias = 0.0`；
- **末端同心轴 `J6`**：近乎同心且剛度小，默认关闭重力补偿（锁定 `scale = 0.0, angle_offset = 0.0, bias = 0.0`），仅保留静摩擦测量值。

---

## 3. 使用流程（支持四臂并行标定）

### 方式一：可视化 Web 标定工作台 (`web`)

```bash
# 终端 1：启动四臂驱动
ros2 launch piper_x_driver four_arms.launch.py

# 终端 2：启动重力与静摩擦标定 Web 工作台（默认监听 http://127.0.0.1:8091/）
ros2 run piper_gravity web --port 8091
```

- 支持在网页顶部切换 `机械臂 1 ~ 4` 并**多臂同时并行执行** `J1 → J6` 阶梯逼近队列；
- 每个关节测完后自动回位至安全锚点角 `SAFE_ANCHOR_POSE`，并实时拟合并保存至 `src/piper_gravity/config/arm_X_gravity.yaml`。

### 方式二：命令行采集、拟合与评估 (`probe` / `fit` / `evaluate`)

```bash
# 1. 在线执行单关节双向准静态阶梯逼近采样并直接拟合保存
ros2 run piper_gravity probe \
  --namespace /arm_1 \
  --arm-id arm_1 \
  --joint 2 \
  --q-min 0.55 \
  --q-max 1.20 \
  --num-points 9 \
  --output-csv /tmp/arm1_j2_gravity_probe.csv \
  --fit-output src/piper_gravity/config/arm_1_gravity.yaml \
  --update-friction-yaml src/piper_friction/config/arm_1_friction.yaml

# 2. 从已有探针或扫描 CSV 批量拟合整臂重力模型
ros2 run piper_gravity fit \
  /tmp/arm1_j*_gravity_probe.csv \
  --arm-id arm_1 \
  --output src/piper_gravity/config/arm_1_gravity.yaml

# 3. 评估给定六轴角度下的标定重力矩
ros2 run piper_gravity evaluate \
  --model src/piper_gravity/config/arm_1_gravity.yaml \
  --position 0.0 0.9 -1.5 0.0 0.0 0.0

# arm3 的复合末端：从 J2～J5 的数据共同拟合夹爪附加质量
ros2 run piper_gravity fit /tmp/arm3_j*_gravity_probe.csv \
  --arm-id arm_3 --output src/piper_gravity/config/arm_3_gravity.yaml
```

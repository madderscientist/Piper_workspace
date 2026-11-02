# PiPER-X 关节摩擦力与周期性齿槽/啮合脉动标定 (`piper_friction`)

针对关节**“慢速有明显齿轮状周期阻碍、快速消失”**的特性，本包提供基于 ROS 2 的多速正反转扫描采集、正反半和/半差解耦、空间域谐波（FFT + 最小二乘）与 Stribeck/衰减包络拟合，以及六轴摩擦力矩评估。

## 1. 物理背景与数学模型详解

### 1.1 为什么会有“慢速像齿轮卡顿、快速却消失”的手感？

机械臂关节由**永磁同步电机（PMSM）+ 减速器**组成。当你驱动或拖动关节时，关节内部的阻力并非恒定值，而是由**随速度变化的宏观摩擦**与**随空间角度周期性起伏的微观脉动（波纹）**叠加而成：
- **慢速时**：关节动能极小，且齿面润滑油膜尚未建立（静摩擦远大于动摩擦）。每遇到一个电机齿槽磁吸峰或齿轮啮合紧点，速度就会跌落甚至短暂卡住，需要积蓄更大的力才能翻过这一个“齿格”，因此呈现出明显的**周期性齿轮阻碍感**。
- **快速时**：齿面建立起流体润滑油膜，且连杆与转子转动惯量 $J$ 对高频力矩波动起到天然的二阶低通滤波作用，直接滑过波峰，周期性卡顿感随之消失。

---

### 1.2 总摩擦力矩公式全景拆解

单关节摩擦力矩模型（代码实现见 [model.py](piper_friction/model.py)）将关节角 $\theta$（单位：$\text{rad}$）与角速度 $\dot{\theta}$（单位：$\text{rad/s}$）映射为摩擦阻力矩 $\tau_{\text{fric}}(\theta, \dot{\theta})$（单位：$\text{N}\cdot\text{m}$）：

$$
\tau_{\text{fric}}(\theta, \dot{\theta}) =
\underbrace{
\Big[
  \overbrace{F_c + (F_s - F_c)\, e^{-\left(|\dot{\theta}| / v_s\right)^\delta}}^{\text{① Stribeck 干摩擦项}}
  + \overbrace{w(\dot{\theta})\, \tau_{\text{mod}}(\theta)}^{\text{② 啮合摩擦调制项}}
\Big]
\overbrace{\tanh\!\left(\frac{\dot{\theta}}{v_0}\right)}^{\text{③ 方向与零速平滑项}}
+ \overbrace{F_v \dot{\theta}}^{\text{④ 粘滞阻尼项}}
}_{\text{A. 与运动方向相关的摩擦分量（正反转时力矩方向相反）}}
\;+\;
\underbrace{
\overbrace{w(\dot{\theta})}^{\text{⑤ 高速衰减包络}}\, \overbrace{\tau_{\text{cog}}(\theta)}^{\text{⑥ 齿槽转矩项}}
}_{\text{B. 与方向无关的位置势能分量（正反转方向相同）}}
$$

下面逐项解释公式中每一个部分的物理意义与数学作用：

---

#### ① Stribeck 干摩擦项：$F_c + (F_s - F_c)\, e^{-\left(|\dot{\theta}| / v_s\right)^\delta}$
中学物理常把滑动摩擦视为常数 $F_c$，但涂有润滑脂的减速器在低速向高速过渡时遵循 **Stribeck 曲线**（摩擦随速度升高**先下降、后上升**）：
- **$F_s$（静摩擦峰值，`fs`，单位 $\text{N}\cdot\text{m}$）**：当关节刚起步（$|\dot{\theta}| \to 0$）时，齿轮接触面没有油膜，金属微凸体直接接触，阻力最大。此时指数项 $e^0 = 1$，干摩擦项等于 $F_c + (F_s - F_c)\times 1 = F_s$。
- **$F_c$（库仑动摩擦，`fc`，单位 $\text{N}\cdot\text{m}$）**：当速度升高后（$|\dot{\theta}| \gg v_s$），油膜建立，指数项 $e^{-\infty} \to 0$，干摩擦回落并稳定在较低的动摩擦平台值 $F_c$（通常 $F_s > F_c$）。
- **$v_s$（Stribeck 特征速度，`vs`，单位 $\text{rad/s}$）**：决定“从高静摩擦 $F_s$ 跌落到低动摩擦 $F_c$”的速度拐点。当速率达到 $|\dot{\theta}| = v_s$ 时，高出的静摩擦部分 $(F_s - F_c)$ 衰减为原来的 $1/e \approx 36.8\%$。
- **$\delta$（Stribeck 形状指数，`delta`，默认 $2.0$）**：控制衰减曲线的形状。取 $\delta = 2$ 时，指数项为高斯型 $e^{-(|\dot{\theta}|/v_s)^2}$，在零速附近曲线顶部平坦光滑。
> **直观理解**：在 $0 < |\dot{\theta}| < v_s$ 的低速区内，“速度越慢，摩擦阻力反而越大”（力矩对速度的导数为负，等效于**负阻尼**），这是导致慢速运动容易一卡一顿（Stick-Slip 粘滑效应）的基础原因。

---

#### ④ 粘滞摩擦（Viscous Friction）项：$F_v \dot{\theta}$
- **$F_v$（粘滞阻尼系数，`fv`，单位 $\text{N}\cdot\text{m}/(\text{rad/s})$）**：当速度较高时，齿轮在润滑脂中高速搅动会受到流体粘性阻力，其大小与角速度 $\dot{\theta}$ 成正比（就像在水里划桨，划得越快阻力越大）。在“摩擦力-速度”曲线的高速段，$F_v$ 就是曲线向上倾斜的斜率。

---

#### ③ 运动方向与零速平滑项：$\tanh\!\left(\frac{\dot{\theta}}{v_0}\right)$
- **为什么要乘方向函数？**：干摩擦永远与运动方向相反——正转（$\dot{\theta} > 0$）时产生正方向阻力矩，反转（$\dot{\theta} < 0$）时产生负方向阻力矩。理论上需要乘以符号函数 $\operatorname{sgn}(\dot{\theta})$。
- **为什么用双曲正切 $\tanh(\dot{\theta}/v_0)$ 代替 $\operatorname{sgn}(\dot{\theta})$？**
  - $\operatorname{sgn}(\dot{\theta})$ 在 $\dot{\theta} = 0$ 处存在从 $-1$ 到 $+1$ 的阶跃跳变。在真机静止时，编码器微小的速度测量噪声会让 $\dot{\theta}$ 在 $\pm 0.0001\text{ rad/s}$ 间波动；若直接用 $\operatorname{sgn}$，补偿力矩会在 $+F_s$ 和 $-F_s$ 之间高频来回抽动（产生抖振）。
  - 双曲正切函数 $\tanh(x) = \frac{e^x - e^{-x}}{e^x + e^{-x}}$ 是一条连续光滑的 S 形曲线：
    - 当 $\dot{\theta} \gg v_0$（正常正转）时，$\tanh(\dot{\theta}/v_0) \to +1$；
    - 当 $\dot{\theta} \ll -v_0$（正常反转）时，$\tanh(\dot{\theta}/v_0) \to -1$；
    - 当 $|\dot{\theta}| < v_0$（零速死区附近）时，$\tanh(\dot{\theta}/v_0) \approx \frac{\dot{\theta}}{v_0}$，平滑过零点而不发生跳变。
- **$v_0$（零速平滑阈值，`v0`，单位 $\text{rad/s}$）**：决定零速附近平滑过渡带的宽度（默认约 $0.005 \sim 0.01\text{ rad/s}$）。

---

#### ⑥ 与 ② 两个空间周期项：为什么同样是“齿轮感”，要拆成 $\tau_{\text{cog}}(\theta)$ 和 $\tau_{\text{mod}}(\theta)$？

虽然手感上都是“每转过固定小角度就遇到一次周期性阻碍”，但关节内部同时存在**两种物理机制截然不同**的空间周期力，它们在**正反转切换时的符号规律完全相反**：

1. **⑥ 方向无关的齿槽转矩 $\tau_{\text{cog}}(\theta)$（Cogging Torque）**
   - **物理成因**：电机转子永磁体与定子硅钢片齿槽之间存在磁吸力。即使不通电，转子也倾向于停在磁阻最小的齿槽对齐点。当你把转子从对齐点拉开时，磁力把你往回拽；当你越过半个齿距滑向下一个槽时，磁力又顺着推你。这就像推车走过波浪形的搓衣板，是一个**只与位置 $\theta$ 有关的保守势能场**（$\tau_{\text{cog}}(\theta) = \frac{dU(\theta)}{d\theta}$）。
   - **为什么“从左往右先正后负、从右往左先负后正”恰恰说明它与速度方向 $\operatorname{sgn}(\dot{\theta})$ 无关？**
     用一个中心在 $\theta = 0$ 的势能“波峰（山坡）”来验证（规定**从左到右 $\longrightarrow$ 为正方向 $+$**，**从右到左 $\longleftarrow$ 为负方向 $-$**）：
     - **山坡左侧（$\theta < 0$，左坡）**：坡面左低右高，磁场本身总是把转子往左谷底拉（$-$）。因此为了抵消磁场力，在 $\theta < 0$ 处补偿力矩必须**向右推（$\tau_{\text{cog}}(\theta < 0) > 0$，恒为正 $+$）**。
     - **山坡右侧（$\theta > 0$，右坡）**：坡面左高右低，磁场本身总是把转子往右谷底拉（$+$）。因此为了抵消磁场力，在 $\theta > 0$ 处补偿力矩必须**向左推（$\tau_{\text{cog}}(\theta > 0) < 0$，恒为负 $-$）**。
     - 现在看两个运动方向经过这座山峰时的**时间先后顺序**：
       1. **从左往右运动（$\theta: -1 \to +1$）**：**先**经过左坡（$\theta < 0$，补偿为 **正 $+$**，帮你推上左坡），**后**经过右坡（$\theta > 0$，补偿为 **负 $-$**，拉住你防止顺右坡溜车）$\implies$ 时间上正好是 **先正后负**！
       2. **从右往左运动（$\theta: +1 \to -1$）**：因为倒着走，你**先**踩到的是右坡（$\theta > 0$，此时右坡对你来说是上坡，需要向左即负方向的推力，补偿为 **负 $-$**），越过山顶后**后**踩到左坡（$\theta < 0$，拉住你防止向左溜下坡，补偿为 **正 $+$**）$\implies$ 时间上自动变成 **先负后正**！
     - **关键结论**：因为在**同一个空间坐标点 $\theta$** 上（例如左坡 $\theta = -1$ 处），无论你是从左往右上坡、还是从右往左下坡，山坡把你往左拉的磁力方向从未改变，补偿力矩在该坐标点永远是 $+1$。正因为从右往左运动时**先经过右坡 $\theta=+1$、后经过左坡 $\theta=-1$（访问空间坐标的先后顺序反过来了）**，**不乘 $\operatorname{sgn}(\dot{\theta})$ 的纯位置函数 $\tau_{\text{cog}}(\theta)$** 在时间上就会自然呈现“从左往右先正后负、从右往左先负后正”！反之，如果错误地乘上 $\operatorname{sgn}(\dot{\theta})$，从右往左时负负得正，反而会变成错误的“先正后负”。

2. **② 方向相关的啮合摩擦调制 $\tau_{\text{mod}}(\theta)$（Meshing Friction Modulation）**
   - **物理成因**：减速器齿轮一齿接一齿啮合时，齿面接触刚度和法向正压力 $N(\theta)$ 随角度周期性涨落（例如单齿啮合与双齿啮合交替、齿形微观误差）。正压力变大时，滑动摩擦力 $\mu N(\theta)$ 随之变大；正压力变小时，摩擦力随之变小。
   - **为什么它必须乘 $\tanh(\dot{\theta}/v_0)$？**：因为它本质上是**摩擦力幅值的周期性波动**，永远阻碍运动——正转经过啮合紧点时阻力为 $+\tau_{\text{mod}}(\theta)$，而反转经过同一个啮合紧点时阻力必然反号，变为 $-\tau_{\text{mod}}(\theta)$。

3. **如何用空间傅里叶级数（Spatial Fourier Series）表达这两项？**
   - 常见的信号分析是对时间 $t$ 做傅里叶展开 $\cos(2\pi f t)$；但电机齿槽和减速器轮齿是刻在**空间角度 $\theta$** 上的——每转过固定的角度就经过一个齿，与转快转慢无关。
   - 因此，我们以关节角度 $\theta$（单位 $\text{rad}$）为自变量，用 $M$ 阶空间傅里叶级数展开这两个周期函数：
     $$
     \tau_{\text{cog}}(\theta) = \sum_{m=1}^{M} \Big( a_m \cos(\omega_m \theta) + b_m \sin(\omega_m \theta) \Big)
     $$
     $$
     \tau_{\text{mod}}(\theta) = \sum_{m=1}^{M} \Big( c_m \cos(\omega_m \theta) + d_m \sin(\omega_m \theta) \Big)
     $$
   - **空间角频率 $\omega_m$（`omega`，单位 $\text{rad}^{-1}$，即每 $2\pi$ 弧度的周期数）**：
     代表**关节输出端每旋转一整圈（$2\pi\text{ rad} = 360^\circ$）所包含的“齿格”波峰数量**。
     - 例如：当 $\omega_1 = 36$ 时，表示关节每转一圈有 $36$ 个周期（每个周期跨度为 $\frac{2\pi}{36}\text{ rad} = 10^\circ$）。若同时存在二倍频谐波 $\omega_2 = 72$，则可以刻画非对称或尖锐的齿峰波形。
   - **余弦与正弦系数 $(a_m, b_m)$ 与 $(c_m, d_m)$**：
     由三角恒等式 $a_m \cos(\omega_m \theta) + b_m \sin(\omega_m \theta) = A_m \cos(\omega_m \theta - \phi_m)$：
     - 齿槽第 $m$ 阶谐波幅值为 $|A_m| = \sqrt{a_m^2 + b_m^2}$，初始相位为 $\phi_m = \operatorname{atan2}(b_m, a_m)$；
     - 啮合调制第 $m$ 阶谐波幅值为 $|C_m| = \sqrt{c_m^2 + d_m^2}$，初始相位为 $\psi_m = \operatorname{atan2}(d_m, c_m)$。
     - 展开成 $a_m \cos + b_m \sin$ 的线性组合后，一旦由 FFT 确定了频率 $\omega_m$，求解 $(a_m, b_m, c_m, d_m)$ 就变成标准的**线性最小二乘问题**，一步即可算出精确全局最优解。

---

#### ⑤ 高速衰减包络：$w(\dot{\theta}) = e^{-\left(\dot{\theta} / v_r\right)^2}$
- **$v_r$（脉动衰减特征速度，`vr`，单位 $\text{rad/s}$）**：控制周期性齿轮脉动随速度升高而退火消失的速度阈值：
  - 当慢速运动（$|\dot{\theta}| \ll v_r$）时，$w(\dot{\theta}) \approx 1$，$\tau_{\text{cog}}(\theta)$ 和 $\tau_{\text{mod}}(\theta)$ 完整生效，抵消慢速下明显的“一格一格”阻碍；
  - 当快速运动（$|\dot{\theta}| \gg v_r$）时，$w(\dot{\theta}) \to 0$，周期性谐波项自动平滑关闭。
- **为什么工程上必须要有 $w(\dot{\theta})$？**
  除了还原“慢显快消”的物理现象外，更重要的是**防止高速通信延迟导致相位反转**。设总线采样与控制延迟为 $\Delta t$，空间谐波频率为 $\omega_m$，则角度采样的相位滞后为 $\Delta \phi = \omega_m |\dot{\theta}| \Delta t$。高速下若 $\Delta \phi > 90^\circ$，迟到的前馈周期力矩不仅不能抵消波峰，反而会变成正反馈激振。乘上包络 $w(\dot{\theta})$ 可确保高速段绝对平滑稳定。

---

### 1.3 参数辨识原理：正反转“半和 / 半差”是如何把各分量解耦的？

在实测数据中，所有力矩都混在一起。本包（[identification.py](piper_friction/identification.py)）通过**正反等速扫描 + 半和/半差正交解耦**把它们干净地拆开：

1. **扣除实发重力矩与标定惯性力矩（带紧支集余弦置信度窗）**：
   - 先从电机实测力矩 $\tau_{\text{meas}}$ 中减去驱动器实发的标定重力矩 $\tau_g^{\text{cal}}(q)$ 与锚点姿态下的标定连杆惯性力矩 $J_{\text{cal}}\ddot{q}$：
     $$\tilde{\tau}(t) = \tau_{\text{meas}}(t) - \tau_g^{\text{cal}}(q) - J_{\text{cal}}\ddot{q}(t)$$
   - **加速度与稳速置信度加权**：当速度进入目标速度 $\pm 8\%$ 且 $|\ddot{q}| \le \alpha_{\text{steady}}$ 时完全信任（权重 $w = 1.0$）；在匀速段边缘的小加速度过渡区（$\alpha_{\text{steady}} < |\ddot{q}| < \alpha_{\max}$ 且速度偏差 $< 25\%$）按紧支集余弦窗 $w = \frac{1}{2}(1 + \cos(\pi u))$ 平滑拓展有效样本；一旦 $|\ddot{q}| \ge \alpha_{\max}$ 或速度偏差 $\ge 25\%$ 则硬截断剔除（$w = 0$）。
2. **正、反方向同速扫描同一角度区间**：
   让关节以恒定速率 $v > 0$ 正转（$\dot{\theta} = +v$）和反转（$\dot{\theta} = -v$）扫过同一段角度 $\theta$，并按置信度加权插值对齐到均匀角度网格 $\theta_i$ 上。根据前面的总公式，正转和反转经过**同一个角度 $\theta$** 时的力矩分别为：
   $$
   \tilde{\tau}^+(\theta) = +F(v) + w(v)\,\tau_{\text{mod}}(\theta) + w(v)\,\tau_{\text{cog}}(\theta)
   $$
   $$
   \tilde{\tau}^-(\theta) = -F(v) - w(v)\,\tau_{\text{mod}}(\theta) + w(v)\,\tau_{\text{cog}}(\theta)
   $$
   （其中 $F(v) = F_c + (F_s - F_c)e^{-(v/v_s)^\delta} + F_v v$ 是该速度下的稳态平均摩擦）。
3. **两式相加除以 2（求半和 $\tau_{\text{even}}$）$\longrightarrow$ 提取齿槽转矩 $\tau_{\text{cog}}(\theta)$**：
   $$
   \tau_{\text{even}}(\theta) = \frac{\tilde{\tau}^+(\theta) + \tilde{\tau}^-(\theta)}{2} = w(v)\,\tau_{\text{cog}}(\theta)
   $$
   正负相反的摩擦项被**完全抵消**！通过对全部慢速档（$\le 0.16\text{ rad/s}$）做空间相干平均与空间 FFT（上限 $\omega_{\max} = 120\text{ rad}^{-1}$）滤除时域粘滑抖动并提取真实齿轮频率 $\omega_m$，再用最小二乘回归即可求出 $(a_m, b_m)$。
4. **两式相减除以 2（求半差 $\tau_{\text{odd}}$）$\longrightarrow$ 提取平均摩擦 $F(v)$ 与啮合调制 $\tau_{\text{mod}}(\theta)$**：
   $$
   \tau_{\text{odd}}(\theta) = \frac{\tilde{\tau}^+(\theta) - \tilde{\tau}^-(\theta)}{2} = F(v) + w(v)\,\tau_{\text{mod}}(\theta)
   $$
   方向相同的齿槽转矩（以及任何与方向无关的重力模型残差）被**完全抵消**！
   - 对 $\tau_{\text{odd}}(\theta)$ **在整个角度区间上求均值**：由于周期函数 $\tau_{\text{mod}}(\theta)$ 的均值为 $0$，算出的均值就是该速度下的稳态摩擦力 $F(v)$。采集多组不同速度 $(v_k, F(v_k))$ 后即可拟合出 Stribeck 曲线参数 $(F_c, F_s, v_s, F_v)$。
   - 将 $\tau_{\text{odd}}(\theta)$ **减去均值 $F(v)$**：剩下的零均值周期波动就是 $w(v)\,\tau_{\text{mod}}(\theta)$，用最小二乘回归即可求出 $(c_m, d_m)$。
5. **跨速度投影拟合衰减速度 $v_r$**：
   对比极低速到高速下周期脉动幅值的衰减比例，拟合高斯包络 $w(v) = e^{-(v/v_r)^2}$ 得到 $v_r$，并将低速谐波系数还原到 $v=0$ 处的真实幅值。

---

### 1.4 标定结果 YAML 参数速查表

| YAML 字段 | 数学符号 | 物理名称 | 单位 | 物理含义与调参直观影响 |
| :--- | :---: | :--- | :---: | :--- |
| `fc` | $F_c$ | 库仑动摩擦 | $\text{N}\cdot\text{m}$ | 越过低速区后的基础滑动摩擦平台值 |
| `fs` | $F_s$ | 静摩擦峰值 | $\text{N}\cdot\text{m}$ | 零速起步瞬间的最大摩擦阻力（通常 $F_s \ge F_c$） |
| `vs` | $v_s$ | Stribeck 特征速度 | $\text{rad/s}$ | 摩擦力从 $F_s$ 下降到 $F_c$ 的过渡速度拐点 |
| `delta` | $\delta$ | Stribeck 形状指数 | 无量纲 | 过渡曲线指数（默认 `2.0`，对应高斯型过渡） |
| `fv` | $F_v$ | 粘滞摩擦系数 | $\text{N}\cdot\text{m}/(\text{rad/s})$ | 与角速度成正比的流体阻尼斜率 |
| `v0` | $v_0$ | 零速平滑阈值 | $\text{rad/s}$ | $\tanh(\dot{\theta}/v_0)$ 的平滑宽度，越小过零越陡、越大越柔和 |
| `vr` | $v_r$ | 周期脉动衰减速度 | $\text{rad/s}$ | “齿轮感”随速度升高而消失的特征速度（$|\dot{\theta}| \gg v_r$ 时关闭谐波项） |
| `harmonics[].omega` | $\omega_m$ | 空间角频率 | $\text{rad}^{-1}$ | 关节每旋转一整圈（$2\pi\text{ rad}$）出现的波峰周期数 |
| `harmonics[].a`, `b` | $a_m, b_m$ | 齿槽转矩余弦/正弦系数 | $\text{N}\cdot\text{m}$ | 方向无关的电机齿槽脉动 $\tau_{\text{cog}}(\theta)$，幅值为 $\sqrt{a_m^2+b_m^2}$ |
| `harmonics[].c`, `d` | $c_m, d_m$ | 啮合调制余弦/正弦系数 | $\text{N}\cdot\text{m}$ | 方向相关的减速器啮合摩擦波动 $\tau_{\text{mod}}(\theta)$，幅值为 $\sqrt{c_m^2+d_m^2}$ |

## 2. 全流程可视化 Web 工作台 (`web`)

启动自带的 Web 标定工作台（默认监听 `http://127.0.0.1:8090/`，也可直接在浏览器打开 [web/index.html](web/index.html) 体验离线演示与 CSV 拟合）：

```bash
# 终端 1：启动驱动
ros2 launch piper_x_driver four_arms.launch.py

# 终端 2：启动摩擦力标定 Web 工作台（默认端口 8090）
ros2 run piper_friction web --port 8090
```

在浏览器访问 `http://127.0.0.1:8090/`，页面按五步引导全流程操作：
1. **步骤 ① 选择 CAN 口（单臂）与姿态检查**：一次选定一个 CAN 口（`can0`～`can3`），查看六轴实时位置、速度、力矩与理论重力矩 $\tau_g(q)$。
2. **步骤 ② 配置 J1～J6 扫描计划**：为避免多轴同时运动引入科氏力、离心力及动态重力耦合，系统采用**“同一时刻动一轴、其余五轴锁死”**策略；支持勾选 `J1 → J6` **一键按序自动跑完队列**，也支持点击任意单轴 `▶ 仅扫 Jx` 独立重标。
3. **步骤 ③ 在线正反转等速扫描采集（或离线导入 CSV / 示例数据）**：实时显示队列关节进度、子段进度条及采样帧数。
4. **步骤 ④ 半和/半差解耦与空间谐波 + Stribeck 拟合可视化**：切换 `J1`～`J6` 标签查看三幅核心物理曲线（正反转与半和 $\tau_{\text{cog}}(\theta)$、半差去均值 $\tau_{\text{mod}}(\theta)$、Stribeck 曲线 $F(v)$ 与高速衰减包络 $w(v)$）及空间谐波频谱表。
5. **步骤 ⑤ 前馈补偿力矩实时试算与整臂 YAML 导出**：拖动角度 $\theta$ 与速度 $\dot{\theta}$ 滑块实时观察慢速/快速下各分量力矩占比，一键保存或下载整臂 YAML 参数文件。

## 3. 命令行使用流程

### 步骤 1：启动驱动并执行单关节多速扫描采集 (`sweep`)

确保目标机械臂 `piper_x_driver` 正在运行（例如 `/arm_1`），运行 `sweep` 驱动待测关节在指定角度区间 `[--q-min, --q-max]` 内按多档恒定速度正反往复运动并记录 CSV（可直接加 `--fit-output` 同步产出或更新 YAML 参数文件）：

```bash
# 终端 1：启动驱动
ros2 launch piper_x_driver four_arms.launch.py

# 终端 2：扫描采集 J1（-0.6 ~ +0.6 rad）并直接拟合并写入 config/arm_1_friction.yaml
ros2 run piper_friction sweep \
  --namespace /arm_1 \
  --arm-id arm_1 \
  --joint 1 \
  --q-min -0.6 \
  --q-max 0.6 \
  --speeds 0.03 0.06 0.12 0.25 0.5 1.0 1.5 \
  --output-csv /tmp/arm1_j1_sweep.csv \
  --fit-output /tmp/arm1_friction.yaml
```

### 步骤 2：离线重拟合或多关节合并拟合 (`fit`)

可对一个或多个已采集的 CSV 文件独立运行 `fit`，调整谐波阶数 `--max-harmonics` 或指定已知空间频率 `--omegas`：

```bash
ros2 run piper_friction fit \
  /tmp/arm1_j1_sweep.csv /tmp/arm1_j2_sweep.csv \
  --arm-id arm_1 \
  --max-harmonics 2 \
  --output /tmp/arm1_friction.yaml
```

### 步骤 3：计算补偿力矩 (`evaluate` 或 Python API)

命令行快速验证六轴摩擦力矩（单位：$\text{N}\cdot\text{m}$）：

```bash
ros2 run piper_friction evaluate \
  --model /tmp/arm1_friction.yaml \
  --position 0.1 0.5 -0.5 0.0 0.0 0.0 \
  --velocity 0.05 0.0 0.0 0.0 0.0 0.0
```

在 Python 代码中加载并计算六轴摩擦力矩：

```python
from piper_friction import load_friction_model

model = load_friction_model('/tmp/arm1_friction.yaml')
tau_fric = model.torques(positions, velocities, cogging_scale=1.0, friction_scale=0.8)
```

---

## 4. 夹爪非周期空间摩擦力模型、标定与启用说明

### 4.1 为什么夹爪不能沿用六关节的周期谐波模型？

与 J1～J6 旋转关节（永磁同步电机 + 谐波/行星减速器，阻力具有固定空间角频率 $\omega_m$ 的周期性）不同，末端夹爪 / 示教手柄在行程 $w \in [0, w_{\max}]$（单位 $\text{m}$）上采用**连杆 / 齿轮齿条 / 平行滑块导轨机构**，其空间阻力分布是**非周期的**：
1. **非线性传动比（连杆雅可比 $J(w)$）**：电机转角到指尖平行开度 $w$ 的等效传动比随开度连续变化；
2. **导轨接触应力不均**：闭合端、中段与张开端的滑块倾覆力矩及装配预紧力不同（例如两端紧、中间松）；
3. **非周期位置偏置力 $F_{\text{bias}}(w)$**：复位弹簧或连杆弹性死点产生随开度变化的单向偏置力。

因此，夹爪模型 `GripperFrictionModel`（实现见 [`piper_x_driver/friction.py`](../src/piper_x_driver/piper_x_driver/friction.py)）不再做 FFT 谐波展开，而是采用**一维开度网格分段线性插值剖面（Non-periodic Spatial Profile）+ Stribeck 速度模型**：

$$
F_{\text{grip}}(w, \dot{w}) =
\underbrace{
  \Big[
    \max\!\big(0,\; F_c + (F_s - F_c)e^{-(|\dot{w}|/v_s)^\delta} + e^{-(\dot{w}/v_r)^2}\Delta F_c(w)\big)\tanh\!\left(\frac{\dot{w}}{v_0}\right)
    + F_v \dot{w}
  \Big]
}_{\text{A. 方向相关的非周期动/静摩擦项（由正反半差剖面插值）}}
\;+\;
\underbrace{
  \tanh\!\left(\frac{|\dot{w}|}{v_0}\right) F_{\text{bias}}(w)
}_{\text{B. 方向无关的非周期位置偏置项（由正反半和剖面插值，不做线性 detrend）}}
$$

其中：
- `w_grid`：开度采样节点（单位 $\text{m}$，默认覆盖 `0.003 ~ 0.080 m` 稳态重叠区，共 25 个节点）；
- `bias_grid`：各节点上的方向无关位置偏置力 $F_{\text{bias}}(w_i)$（正反扫描半和 $\frac{1}{2}(f^+(w) + f^-(w))$，完整保留非周期趋势）；
- `modulation_grid`：各节点上的方向相关空间摩擦调制 $\Delta F_c(w_i)$（正反扫描半差去均值 $\frac{1}{2}(f^+(w) - f^-(w)) - \bar{F}_c$）；
- `kp_equiv` 与 `kv_equiv`：由多速扫描期间的稳态跟踪滞后 $\lvert w_{\text{des}} - w_{\text{meas}} \rvert = \Delta w_0 + \lvert \dot{w} \rvert / k_{v,\text{equiv}}$ 联合线性回归辨识出的**固件内环稳态力矩刚度 $k_{p,\text{equiv}} = \text{median}(\lvert f_{\text{meas}} \rvert) / \Delta w_0$（单位 $\text{N/m}$）**与**外环位置-速度比例增益 $k_{v,\text{equiv}}$（单位 $\text{s}^{-1}$）**。

---

### 4.2 统一的夹爪摩擦力补偿公式（主臂 / 从臂 / 双边力控 / 代码控制通用）

夹爪底层通过 CAN `0x159`（`width, effort, status_code`）控制，对应上层 ROS 话题 `/arm_X/command/gripper`（`GripperCommand`）。固件内部采用**位置外环（增益 $k_{v,\text{equiv}}$）+ 速度-力矩内环（刚度 $k_{p,\text{equiv}}$）**的串级结构。每条机械臂统一**仅读取本臂自身 CAN `0x2A8` 反馈的实测开度 $w_{\text{meas}}$ 与实测速度 $\dot{w}_{\text{meas}}$**（经 `30 ms` 滑动窗口估计，静止时严格为 $0$），完全使用标定参数进行补偿（实现见 [`GripperFrictionModel.compensate_command`](../src/piper_x_driver/piper_x_driver/friction.py)）：

#### 1. 由本臂实测开度与速度计算补偿力 $f_{\text{comp}}$

直接使用标定得到的完整非周期空间摩擦 + Stribeck 模型 $F_{\text{grip}}(w_{\text{meas}}, \dot{w}_{\text{meas}})$（含 Stribeck 静-动摩擦、空间调制 $\Delta F_c(w)$、粘滞项 $F_v\dot{w}$ 与非周期位置偏置 $F_{\text{bias}}(w)$，在 $\dot{w}_{\text{meas}} = 0$ 处经 $\tanh(\dot{w}_{\text{meas}}/v_0)$ 严格过零且连续可导）：

$$
f_{\text{comp}}(w_{\text{meas}}, \dot{w}_{\text{meas}}) = s \cdot F_{\text{grip}}(w_{\text{meas}}, \dot{w}_{\text{meas}})
$$

（其中 $s = \text{friction\_scale\_applied}$；当 $\lvert f_{\text{comp}} \rvert < 10^{-3}\text{ N}$ 小于硬件 $1\text{ mN}$ 最小分辨率或静止 $\dot{w}_{\text{meas}} = 0$ 时，$f_{\text{comp}} = 0$。）

#### 2. 统一的力矩上限 $\text{effort}_{\text{out}}$ 与位置死区补偿 $w_{\text{out}}$

与六轴关节（J1～J6）在 `driver` 内部根据 `kp == 0`（零刚度被动拖动）或 `kp > 0`（有刚度主动跟踪）自动切换重力补偿路径完全对称，夹爪在 CAN `0x159` 中没有 `kp` 字段，**`effort_cmd` 即为夹爪唯一表征“有无外部主动保持力”的物理量**。因此 `driver` 内部直接根据 `effort_cmd <= 0`（被动自由拖动 / 主臂）与 `effort_cmd > 0`（主动控制 / 从臂 / 双边力控）确定基准目标位置 $w_{\text{target}}$，使上层节点（如 `TeachingNode`）无需感知底层摩擦标定参数，且单臂不启动示教节点时也能独立享有被动捏合补偿：

- **基准目标位置 $w_{\text{target}}$（为什么 `effort_cmd <= 0` 需要加一项速度折算位置 $\dot{w}_{\text{meas}} / k_{v,\text{equiv}}$？）**：
  - 夹爪固件内部是**“位置外环（增益 $k_v$）$\to$ 速度-力矩内环（刚度 $k_p$）”**的串级控制器：
    1. 位置外环先根据位置差算出固件内部期望速度：$\dot{w}_{\text{ref}} = k_{v,\text{equiv}} \cdot (w_{\text{out}} - w_{\text{meas}})$；
    2. 速度-力矩内环再根据速度差 $(\dot{w}_{\text{ref}} - \dot{w}_{\text{meas}})$ 输出电机力矩。**只有当内部期望速度 $\dot{w}_{\text{ref}} > \dot{w}_{\text{meas}}$ 时，内环才会产生顺向助推力；若 $\dot{w}_{\text{ref}} < \dot{w}_{\text{meas}}$，内环反而会输出反向制动力（电感/阻尼刹车）！**
  - 当 **`effort_cmd > 0`（主动控制 / 从臂 / 双边力控）** 时，外部或主臂给定的目标位置 $w_{\text{cmd}}$ 本身就超前于当前实测位置 $w_{\text{meas}}$（在速度 $\dot{w}$ 下天然保持 $\ge \dot{w} / k_{v,\text{equiv}}$ 的跟踪超前量），因此直接使用 $w_{\text{target}} = w_{\text{cmd}}$ 即可，无需也不能再叠加自身实测速度（否则会将从臂输出速度正反馈回输入位置引发自激）。
  - 当 **`effort_cmd <= 0`（主臂 / 被动自由拖动）** 时，上层没有外部轨迹输入（每帧传入的基准位置就是当前实测位置 $w_{\text{meas}}$，位置差恒为 $0$）。若不补偿速度环超前量，当人手以速度 $\dot{w}_{\text{meas}}$ 捏动夹爪时，固件位置外环算出的期望速度 $\dot{w}_{\text{ref}} \approx 0 < \dot{w}_{\text{meas}}$，内环会把人手运动当成“超速”而反向制动，完全无法输出顺向摩擦补偿力。因此必须加上速度折算位置 $s \cdot \dot{w}_{\text{meas}} / k_{v,\text{equiv}}$，使固件外环算出的 $\dot{w}_{\text{ref}}$ 恰好同步跟上人手当前速度 $\dot{w}_{\text{meas}}$（消除内环阻尼拖累），再由 $\dfrac{f_{\text{comp}}}{k_{p,\text{equiv}}}$ 产生正向补偿推力。
- **统一的摩擦位置偏置与力矩补偿**（对主动代码控制、从臂、双边力控、主臂完全一致）：

$$
\begin{aligned}
w_{\text{out}} &= \begin{cases}
w_{\text{cmd}}, & \text{effort}_{\text{cmd}} > 0 \quad \text{（代码主动控制 / 从臂 / 双边力控：保持外部目标位置）} \\[4pt]
w_{\text{meas}} + \dfrac{f_{\text{comp}}}{k_{p,\text{equiv}}} + \dfrac{\dot{w}_{\text{meas}}}{k_{v,\text{equiv}}}, & \text{effort}_{\text{cmd}} \le 0 \quad \text{（主臂 / 被动自由拖动：瞬态比例饱和 + 抵消速度外环滞后）}
\end{cases} \\[8pt]
\text{effort}_{\text{out}} &= \max(0,\; \text{effort}_{\text{cmd}}) + \lvert f_{\text{comp}} \rvert
\end{aligned}
$$

| 运行场景 | 上层输入的 `(w_cmd, effort_cmd)` | 最终下发给 `0x159` 的 `(w_out, effort_out)` |
|---|---|---|
| **单臂自由拖动 / 主从模式主臂** | `w_cmd = w_meas`, `effort_cmd = 0` | $w_{\text{out}} = w_{\text{meas}} + \dfrac{f_{\text{comp}}}{k_{p,\text{equiv}}} + \dfrac{\dot{w}_{\text{meas}}}{k_{v,\text{equiv}}}, \quad \text{effort}_{\text{out}} = \lvert f_{\text{comp}} \rvert$ |
| **代码主动控制 / 主从从臂 / 双边力控** | `w_cmd`, `effort_cmd > 0` | $w_{\text{out}} = w_{\text{cmd}}, \quad \text{effort}_{\text{out}} = \text{effort}_{\text{cmd}} + \lvert f_{\text{comp}} \rvert$ |

---

### 4.3 夹爪摩擦力标定流程与启用方法

#### 1. 使用独立 Web 专页标定（`gripper.html`）

```bash
# 终端 1：启动驱动
ros2 launch piper_x_driver four_arms.launch.py

# 终端 2：启动摩擦力标定工作台
ros2 run piper_friction web --port 8090
```

浏览器打开 **`http://127.0.0.1:8090/gripper.html`**（或点击首页右上角 **“🦾 夹爪非周期摩擦标定专页 →”**）：
1. 点击选择目标机械臂卡片（选中卡片高亮显示）；
2. 确认扫描行程范围（默认 `0.003 ~ 0.080 m`）、扫描线速度（默认 `0.005 0.010 0.020 0.040 0.080 m/s`）与扫描力限幅（默认 `2.0 N`）；
3. 点击 **“▶ 开始夹爪多速往复扫描采集”**（扫描期间机械臂六个关节保持原状不动，仅夹爪通过 `0x159` 独立往复开合，完成后自动拟合并将 `gripper:` 段合并写入 `config/arm_X_friction.yaml`）。

也可直接使用命令行完成夹爪（`--joint 7`）扫描与拟合：

```bash
ros2 run piper_friction sweep \
  --namespace /arm_1 --arm-id arm_1 --joint 7 \
  --q-min 0.003 --q-max 0.080 \
  --speeds 0.005 0.010 0.020 0.040 0.080 \
  --gripper-effort 2.0 \
  --output-csv /home/a/piper_x_ws/src/piper_friction/config/arm_1_sweep_j7.csv \
  --fit-output /home/a/piper_x_ws/src/piper_friction/config/arm_1_friction.yaml
```

#### 2. 在驱动中启用夹爪摩擦力补偿

1. 确保 [`src/piper_friction/config/arm_1_friction.yaml`](config/arm_1_friction.yaml)～[`arm_4_friction.yaml`](config/arm_4_friction.yaml) 中已包含标定生成的 `gripper:` 节点；
2. 确保 [`src/piper_x_driver/config/arms.yaml`](../src/piper_x_driver/config/arms.yaml) 中对应机械臂的：
   - `friction_config` 指向该 YAML 文件；
   - `friction_scale` 大于 `0`（推荐 `0.85`；注意标定采集开始时会自动将 `friction_scale` 置 `0.0`，标定结束后可在 Web 页面点击 **“恢复 0.85”** 或在 `arms.yaml` 中设回 `0.85`）；
3. 启动或重启驱动（`ros2 launch piper_x_driver four_arms.launch.py`），驱动即会根据本臂实测开度与速度在主动跟随（`effort > 0`）与被动捏合（`effort <= 0`）下自动应用非周期夹爪摩擦力补偿，并在 `/arm_X/control_state` 的 `gripper_friction` 字段中实时上报当前补偿力。


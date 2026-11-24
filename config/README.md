# config — 本机硬件配置与标定数据

本目录存放**本机私有**的硬件配置与标定结果，由 `source ./setup_env.bash` 导出的 `PIPER_CONFIG_DIR` 指向。

除 `examples/` 和本说明文件外，本目录下其余所有文件均被 `.gitignore` 忽略，多台主机可共享同一份代码仓库而各自维护不同的硬件和标定数据。

## 目录布局

```text
config/
├── README.md                  # 本说明（进 git）
├── examples/                  # 配置模板（进 git）
│   ├── env.bash
│   ├── arms/                  # 机械臂配置模板
│   └── <package>/             # 各外设包设备配置模板（如 paxini_driver/）
├── env.bash                   # 本机环境变量，如 ROS_DOMAIN_ID（不进 git）
├── arms/                      # 本机机械臂，一个文件一只臂（不进 git）
│   └── arm_N.yaml
├── <package>/                 # 各外设包本机设备，一个文件一台设备（不进 git）
│   └── <device>.yaml
└── calibration/               # 所有硬件标定结果平铺存放（不进 git）
    ├── arm_N_gravity.yaml
    ├── arm_N_friction.yaml
    └── <device>.yaml
```

## 约定

- **单向关联**：机械臂文件（`arms/*.yaml`）不引用外设；外设文件（`<package>/<device>.yaml`）用 `arm: arm_X` 声明自己装在哪只臂上，并记录本机硬件标识（串口/固件版本/IP/序列号等）。
- **标定路径**：设备文件中用相对路径指向标定结果（如 `../calibration/<name>.yaml`）。未填写、文件不存在或内容无效时，均视为**未标定**（驱动正常启动并提示告警）。
- **与包内配置的边界**：协议默认值、标定板几何尺寸、公共标定姿态等不随机器变化的静态配置留在各功能包的 `config/` 中并进 git；本机硬件列表和标定结果放在本目录。

## 新机器初始化

1. 执行 `source ./setup_env.bash`，会自动创建目录骨架并从 `examples/env.bash` 复制 `env.bash`（不会覆盖已有文件）。
2. 从 `examples/arms/` 复制本机实际接入的机械臂文件到 `arms/`，按本机接线修改 `can_port`。
3. 如有外设，从 `examples/<package>/` 复制对应模板到 `<package>/`，按本机设备修改端口/地址/序列号等字段。
4. 按根目录 `README.md` 中的步骤运行重力、摩擦及外设标定，结果会自动写入 `calibration/`。

# PX6D-75 原始模型存档

- `PX6D-75_official.STEP` 是从[帕西尼官方支持页](https://paxini.com/cn/developer)公开资料取得的 PX6D-75 商业模型原文件，单位mm。
- 其余六个 STL（`housing`、`load_plate`、`fasteners`、`thread_inserts`、`insert_keys`、`connector`）是从该 STEP 转换的详细网格，单位为米。转换时将厂家坐标变为 `(x, y, z) = (y_STEP, -x_STEP, z_STEP + 2 mm)`，使背面安装面位于 `z=0`、接头朝 `-Y`。

当前组件 URDF/Xacro 使用上一级的单个轮廓网格 `meshes/px6d75.stl`。本目录用于保存详细源模型，未接入组件 URDF。

# aloha2 — 硬件机械图纸

本目录是 **ALOHA 2 硬件平台的机械图纸与加工文件**，纯 CAD/3D 模型，不含任何代码。

仓库中的其余部分负责软件侧：[`aloha_scripts/`](../aloha_scripts/) 遥操作与数据采集、[`config/`](../config/) 参数、[`launch/`](../launch/) 启动文件。本目录提供搭建这台实机所需的 3D 打印件与装配参考，用于复现硬件平台。

> 该目录原名 `hardware/`，在提交 `7e55453`（Update name of hardware folder to aloha2）中改为现名。

## 文件清单

### 相机安装件

| 文件 | 说明 |
| --- | --- |
| `Aloha cam wrist mount v13.stl` | 腕部相机支架 |
| `RSd405 2020 Overhead Cam Mount v3.stl` | RealSense D405 俯视相机支架，适配 2020 铝型材 |
| `RSd405 2020 Worms-eye Cam Mount v3.stl` | RealSense D405 鱼眼视角相机支架，适配 2020 铝型材 |

### 重力补偿机构

| 文件 | 说明 |
| --- | --- |
| `Gravity compensator clip, elbow mount.stl` | 配重夹，肘部安装 |
| `Gravity compensator clip, wrist mount.stl` | 配重夹，腕部安装 |
| `Shim_rotor_v1.STL` | 配重垫片 |
| `W-shim_rotor_v1.STL` | 配重垫片（W 型） |

### 夹爪与转接件

| 文件 | 说明 |
| --- | --- |
| `viperx_gripper_stl.zip` | ViperX 夹爪 STL |
| `viperx_joint1_adapter.zip` | ViperX joint1 转接件 |
| `widowx_gripper_std.zip` | WidowX 夹爪 STL |
| `aloha_gripper_assembly.pdf` | 夹爪装配说明 |

### 工作台

| 文件 | 说明 |
| --- | --- |
| `workcell_v2/Aloha workcell v2.stl` | 整套实验台模型（约 97 MB） |

## 备注

- 本目录已被 git 跟踪，不是忽略项。
- STL 文件为二进制网格模型，可用任意 CAD 软件（Fusion 360、SolidWorks、FreeCAD、MeshLab）打开或切片打印。

# aloha_scripts 说明

本目录是 ALOHA 真机系统的 **遥操作 + 数据采集** 层，对应论文 *Learning Fine-Grained Bimanual Manipulation with Low-Cost Hardware*（RSS 2023）的开源实现。它不包含任何学习算法，职责很单一：**让人用手拖动一对小臂，把动作镜像到一对大臂上，同时录下 (观测, 动作) 对，写成 hdf5 供 ACT 训练**。

硬件侧的装配、ROS 环境、udev 端口绑定、夹爪电流上限设置等，见仓库根目录的 `README.md`。本文只讲 `aloha_scripts` 里的代码。

---

## 1. 运行前提

| 项目 | 要求 |
|---|---|
| OS / ROS | Ubuntu 18.04 或 20.04 + ROS 1 noetic |
| 第三方包 | `interbotix_xs_modules`、`interbotix_xs_msgs`、`ros-noetic-usb-cam`、`ros-noetic-cv-bridge` |
| Python 依赖 | `numpy`、`h5py`、`opencv-python`、`matplotlib`、`tqdm`、`dm_env`、`IPython` |
| 硬件 | 4 台 Interbotix 臂（2×`wx250s` 作主臂 + 2×`vx300s` 作从臂）+ 4 路 USB 相机 |
| 启动方式 | 先 `roslaunch aloha 4arms_teleop.launch` 把 4 臂 4 相机拉起来，再跑本目录脚本 |

**两个容易踩的前提：**

1. **必须改上游库**。`interbotix_xs_modules/arm.py` 的 `publish_positions()` 里，把
   `self.T_sb = mr.FKinSpace(self.robot_des.M, self.robot_des.Slist, self.joint_commands)`
   改成 `self.T_sb = None`。不改的话每步都会算一次正运动学，遥操作频率会被拖到不可用。
2. **rospy 节点只能初始化一次**。所以所有脚本里只有第一个被构造的机器人传 `init_node=True`，其余一律传 `init_node=False`。这是贯穿全目录的硬约束，改动时务必保持。

---

## 2. 文件清单

| 文件 | 行数 | 职责 |
|---|---|---|
| `constants.py` | 52 | 全局常数：控制周期、起始姿态、夹爪标定值、归一化函数 |
| `robot_utils.py` | 187 | 底层封装：ROS 订阅、平滑轨迹、力矩与 PID 设置 |
| `real_env.py` | 205 | `RealEnv`，把真机包装成 `reset`/`step` 接口 |
| `record_episodes.py` | 228 | **主入口**，采集一集并写 hdf5 |
| `replay_episodes.py` | 40 | 回放已录 hdf5 的动作序列 |
| `visualize_episodes.py` | 176 | 导出四宫格视频 + 关节曲线/力矩/跟踪误差图 |
| `one_side_teleop.py` | 70 | 单侧遥操作，不录数据，用于硬件调试 |
| `sleep.py` | 19 | 让从臂收拢到休眠姿态 |
| `auto_record.sh` | 14 | 循环调用 `record_episodes.py` 录多集 |

---

## 3. 系统拓扑与数据流

### 3.1 四个角色

```
              操作者双手
                  │ 拖拽
        ┌─────────┴─────────┐
   master_left          master_right        wx250s，全程 torque off
   (只读，当传感器)      (只读)
        │                    │
        │  joint_states      │  joint_states
        ▼                    ▼
   ┌─────────────────────────────────┐
   │   get_action()  →  14 维 action  │
   └─────────────────────────────────┘
        │                    │
        │  set_joint_positions / pub gripper
        ▼                    ▼
   puppet_left          puppet_right       vx300s，全程 torque on
   (执行任务的臂)        (执行任务)
        │                    │
        │  joint_states      │  joint_states
        ▼                    ▼
   ┌─────────────────────────────────┐
   │  Recorder 缓存 → get_observation │
   └─────────────────────────────────┘
                  ▲
   ┌──────────────┴──────────────┐
   │  4× USB 相机 (ImageRecorder) │
   └─────────────────────────────┘
                  │
                  ▼
            hdf5 (观测 + 动作)
```

主臂和从臂是**同一运动学系的同构臂**（WidowX / ViperX 家族均 6 自由度、关节方向约定一致），所以 `wx250s → vx300s` 可以直接做**关节空间镜像**，不需要逆运动学或坐标变换。

### 3.2 用到的 ROS topic

| Topic | 类型 | 方向 | 用途 |
|---|---|---|---|
| `/puppet_{left,right}/joint_states` | `JointState` | 订阅 | 读取从臂真实状态，构成观测 |
| `/puppet_{left,right}/commands/joint_group` | `JointGroupCommand` | 订阅 | 记录下发的臂指令（当前未写入数据集） |
| `/puppet_{left,right}/commands/joint_single` | `JointSingleCommand` | 订阅 | 记录下发的夹爪指令（当前未写入数据集） |
| `/puppet_{left,right}/commands/joint_single` | `JointSingleCommand` | 发布 | 下发夹爪目标 |
| `/usb_cam_{high,low,left_wrist,right_wrist}/image_raw` | `Image` | 订阅 | 四路相机图像 |

主臂**不订阅任何 topic**，只通过 `master_bot.dxl.joint_states.position` 直接读取底层 Dynamixel 状态。

---

## 4. 数据格式

### 4.1 Action（14 维，float64）

```
[0:6]    左臂 6 关节绝对角 (rad)      ← master_left 的 joint_states.position[:6]
[6]      左夹爪，归一化到 [0,1]        ← 0=闭合, 1=张开
[7:13]   右臂 6 关节绝对角 (rad)      ← master_right 的 joint_states.position[:6]
[13]     右夹爪，归一化到 [0,1]
```

注意 action 是**绝对关节目标位置**，不是增量、不是速度。第 6 位取的是主臂**夹爪电机角度**（`position[6]`），经 `MASTER_GRIPPER_JOINT_NORMALIZE_FN` 归一化。

### 4.2 Observation

| 键 | 形状 | 说明 |
|---|---|---|
| `qpos` | (14,) float64 | 左臂 6 关节角 + 左夹爪 + 右臂 6 + 右夹爪 |
| `qvel` | (14,) float64 | 同上维度，关节速度 |
| `effort` | (14,) float64 | 左右各 7 维原始力矩（6 关节 + 夹爪电机），未归一化 |
| `images` | 4× (480, 640, 3) uint8 | BGR 通道序，键为 `cam_high` / `cam_low` / `cam_left_wrist` / `cam_right_wrist` |

**qpos 的夹爪位取的是 `joint_states.position[7]`（手指实际位置），而不是 `[6]`（电机角度）。** 见 `real_env.py` 的 `get_qpos()`。这是有意为之：夹爪是力矩驱动，被物体挡住时电机角度会失真，手指位置才反映真实张合。而 action 侧用的是电机角度，因为那是下发的目标量。两侧用**不同的归一化函数**（`PUPPET_GRIPPER_POSITION_*` vs `MASTER_GRIPPER_JOINT_*`）。

### 4.3 hdf5 结构

```
episode_N.hdf5
├── attrs: sim = False
├── observations/
│   ├── images/{cam_high, cam_low, cam_left_wrist, cam_right_wrist}   (T,480,640,3) uint8, chunk=(1,480,640,3)
│   ├── qpos    (T, 14) float64
│   ├── qvel    (T, 14) float64
│   └── effort  (T, 14) float64
└── action      (T, 14) float64
```

`T` 恒等于 `TASK_CONFIGS[task_name]['episode_len']`（默认 1000，即 50Hz × 20 秒）。图像未压缩（源码里 gzip 那行被注释掉了）——单帧 480×640×3 = 0.92 MB，四路 × 999 帧约 **3.7 GB / 集**，注意磁盘。

---

## 5. 关键机制

### 5.1 主从力矩状态：为什么主臂能被拖动

- **从臂 (puppet)**：`setup_puppet_bot()` 设为 `arm=position` 模式 + `gripper=current_based_position` 模式，`torque_on`。它接受绝对位置指令。
- **主臂 (master)**：`opening_ceremony()` 里先临时设成 `position` 模式并上电，以便**自动归位到起始姿态**；归位完成后立刻 `torque_off`，整个臂变成自由状态，操作者才能用手拖。
- 主臂一旦 torque off，它的 `joint_states.position` 就是操作者手的朝向 —— 这正是 action 的来源。

`one_side_teleop.py` 是同一套逻辑的单臂版本，调试硬件时用它。

### 5.2 "捏夹爪"作为录制开关

`opening_ceremony()` 的最后一段（`record_episodes.py`）是个巧妙的小交互：

1. 单独把**主臂夹爪**的 torque 关掉（臂保持断电，夹爪额外放开）；
2. 打印 `Close the gripper to start`，以 200Hz 轮询两个主臂夹爪位置；
3. 当**两侧都** < `-0.3` 时判定为"捏合"，正式开始录制。

这样操作者摆好物体后不需要碰键盘，捏一下夹爪就开始。`one_side_teleop.py` 有单臂版本（`press_to_start`）。

### 5.3 采集时序：action_t ↔ obs_t 的配对

```python
ts = env.reset(fake=True)     # 拿到 1 个 FIRST timestep
timesteps = [ts]              # 长度 1
actions = []
for t in range(max_timesteps):
    action = get_action(...)  # 读主臂当前姿态
    ts = env.step(action)     # 下发到从臂，再读回观测
    timesteps.append(ts)      # 长度 t+2
    actions.append(action)

# 配对：pop(0) 逐个对齐，丢弃最后一个 timestep
while actions:
    action = actions.pop(0)
    ts = timesteps.pop(0)     # 取的是 step 之前的那个观测
```

结果：`actions[t]` 与 `timesteps[t]` 配对，即数据集里的 **`action[t]` 对应观测 `obs[t]`（而非 `obs[t+1]`）**。最后那个 `step` 返回的 timestep 被丢弃。训练代码必须按这个约定理解数据。

### 5.4 帧率健康检查 —— 不达标的整集重录

`record_episodes.py` 的 `capture_one_episode()` 记录了每步三个时间戳：

```
t0 ─── t1 ─── t2
    读action   step+读观测
```

`print_dt_diagnosis()` 算出真实平均频率，**如果 < 42Hz 就返回 `False`**；`main()` 里的 `while True` 会立刻重录这一集（同一个 episode 序号）。

原因：`DT = 0.02` 名义上是 50Hz，但一旦 USB 带宽不足或某一步卡顿，数据的时间尺度就与训练时的假设不符，直接污染模仿学习。宁可重录。这是整份代码里对数据质量最关键的一处保护。

### 5.5 平滑轨迹

`move_arms()` / `move_grippers()` 不发阶跃指令，而是在 `move_time` 内用 `np.linspace()` 线性插值，每 `DT` 推进一步，`blocking=False`。归位、开合夹爪都走这个路径，防止关节载荷突变。

### 5.6 相机没有时间对齐

`ImageRecorder` 的回调只做两件事：存下最新一帧 cv 图像、存下时间戳。`get_images()` 直接返回"最近一次回调拿到的帧"，**既不查询时间戳也不做插值**。

因此图像与同一时刻的 qpos 之间存在一个未被建模的、随 USB 帧率波动的偏移。这是 ALOHA 数据的已知系统性问题。`visualize_episodes.py` 里有个 `visualize_timestamp()` 就是为排查它准备的，但已随数据格式从 pkl 换成 hdf5 而废弃（未被调用）。

---

## 6. 常用命令

```bash
# 一次性启动 4 臂 4 相机
roslaunch aloha 4arms_teleop.launch

# 采集：先填好 constants.py 里的 DATA_DIR
python3 record_episodes.py --task_name aloha_wear_shoe
python3 record_episodes.py --task_name aloha_wear_shoe --episode_idx 3   # 指定序号，覆盖已有

# 硬件/频率诊断（需手动取消 __main__ 末尾的注释）
python3 record_episodes.py --task_name aloha_wear_shoe   # debug() 分支

# 单侧遥操作调试
python3 one_side_teleop.py left
python3 one_side_teleop.py right

# 回放验证
python3 replay_episodes.py --dataset_dir <dir> --episode_idx 0

# 导出视频 + 曲线
python3 visualize_episodes.py --dataset_dir <dir> --episode_idx 0

# 收工让机械臂趴下
python3 sleep.py
```

`visualize_episodes.py` 会产出四个文件：`episode_N_video.mp4`（四路相机横向拼接，50fps）、`episode_N_qpos.png`（关节状态 vs 指令）、`episode_N_effort.png`（原始力矩）、`episode_N_error.png`（`action - qpos` 跟踪误差）。

---

## 7. 已知问题

改动前先读这一节。以下都是当前代码里的真实缺陷，不是设计选择。

1. **`auto_record.sh` 无法工作。** 它调用 `record_episodes.py --task "$1"`，但脚本定义的参数名是 `--task_name`，会直接以 `unrecognized arguments` 退出；而随后的 `if [ $? -ne 0 ]` 会终止整个循环，所以一集都录不到。此外第 1 行的 `[ "$2" -lt 0 ]` 参数校验对任何合法输入都是假，形同虚设。

2. **相机频率诊断是错的。** `robot_utils.py` 的 `image_cb()`：
   ```python
   getattr(self, f'{cam_name}_timestamps').append(data.header.stamp.secs + data.header.stamp.secs * 1e-9)
   ```
   第二个 `secs` 应为 `nsecs`。`print_diagnostics()` 输出的相机帧率不可信。

3. **没有提前终止。** `for t in range(max_timesteps)` 恒定跑满 1000 步，没有"任务完成就停"的机制。操作者必须在 20 秒内完成任务并保持静止，否则尾部会是冗余的静止帧，且会被当作有效动作参与训练。

4. **`DATA_DIR` 是未填的占位符** `'<put your data dir here>'`，`TASK_CONFIGS` 只有一个 `aloha_wear_shoe` 示例。换任务需要自己加条目，并且 `episode_len` 决定 hdf5 形状，改它等于改变数据的时间跨度。

5. **相机列表硬编码不一致。** `TASK_CONFIGS` 里有 `camera_names`，但 `ImageRecorder.__init__` 内部把四条相机名字**写死了**并逐一建回调。想增减相机必须同时改两处。

6. **`Recorder` 记录了 `arm_command` / `gripper_command` 却从未使用。** 订阅的 `commands/joint_group` 和 `commands/joint_single` 两个 topic 只在内存里缓存，没有写进 hdf5。名义上的"实际下发指令"目前并不在数据集里——数据集里的 action 是主臂姿态，不是从臂收到的指令。

7. **`visualize_joints()` 的图例被覆盖。** 它在同一个 `ax` 上先画 state 再画 command，两次都调 `ax.legend()`，最终只显示 "Command" 一项（曲线本身都在）。作者本意应是每个子图一个 legend。

8. **`visualize_episodes.py` 的 `--episode_idx` 未设默认值也非必填**，不传时会拼出 `episode_None.hdf5` 然后报"文件不存在"退出。

9. **`sleep.py` 里的 `master_sleep_position` 是死代码**，且创建的两个 master bot 从未被使用——只有从臂会被移动。

10. **图像未做压缩或时间同步**（见 5.6），单集磁盘占用约 1.5 GB。

---

## 8. 与其它模块的关系

- **上游硬件/装配说明**：仓库根 `README.md`，`config/`（每个机器人的端口绑定）、`launch/`（4 臂 4 相机的 ROS launch）、`aloha2/`（结构件 CAD）。
- **下游学习算法**：本目录只产数据。ACT 的训练与推理在独立仓库 [tonyzhaozh/act](https://github.com/tonyzhaozh/act) 里，其 `sim_env.py` / gym_aloha 是这里的仿真对应物——两者的 action/observation 语义是刻意保持一致的。

一句话概括：`real_env.py` 定义了数据语义，`record_episodes.py` 定义了采集时序，其余文件都是围绕这两者的 IO 细节与诊断工具。

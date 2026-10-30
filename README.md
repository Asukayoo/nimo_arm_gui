# nimo_arm_gui

本机 HMI 仓库。后端与 GUI **解耦**：远程机器人 workspace 叫 **`nimo_robot`**，不叫 `nimo_arm_gui`。

| 机器 | 角色 | 路径 / 跑什么 |
|------|------|----------------|
| **本机** 192.168.0.92 · Jazzy | hmi | 本仓 `install` → GUI + RViz；拖拽时另起 `rokea_drag_bridge` |
| **nimo-12** · Humble 容器 `teleop` | robot | `/home/mundane/nimo_robot` → `dual_arm_node`；Pico 时另起 `vrcon_ws` |

`ROS_DOMAIN_ID=0`；`nimo_arm_msgs` 两边各编一份。  
**禁止**叠跑旧整机 `teleop_hw` / `nimo_bringup`（抢 `/joint_states`）。

## 概念

| 说法 | 含义 |
|------|------|
| `use_sim:=true` | 臂不连柜 |
| `use_sim:=false` | 真机臂 |
| Pico | **无仿真**；真头显 + PC Service + 真 `nimo_xr` → `ServoPByPico` |
| Rokea 拖拽 | 本机 `rokea_drag_bridge` 开双臂关节拖拽 → `ServoJ`（与 Pico **互斥**） |

## 包

| 包 | 本机 | 远程 `nimo_robot` |
|----|------|-------------------|
| `nimo_arm_msgs` | 编 | 编 |
| `nimo_arm_description` | 编 | 不同步 |
| `nimo_arm_gui` | 编 | 不同步 |
| `nimo_arm_ros` | 不编 | 编（链 `nimo_arm`） |
| `nimo_arm_bringup` | 编 | 编 |

## 一次搭建（本机）

```bash
cd /home/yxc/nimo_arm_gui
# meshes already vendored: Marvin M3-S-CCS-615 dual-arm under src/nimo_arm_description/meshes
bash deploy/sync.sh      # → 远程 nimo_robot
bash deploy/build.sh     # 容器内编 nimo_robot
bash deploy/build_local.sh
```

## 日常启动

前置：两边 `ROS_DOMAIN_ID=0`；远程 `teleop` 已 up。本机不要 SSH 代启远程。  
Pico 与 Rokea 拖拽 **二选一**（GUI 上 Pico / ServoJ 遥操作互斥）。

### 远程 — 臂后端（必起）

```bash
cd ~/V1_Hardware_Control
bash deploy/enter.sh
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.bash && source /home/mundane/nimo_robot/install/setup.bash
ros2 launch nimo_arm_bringup backend.launch.py use_sim:=false
```

停后端并清残留（本机执行；僵尸需 `--restart`）：

```bash
cd /home/yxc/nimo_arm_gui
bash deploy/stop_backend.sh            # INT→TERM→KILL + 清 FastDDS SHM
bash deploy/stop_backend.sh --restart  # 再 docker restart teleop（彻底清 zombie）
```

### 远程 — Pico（与拖拽二选一）

```bash
cd ~/V1_Hardware_Control
bash deploy/enter.sh
export ROS_DOMAIN_ID=0

source /opt/ros/humble/setup.bash && source /home/mundane/vrcon_ws/install/setup.bash
ros2 launch nimo_bringup hmi_pico_arms.launch.py
```

### 本机 — GUI（必起）

```bash
cd /home/yxc/nimo_arm_gui
export ROS_DOMAIN_ID=0
source /opt/ros/jazzy/setup.bash && source install/setup.bash
bash deploy/run_gui.sh
```

### 本机 — Rokea 拖拽桥（与 Pico 二选一）

需本机到 Rokae 网段（默认 `192.168.2.160` / `.161`，本机口 `192.168.2.100`）且同 `ROS_DOMAIN_ID=0`。

```bash
source /opt/ros/jazzy/setup.bash
source /home/yxc/rokea_control/ws/install/setup.bash
export ROS_DOMAIN_ID=0
ros2 launch rokea_drag_bridge drag_bridge.launch.py
# 改 IP 例：
# ros2 launch rokea_drag_bridge drag_bridge.launch.py \
#   local_ip:=192.168.2.100 ip_a:=192.168.2.160 ip_b:=192.168.2.161 rate_hz:=50.0
```

### 自检（本机）

```bash
ros2 topic echo /nimo_arm/status --once
ros2 service list | grep set_teleop_motion
# Pico：
ros2 topic hz /arm/left/target_pose
# Rokea 拖拽：
ros2 topic hz /arm/left/target_joints
ros2 topic echo /rokea/drag_bridge/status --once
```

**Pico 路径 HMI：** Init → Enable → ControlMode → 勾选「Pico遥操作模式」→ **右摇杆键**对齐 yaw → 左内侧扳机跟随。

**拖拽路径 HMI：** Init → Enable → ControlMode → 起 `rokea_drag_bridge` → 勾选「ServoJ遥操作模式」→ 人推 Rokae；结束先取消勾选再 Ctrl+C bridge。  
（过容差会先 MovJ；Fault/EStop/Alarm / 话题超时 / ServoJ 失败会自动退出 ServoJ。）

正式链路 — Pico（`hmi_pico_arms` + `backend`）：

```text
SDK → FLUZ → 世界 yaw 对齐
  → 固定一极点 LP（真实设备 Δt；位置欧氏，姿态 SLERP/SO(3)；默认 fc=12 Hz）
  → One Euro（真实 Δt；姿态 Log+SLERP/SO(3)；默认 pos 2.5/β0.5，ori 0.7/β0.15）
  → 收即发 xr/pose/*
  → nimo_teleop（门控/缩放，不再 FLUZ/滤波/yaw）→ arm/*/target_pose
  → ServoPByPico（Δpos+ΔR；固定插值 H=0.03 s；500 Hz 环上一阶低通进 IK）
```

正式链路 — Rokea 拖拽（`rokea_drag_bridge` + `backend`）：

```text
Rokae enableDrag(jointSpace) → jointPos
  → 轴 2/4/6 取反（Rokea→Marvin）→ 50 Hz arm/*/target_joints
  → dual_arm_node（servoj_teleop_enabled）→ ServoJ(H=0.02 s)
```

`xr/pose_raw/*` = 仅 FLUZ（对齐与滤波之前），供录包分析。详细拖拽说明见 [`rokea_control/docs/rokea_drag_bridge.md`](../rokea_control/docs/rokea_drag_bridge.md)。

### 远程 RT（SCHED_FIFO 失败时，宿主）

```bash
echo 950000 | sudo tee /sys/fs/cgroup/cpu/docker/cpu.rt_runtime_us
cd ~/V1_Hardware_Control
TELEOP_RECREATE=1 bash deploy/up.sh
CID=$(docker inspect -f '{{.Id}}' teleop)
echo 950000 | sudo tee /sys/fs/cgroup/cpu/docker/$CID/cpu.rt_runtime_us
bash deploy/ensure_rt.sh
```

## ROS 接口（`/nimo_arm`）

- `/status`、`/joint_states`；`teleop_motion_enabled`（Pico）/ `servoj_teleop_enabled` / `traj_*`
- `init` / `shutdown` / `go_home` / `go_work`；每臂 `enable` / `disable` / …
- `/set_teleop_motion`（`mode`：Pico / ServoJ，互斥）；Pico 订 `arm/*/target_pose`；ServoJ 订 `arm/*/target_joints`（H=0.02 s；过容差先 MovJ；Fault/EStop/Alarm/话题超时/ServoJ 失败自动退出）
- `/left|right/set_joint_imp_kd`、`/left|right/set_cart_imp_kd`（默认见 `arm.yaml` impedance）
- 轨迹：`/traj/record|postprocess|replay/*`（`replay/start.auto_record` 自动写状态 CSV：q/dq/TCP/tau）

## 安全

- 退出：Home → Disable → Shutdown；**禁止**对臂进程 `kill -9`
- HMI **不**暴露 `HwInterface`

# 26GO2 — 睿抗机器人四足多模态巡检

基于宇树 Unitree Go2 四足机器人的自主任务控制代码，功能涵盖视觉巡检、雷达避障与机械臂视觉抓取。

## 项目结构

```
.
├── total10.py              # 主控程序：有限状态机(FSM)全流程控制器
├── line_camera3.py         # 功能测试：RealSense D435i 黑线循迹
├── pm_dataread.py          # 功能测试：跑道黑线断裂检测
├── imu_readonly2.py        # 功能测试：IMU/SportModeState 姿态读取
├── Template_*.png          # 模板匹配素材（电/氧化剂/辐射标志）
└── reference.md            # Unitree Go2 运动控制 API 参考
```

## 硬件依赖

| 硬件 | 用途 |
|------|------|
| Unitree Go2 | 四足机器人本体 |
| Intel RealSense D435i | 前置深度相机（巡线/红点检测） |
| 机器人内置 LiDAR | 前向测距（避障） |
| 机器人内置 IMU | 航向角获取（定向导航） |

## 软件依赖

- **Python 3.x** + OpenCV (`cv2`), NumPy
- **pyrealsense2** — RealSense 相机驱动
- **unitree_sdk2py** — 宇树 Go2 Python SDK

```bash
pip install opencv-python numpy pyrealsense2
# unitree_sdk2py 需从宇树官方获取
```

## 主程序状态机 (`total10.py`)

```
FOLLOW_TO_OBSTACLE → OBSTACLE_AVOID → FOLLOW_TO_STAIR → DO_STAIRS
                                                                    ↓
      RETURN ← RED_DOT_ACTION ← FOLLOW_TO_RED_DOT ←────────────────┘
```

| 状态 | 说明 |
|------|------|
| `FOLLOW_TO_OBSTACLE` | 沿黑线巡线，检测泡沫条断线触发跳跃 |
| `OBSTACLE_AVOID` | 纯 LiDAR 避障，重新看到线后退出 |
| `FOLLOW_TO_STAIR` | 爬楼后巡线，检测 ArUco 16 标记后上楼梯 |
| `DO_STAIRS` | FreeWalk 步态上下楼梯序列 |
| `FOLLOW_TO_RED_DOT` | 三阶段：IMU 定向到北→西向→红点视觉识别 |
| `RED_DOT_ACTION` | ORB+BF 模板匹配识别物块，执行对应动作 |
| `RETURN` | 检测泡沫条→前跳→IMU 回航到起始姿态 |

### 感知算法

- **黑线检测**：自适应阈值 + 形态学开闭运算 + 轮廓面积筛选
- **断线检测**：逐行黑像素比例统计，连续低比例行数 ≥22 触发
- **红点检测**：HSV 颜色空间掩码 + 圆度 (`4πA/P²>0.55`) + 宽高比校验
- **模板匹配**：鱼眼矫正 → 黄色区域提取 → ORB 特征 + BF 暴力匹配
- **避障**：LiDAR 三向测距 (前/左/右) + 居中 PID 控制

### 运行方式

```bash
# 主程序（需在机器人上运行）
nohup python -u total10.py eth0 > robot_run.log 2>&1 &

# 各功能模块可独立测试
python line_camera3.py    # 巡线测试（需连接相机）
python pm_dataread.py     # 断线检测可视化
python imu_readonly2.py   # IMU 数据读取
```

## 功能模块说明

### `line_camera3.py` — 黑线循迹

基于 RealSense D435i 640×480 彩色图像，截取底部 30% ROI，阈值二值化提取黑线轮廓，计算中心误差后 PID 控制偏航角速度 `vyaw=-k*error`，发送 `sport.Move(vx, 0, vyaw)` 实现巡线。

### `pm_dataread.py` — 断线检测

截取图像 55%~90% 区域，逐行统计黑像素比例，当某行比例 <8% 时记为"断"，连续断行数 `max_break` 实时输出并可视化。

### `imu_readonly2.py` — IMU 姿态读取

订阅 DDS Topic `rt/sportmodestate`，解析 `SportModeState_` 消息中的 position、velocity、imu_state.rpy 并打印，用于 IMU 数据调试。

## API 参考

详见 [reference.md](./reference.md)，包含 Go2 全部 39 个运动控制接口的原型、参数及错误码表。

---

## 机械臂模块 (`机械臂/`)

基于 Lobot 总线舵机的 4 自由度机械臂控制系统，使用串口通信驱动舵机，配合 Intel RealSense 深度相机 + ArUco 标记实现视觉伺服闭环抓取。

**运行平台：** Go2 机器人算力板 (Ubuntu, Python 3.10)
**工作目录：** `/home/unitree/workspace/xwh/arm/`

### 文件说明

| 文件 | 说明 |
|------|------|
| `servo.py` | Lobot 舵机串口驱动核心库，实现通信协议（帧构造、反码校验、收发） |
| `main_4dof_test.py` | 舵机 ID 扫描工具，扫描指定 ID 是否在线并读取当前位置，运动测试代码已注释 |
| `single_place_test.py` | 单舵机位置读写测试，验证基本运动控制（小角度移动+回位） |

### 硬件依赖

| 硬件 | 用途 |
|------|------|
| Lobot 总线舵机 ×4 | 关节执行器（ID 1 底座旋转 / 2 肩 / 4 肘 / 5 腕） |
| Lobot 舵机控制板 | USB 转串口 → `/dev/ttyUSB0` |
| Intel RealSense D435/D455 | 深度相机，ArUco 标记位姿检测（序列号 `115222070999`） |

> **注意：** 舵机 ID 3 已损坏，当前系统使用 4 自由度（ID 1, 2, 4, 5）。

### 通信协议

舵机使用 Lobot 串行总线协议，波特率 **115200**：

```
数据帧格式: 0x55 0x55 [ID] [长度] [命令] [参数...] [校验和]
              ↑ 双字节帧头              ↑ 反码求和，取低 8 位
```

- 舵机 ID 范围：0–253，**254** 为广播 ID（控制所有舵机）
- 位置范围：**0–1000**，对应物理角度约 0°–240°
- 核心指令：`0x01`（按时移动）、`0x1C`（读位置）、`0x1F`（加载/卸载）

### 舵机 ID 与关节映射

| 舵机 ID | 关节 | 状态 |
|---------|------|:--:|
| 1 | 底座旋转 | ✅ |
| 2 | 肩关节 | ✅ |
| 3 | — | ❌ 损坏 |
| 4 | 肘关节 | ✅ |
| 5 | 腕关节 / 末端 | ✅ |

### 视觉伺服控制流程

```
RealSense 相机
      │
      ▼
ArUco 检测 + PnP 位姿估算  →  标记在相机坐标系下的 (X, Y, Z)
      │
      ▼
手眼标定变换  →  标记在机械臂基座坐标系下的 (X, Y, Z)
      │
      ▼
逆运动学 (IK) 求解  →  各关节目标角度 [θ1, θ2, θ4, θ5]
      │
      ▼
角度 → 舵机位置映射 (0–1000)  →  串口发送 Lobot 协议指令  →  舵机转动
```

> **IK 和手眼标定目前为占位状态，待实现。** 去年方案（`25_Arm.py`）使用 ArUco + PID 直接像素映射，无需 IK，可作为参考。

### 软件依赖

```bash
# 注意：Go2 上必须用 python -m pip install（裸 pip 指向系统 Python 3.8）
python -m pip install pyserial pyrealsense2 opencv-python numpy
# 可选：逆运动学数值求解
python -m pip install ikpy
```

### 使用方式

```bash
# 1. 串口授权（每次重启后执行一次）
sudo chmod 666 /dev/ttyUSB0

# 2. 扫描在线舵机 ID
python 机械臂/main_4dof_test.py

# 3. 单舵机运动测试
python 机械臂/single_place_test.py
```

### 当前进展（2026-06-02）

- [x] Go2 环境配置（Python 3.10, pyserial 3.5）
- [x] 串口 `/dev/ttyUSB0` 通信正常，舵机可读取位置
- [x] RealSense 相机已确认（序列号 `115222070999`）
- [ ] 完成所有 4 个舵机 ID 扫描及初始位置记录
- [ ] 运行 `realsense_camera_test.py` 验证 ArUco 检测
- [ ] 测量机械臂连杆长度 L1–L4
- [ ] 实现逆运动学（几何法或 ikpy）
- [ ] 手眼标定（eye-to-hand 方案）
- [ ] 整体视觉伺服联调

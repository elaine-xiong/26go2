# 26GO2 — Unitree Go2 机器狗竞赛代码

基于宇树 Unitree Go2 四足机器人的自主任务控制代码，用于第26届高教杯机器人竞赛。

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

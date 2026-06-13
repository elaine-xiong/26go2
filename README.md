# Go2 机械臂联动集成代码

## 文件说明

| 文件 | 作用 |
|------|------|
| `total10.py` | 狗主控，状态机 + 巡线 + 避障 + 三个位置点调机械臂 |
| `arm_one_shot.py` | 机械臂一次性脚本，支持 grab / place / place_grab 三种任务 |
| `servo.py` | Lobot 舵机串口驱动（来自 arm_code） |
| `Template_*.png` | 模板匹配图片（电/氧化剂/辐射） |

## 部署到 Go2
# 1. 串口权限
sudo chmod 666 /dev/ttyUSB0

# 2. 确认两个 RealSense 都在
python3 -c "import pyrealsense2 as rs; [print(d.get_info(rs.camera_info.serial_number)) for d in rs.context().devices]"
# 应该看到: 135122071432（狗）和 115222070999（臂）
或者
lsusb

# 3. 启动
nohup python3 -u total.py > robot_run.log 2>&1 &

# 4. 看日志
tail -f robot_run.log
```

## 状态机流程

```
STATE 1 (FOLLOW_TO_OBSTACLE)    巡线 + 泡沫条检测
STATE 2 (OBSTACLE_AVOID)        雷达避障
STATE 3 (FOLLOW_TO_STAIR)       巡线到楼梯
STATE 4 (DO_STAIRS)             爬楼 + 下楼
STATE 5 (FOLLOW_TO_RED_DOT)
    phase 0: 巡线 → IMU朝北(0°) → 停 → 模板识别 → [机械臂①: grab left]
    phase 1: 巡线 → 第二次朝西(100°) → 停 → 右移 → [机械臂②: place_grab right] → 左移
    phase 2: 巡线 → 识别红点
STATE 6 (RED_DOT_ACTION)        识牌做动作
STATE 7 (RETURN)
    sub_phase 0: 计时巡线 → 停 → [机械臂③: place left]
    sub_phase 1: 泡沫条跳跃 → 方位回航
```

## 三个机械臂位置点

| 位置 | 触发条件 | 任务 | 方向 | arm_one_shot 参数 |
|------|----------|------|------|-------------------|
| ① 朝北 | IMU rel_yaw≈0° | 只夹取 | 左 | `--task grab --direction left` |
| ② 二次朝西 | IMU 第2次进入西向 | 先放再夹 | 右 | `--task place_grab --direction right` |
| ③ 红点后 | 巡线N秒后 | 只放 | 左(现场定) | `--task place --direction left` |

## 现场可调参数

### total10.py（顶部常量）

```python
DOG_CAMERA_SERIAL = "135122071432"    # 狗用 RealSense
ARM_CAMERA_SERIAL = "115222070999"    # 臂用 RealSense
ARM_PORT = "/dev/ttyUSB0"             # 舵机串口
ARM_TIMEOUT = 40.0                    # 臂超时(秒)

POINT2_RIGHT_VY = 0.25               # 位置②右移速度
POINT2_RIGHT_DURATION = 1.5          # 位置②右移时长(秒)
RET_SUB_PHASE0_LINE_TIME = 4.0       # 位置③巡线时长(秒)
```

### arm_one_shot.py（顶部常量）

```python
# 初始姿态（左右各一套）
POSE_INIT = {
    "left":  {BASE: 165, SHOULDER: 234, ELBOW: 136, GRIPPER: 470},
    "right": {BASE: 890, SHOULDER: 163, ELBOW: 135, GRIPPER: 470},
}

# 放物姿态（胡编的，现场必调！）
POSE_PLACE = {
    "left":  {BASE: 165, SHOULDER: 380, ELBOW: 260, GRIPPER: 700},
    "right": {BASE: 890, SHOULDER: 340, ELBOW: 240, GRIPPER: 700},
}

GRIPPER_CLOSE_DEFAULT = 800           # 夹爪闭合值，三棱锥可能要调

```

## 保险措施

- 臂脚本内部超时 → 归位 → exit(1)
- 狗 `subprocess.wait(timeout+5)` → 超时 `kill()` → 继续流程
- 无论臂成功/失败/超时，狗都继续，不死等
- 臂启动时枚举相机 → 序列号不在 → 立即 exit(1)，不卡

## 单独测试机械臂

```bash
# 只夹取（左）
python3 single_arm.py --serial 115222070999 --task grab --direction left

# 只放（右）
python3 single_arm.py --serial 115222070999 --task place --direction right

# 先放再夹（右）
python3 single_arm.py --serial 115222070999 --task place_grab --direction right
```

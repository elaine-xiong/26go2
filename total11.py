# -*- coding: gbk -*-
# 集成版：机械臂联动
# nohup python -u total.py > robot_run.log 2>&1 &
import cv2
import cv2.aruco as aruco
import numpy as np
import time
import math
import os
import sys
import threading
import subprocess
import pyrealsense2 as rs

from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
from unitree_sdk2py.idl.geometry_msgs.msg.dds_ import PointStamped_
from unitree_sdk2py.idl.unitree_go.msg.dds_ import SportModeState_
from unitree_sdk2py.go2.sport.sport_client import SportClient
from unitree_sdk2py.go2.video.video_client import VideoClient
from unitree_sdk2py.go2.vui.vui_client import VuiClient

# ==========================================
# 状态机常量枚举
# ==========================================
STATE_FOLLOW_TO_OBSTACLE = 1
STATE_OBSTACLE_AVOID     = 2
STATE_FOLLOW_TO_STAIR    = 3
STATE_DO_STAIRS          = 4
STATE_FOLLOW_TO_RED_DOT  = 5
STATE_RED_DOT_ACTION     = 6
STATE_RETURN             = 7

# ==========================================
# 机械臂配置
# ==========================================
# 狗用相机序列号
DOG_CAMERA_SERIAL = "135122071432"

# 机械臂脚本路径（与本文件同目录）
_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
ARM_SCRIPT_PATH = os.path.join(_THIS_DIR, "arm.py")
ARM_CAMERA_SERIAL = "115222070999"
ARM_PORT = "/dev/ttyUSB0"
ARM_TIMEOUT = 60.0           # 臂脚本整体超时

# 位置一：识别动作参数
POINT1_FAKE_RIGHT_VY = -0.3       # 右移速度
POINT1_FAKE_RIGHT_DURATION = 1.5  # 右移时长（秒）
POINT1_FAKE_LEFT_YAWRATE = 1.0    # 左转速度
POINT1_FAKE_LEFT90_DURATION = 2.0 # 左转90°时长（秒）
POINT1_FAKE_RIGHT_YAWRATE = -1.0  # 右转速度
POINT1_FAKE_RIGHT45_DURATION = 1.5 # 右转45°时长（秒）

# 位置一：机械臂前平移调整参数（类似位置二）
POINT1_ADJ_LEFT_VY = 0.3          # 左移速度（正值=左）
POINT1_ADJ_LEFT_DURATION = 1.0    # 左移时长（秒）
POINT1_ADJ_RIGHT_VY = -0.3        # 右移速度（归位，负值=右）
POINT1_ADJ_RIGHT_DURATION = 2.0   # 右移时长（秒）
POINT1_RECOGNIZE_TIMEOUT = 1.0    # 识别超时（秒）

# 位置二：平移参数
POINT2_FORWARD_VX = 0.0       # 前进速度
POINT2_FORWARD_DURATION = 1.0  # 前进时长（秒）
POINT2_RIGHT_VY = -0.3         # 右移速度
POINT2_RIGHT_DURATION = 1.0    # 右移时长（秒）
POINT2_SUPPRESS_TIME = 11.0     # 位置一机械臂完成后，抑制朝西触发的时长（秒）

# 位置三：STATE_RETURN 子阶段0 巡线时长
RET_SUB_PHASE0_LINE_TIME = 7.0

# 夹爪力度（三棱锥=800，其余=700，抽签后只改对应的）
POSITION1_GRIP_CLOSE = 700   # 位置① 夹取力度
POSITION2_GRIP_CLOSE = 700   # 位置② 夹取力度

# ==========================================
# 视觉/感知算法模块
# ==========================================
def detect_track_line(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 50, 255, cv2.THRESH_BINARY_INV)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_h, roi_w = roi.shape[:2]
    roi_area = roi_h * roi_w
    best, best_area = None, -1
    MIN_AREA_RATIO = 0.1
    for c in contours:
        area = cv2.contourArea(c)
        if area / roi_area < MIN_AREA_RATIO:
            continue
        if area > best_area:
            best_area = area
            x, y, w, h = cv2.boundingRect(c)
            best = (x, y, w, h, area / roi_area, area / max(w * h, 1), w / float(h) if h > 0 else 0)
    return best, binary

def detect_line_break(frame):
    h, w = frame.shape[:2]
    roi = frame[int(h * 0.55):int(h * 0.90), :]
    roi_h, roi_w = roi.shape[:2]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    _, binary = cv2.threshold(cv2.GaussianBlur(gray, (5, 5), 0), 70, 255, cv2.THRESH_BINARY_INV)
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    break_count, max_break, detected = 0, 0, False
    for y in range(roi_h):
        if (cv2.countNonZero(binary[y, :]) / float(roi_w)) < 0.08:
            break_count += 1
        else:
            break_count = 0
        max_break = max(max_break, break_count)
        if break_count >= 22:
            detected = True
            break
    return detected, roi, binary, None, max_break

def detect_red_circle(frame):
    h, w = frame.shape[:2]
    roi = frame[int(h * 0.7):h, :]
    hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
    mask = cv2.morphologyEx(
        cv2.inRange(hsv, (0, 80, 60), (10, 255, 255)) |
        cv2.inRange(hsv, (160, 80, 60), (180, 255, 255)),
        cv2.MORPH_OPEN,
        np.ones((5, 5), np.uint8)
    )
    cnts, _ = cv2.findContours(
        cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8)),
        cv2.RETR_EXTERNAL,
        cv2.CHAIN_APPROX_SIMPLE
    )
    for c in cnts:
        area = cv2.contourArea(c)
        if area < 500:
            continue
        peri = cv2.arcLength(c, True)
        if 4 * np.pi * area / (peri * peri) > 0.55:
            x, y, bw, bh = cv2.boundingRect(c)
            if 0.7 <= bw / float(bh) <= 1.3:
                return True, roi, mask
    return False, roi, mask

# ==========================================
# 核心主类：机器人状态机控制器
# ==========================================
class RobotFSM:
    def __init__(self, iface="eth0"):
        self.iface = iface
        self.state = STATE_FOLLOW_TO_OBSTACLE
        self.running = True

        # 频率限制与启动
        self.last_move_time = 0.0
        self.MOVE_INTERVAL = 0.08
        self.startup_time = 0.0
        self.warmup_seconds = 2.0

        # 巡线运动参数
        self.vx, self.k = 0.3, 0.0035
        self.smooth_error = 0.0
        self.lost_line_count = 0
        self.last_vyaw = 0.0
        self.post_vx, self.post_k = 0.3, 0.008
        self.post_smooth_error, self.post_lost_count = 0.0, 0

        # 避障相关
        self.front_dist = self.left_dist = self.right_dist = 3.0
        self.TURN_COOLDOWN = 1.2
        self.last_turn_finish_time = 0.0
        self.STOP_FRONT, self.SAFE_SIDE, self.CENTER_THRESHOLD = 0.55, 0.2, 0.05
        self.FORWARD_SPEED, self.TURN_GAIN, self.MAX_YAW, self.ALPHA = 0.3, 1.5, 1.0, 0.35

        # 泡沫条状态
        self.foam_triggered_state1 = False
        self.foam_triggered_state7 = False
        self.foam_action_lock_until = 0.0
        self.foam_action_busy = False

        # 避障退出延迟
        self.line_seen_time = None

        # IMU / 航向相关（使用 rt/sportmodestate）
        self.sport_state = None
        self.sport_state_lock = threading.Lock()
        self.imu_ready = False
        self.yaw_deg = 0.0
        self.yaw_ref_deg = None
        self.HEADING_TOL = 15.0
        self.IMU_PRINT_INTERVAL = 0.5
        self.last_imu_print_time = 0.0

        # 回航参数（do_return_tick 内部使用）
        self.return_phase = 0
        self.return_phase_start = 0.0
        self.return_target_rel_yaw = None

        self.RETURN_FORWARD_VX = 0.3
        self.RETURN_FORWARD_TIME = 2.5
        self.RETURN_HEADING_TOL = 8.0
        self.RETURN_TURN_GAIN = 0.025
        self.RETURN_TURN_MAX = 0.80
        self.RETURN_TURN_MIN = 0.40

        # STATE_RETURN 子阶段（新增）
        # ret_sub_phase = 0: 计时巡线 → 停住 → 机械臂
        # ret_sub_phase = 1: 泡沫条检测 + 方位回航（原逻辑）
        self.ret_sub_phase = 0
        self.ret_sub_phase_start = 0.0

        # state 5 内部子阶段
        self.red_dot_phase = 0
        self.point1_done = False
        self.point2_done = False
        self.point1_object_label = None
        self.point2_prev_west = False
        self.point2_suppress_until = None   # 位置二触发抑制时间锁
        self.point2_wait_fresh_west = False # 解锁后，等待一次“新的进入西向”边沿

        # --- 鱼眼识别配置 ---
        self.K = np.array([[265.0, 0, 320.0], [0, 265.0, 240.0], [0, 0, 1]], dtype=np.float32)
        self.D = np.array([-0.02, -0.01, 0.0, 0.0], dtype=np.float32)
        self.NEW_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            self.K, self.D, (640, 480), np.eye(3), balance=0.0
        )
        self.ORB = cv2.ORB_create(nfeatures=500)
        self.BF = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
        self.LOWER_YELLOW = np.array([15, 80, 80])
        self.UPPER_YELLOW = np.array([35, 255, 255])
        self.MATCH_THRESHOLD = 15
        self.templates = {}
        self.preload_templates()

        # ArUco
        self.TARGET_ID = 16
        self.aruco_detector = aruco.ArucoDetector(
            aruco.getPredefinedDictionary(aruco.DICT_4X4_50),
            aruco.DetectorParameters()
        )

    def preload_templates(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        files = {0: "Template_electric.png", 1: "Template_oxidizer.png", 2: "Template_radiation.png"}
        for label, fname in files.items():
            img = cv2.imread(os.path.join(current_dir, fname), cv2.IMREAD_UNCHANGED)
            if img is None:
                continue
            if img.shape[-1] == 4:
                alpha = img[:, :, 3][:, :, np.newaxis].astype(np.float32) / 255.0
                img = (img[:, :, :3] * alpha + np.ones_like(img[:, :, :3]) * 255 * (1 - alpha)).astype(np.uint8)
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
            _, th = cv2.threshold(gray, 240, 255, cv2.THRESH_BINARY_INV)
            cnts, _ = cv2.findContours(th, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cnts:
                x, y, w_c, h_c = cv2.boundingRect(max(cnts, key=cv2.contourArea))
                gray = gray[y:y+h_c, x:x+w_c]
            gray = cv2.resize(gray, (200, 173))
            kp, des = self.ORB.detectAndCompute(gray, None)
            self.templates[label] = (kp, des)

    def _to_list(self, x):
        try:
            return [float(i) for i in x]
        except:
            return None

    def _wrap_deg(self, ang):
        return (ang + 180.0) % 360.0 - 180.0

    def sportstate_callback(self, msg):
        with self.sport_state_lock:
            self.sport_state = msg

    def _pull_imu_update(self):
        with self.sport_state_lock:
            ss = self.sport_state

        if ss is None:
            return False

        imu = getattr(ss, "imu_state", None)
        if imu is None:
            return False

        quat = self._to_list(getattr(imu, "quaternion", None))
        gyro = self._to_list(getattr(imu, "gyroscope", None))
        acc = self._to_list(getattr(imu, "accelerometer", None))
        rpy = self._to_list(getattr(imu, "rpy", None)) or self._to_list(getattr(imu, "euler", None))

        if rpy is None or len(rpy) < 3:
            return False

        yaw_deg = math.degrees(float(rpy[2]))
        self.yaw_deg = yaw_deg

        if self.yaw_ref_deg is None:
            self.yaw_ref_deg = yaw_deg
            self.imu_ready = True
            print(f"[IMU] North reference locked at {self.yaw_ref_deg:.2f} deg")

        self.imu_debug_print(quat, gyro, acc, rpy)
        return True

    def _rel_yaw(self):
        if self.yaw_ref_deg is None:
            return None
        return self._wrap_deg(self.yaw_deg - self.yaw_ref_deg)

    def _yaw_near(self, target_deg, tol=None):
        if tol is None:
            tol = self.HEADING_TOL
        rel = self._rel_yaw()
        if rel is None:
            return False
        return abs(self._wrap_deg(rel - target_deg)) <= tol

    def imu_debug_print(self, quat, gyro, acc, rpy):
        now = time.time()
        if self.state != STATE_FOLLOW_TO_RED_DOT:
            return
        if now - self.last_imu_print_time < self.IMU_PRINT_INTERVAL:
            return

        self.last_imu_print_time = now
        rel_yaw = self._rel_yaw()

        print(
            f"[STATE={self.state} phase={self.red_dot_phase}] "
            f"quat={quat} gyro={gyro} acc={acc} rpy={rpy} "
            f"yaw={self.yaw_deg:.2f}deg rel={rel_yaw:.2f}deg"
        )

    def pause_5s(self, msg=""):
        if msg:
            print(msg)
        self.safe_move(0, 0, 0, force=True)
        time.sleep(3.0)

    # ==========================================
    # 机械臂子进程调用（新增）
    # ==========================================
    def _call_arm(self, position_label, timeout=None, task="grab",
                  direction="left", grip_close=700):
        """
        启动 arm.py 子进程，等待完成或超时。
        返回 True=成功, False=失败/超时/异常
        无论成败，狗继续流程，不死等。
        """
        if timeout is None:
            timeout = ARM_TIMEOUT

        cmd = [
            sys.executable,
            ARM_SCRIPT_PATH,
            "--serial", ARM_CAMERA_SERIAL,
            "--port", ARM_PORT,
            "--timeout", str(int(timeout)),
            "--task", task,
            "--direction", direction,
            "--grip-close", str(grip_close),
            "--label", position_label,
        ]
        print(f"[ARM] 启动: {' '.join(cmd)}")

        try:
            proc = subprocess.Popen(cmd)
            # 等待时间 = 臂内部超时 + 5 秒余量
            proc.wait(timeout=timeout + 5)
            if proc.returncode == 0:
                print(f"[ARM] {position_label} — 成功")
                return True
            else:
                print(f"[ARM] {position_label} — 失败 (exitcode={proc.returncode})")
                return False
        except subprocess.TimeoutExpired:
            print(f"[ARM] {position_label} — 超时 ({timeout+5}s)，强制终止")
            proc.kill()
            try:
                proc.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
            return False
        except FileNotFoundError:
            print(f"[ARM] {position_label} — 错误：找不到脚本 {ARM_SCRIPT_PATH}")
            return False
        except Exception as e:
            print(f"[ARM] {position_label} — 异常: {e}")
            return False

    def _recognize_template_label(self, timeout=10.0):
        """
        复用模板识别逻辑，返回 best_label，识别不到返回 None
        """
        t0 = time.time()
        while time.time() - t0 < timeout:
            code, data = self.video_client.GetImageSample()
            if code != 0:
                time.sleep(0.05)
                continue

            if data is None or len(data) == 0:
                print("Empty image data")
                time.sleep(0.05)
                continue

            raw = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
            if raw is None:
                print("Decode failed")
                time.sleep(0.05)
                continue

            try:
                raw = cv2.resize(raw, (640, 480))
            except:
                time.sleep(0.05)
                continue

            head = cv2.fisheye.undistortImage(raw, self.K, self.D, Knew=self.NEW_K)
            hsv = cv2.cvtColor(head, cv2.COLOR_BGR2HSV)
            mask = cv2.morphologyEx(
                cv2.inRange(hsv, self.LOWER_YELLOW, self.UPPER_YELLOW),
                cv2.MORPH_OPEN,
                np.ones((5, 5), np.uint8)
            )

            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_label, max_good = -1, 0

            for c in cnts:
                if cv2.contourArea(c) < 1500:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                crop = head[y:y+h, x:x+w]
                if crop.size == 0:
                    continue

                roi = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                kp_r, des_r = self.ORB.detectAndCompute(roi, None)
                if des_r is None:
                    continue

                for lbl, (kp_t, des_t) in self.templates.items():
                    good = len([m for m in self.BF.match(des_t, des_r) if m.distance < 50])
                    if good > max_good:
                        max_good = good
                        best_label = lbl

            if max_good > self.MATCH_THRESHOLD:
                return best_label

            time.sleep(0.1)

        return None

    def execute_point1_vision_task(self):
        """
        第一个IMU点触发后的图像识别 + 机械臂操作。
        """
        # ===== 快速识别（缩短无用时间）=====
        print(">>> 第一个点：快速图像识别 <<<")
        label = self._recognize_template_label(timeout=POINT1_RECOGNIZE_TIMEOUT)
        self.point1_object_label = label

        if label is None:
            print("[Point1] 没识别到有效目标")
        else:
            print(f"[Point1] 识别结果 label = {label}")

        # ===== 平移调整（类似位置二）=====
        # 机械臂前先向左稍微平移
        print(f">>> 第一个点：向左平移 {POINT1_ADJ_LEFT_DURATION}s <<<")
        self.move_continuous(0, POINT1_ADJ_LEFT_VY, 0, POINT1_ADJ_LEFT_DURATION)
        self.safe_move(0, 0, 0, force=True)
        time.sleep(0.3)

        # ===== 调用机械臂抓取 =====
        print(">>> 第一个点：启动机械臂抓取 <<<")
        success = self._call_arm("point1", task="grab", direction="left",
                                grip_close=POSITION1_GRIP_CLOSE)
        if success:
            print("[Point1] 机械臂抓取成功")
        else:
            print("[Point1] 机械臂抓取失败/超时，继续流程")

        # 机械臂完成后向右平移归位
        print(f">>> 第一个点：向右平移归位 {POINT1_ADJ_RIGHT_DURATION}s <<<")
        self.move_continuous(0, POINT1_ADJ_RIGHT_VY, 0, POINT1_ADJ_RIGHT_DURATION)
        self.safe_move(0, 0, 0, force=True)
        time.sleep(0.3)

        # ===== 伪识别动作 =====
        # 向右平移
        self.move_continuous(0, POINT1_FAKE_RIGHT_VY, 0, POINT1_FAKE_RIGHT_DURATION)
        self.safe_move(0, 0, 0, force=True)
        time.sleep(0.3)
        # 向左旋转90°
        self.move_continuous(0, 0, POINT1_FAKE_LEFT_YAWRATE, POINT1_FAKE_LEFT90_DURATION)
        self.safe_move(0, 0, 0, force=True)
        time.sleep(0.3)
        # 定3秒
        self.safe_move(0, 0, 0, force=True)
        time.sleep(3.0)
        # 向右转45°
        self.move_continuous(0, 0, POINT1_FAKE_RIGHT_YAWRATE, POINT1_FAKE_RIGHT45_DURATION)
        self.safe_move(0, 0, 0, force=True)
        print(">>> 第一个点：识别动作完成 <<<")

        # 位置二触发时间锁：伪识别全部结束后才开始计时
        self.point2_suppress_until = time.time() + POINT2_SUPPRESS_TIME
        self.point2_wait_fresh_west = False
        self.point2_prev_west = self._yaw_near(100.0)  # 记录抑制开始时的当前状态
        print(f"[Point2] 抑制朝西触发直到 {self.point2_suppress_until:.1f}")
    def init_hardware(self):
        ChannelFactoryInitialize(0, self.iface)

        self.sport = SportClient()
        self.sport.Init()

        self.video_client = VideoClient()
        self.video_client.Init()

        self.vui_client = VuiClient()
        self.vui_client.Init()

        self.lidar_sub = ChannelSubscriber("rt/utlidar/range_info", PointStamped_)
        self.lidar_sub.Init(self.lidar_callback, 10)

        self.sportstate_sub = ChannelSubscriber("rt/sportmodestate", SportModeState_)
        self.sportstate_sub.Init(self.sportstate_callback, 10)

        time.sleep(2)
        self.sport.RecoveryStand()
        time.sleep(2)
        self.sport.SpeedLevel(1)

        # 使用狗专用相机序列号，避免和机械臂抢设备
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_device(DOG_CAMERA_SERIAL)
        config.enable_stream(rs.stream.color, 424, 240, rs.format.bgr8, 30)
        self.pipeline.start(config)

        self.startup_time = time.time()

    def lidar_callback(self, msg):
        x, y, z = msg.point.x, msg.point.y, msg.point.z
        if not math.isinf(x):
            self.front_dist = x
        if not math.isinf(y):
            self.left_dist = y
        if not math.isinf(z):
            self.right_dist = z

    def safe_move(self, vx, vy, vyaw, force=False):
        now = time.time()
        if force or (vx == 0 and vy == 0 and vyaw == 0) or (now - self.last_move_time >= self.MOVE_INTERVAL):
            self.sport.Move(vx, vy, vyaw)
            self.last_move_time = now

    def set_state(self, new_state):
        print(f"Transition: {self.state} -> {new_state}")
        self.state = new_state
        self.smooth_error = self.post_smooth_error = 0.0
        self.safe_move(0, 0, 0, force=True)
        time.sleep(0.2)
        if new_state == STATE_FOLLOW_TO_RED_DOT:
            self.red_dot_phase = 0
            self.point1_done = False
            self.point2_done = False
            self.point1_object_label = None
            self.point2_prev_west = False
            self.point2_suppress_until = None
            self.point2_wait_fresh_west = False
        # ===== 新增：STATE_RETURN 子阶段重置 =====
        if new_state == STATE_RETURN:
            self.ret_sub_phase = 0
            self.ret_sub_phase_start = 0.0

    def move_continuous(self, vx, vy, vyaw, duration):
        t0 = time.time()
        while time.time() - t0 < duration:
            self.safe_move(vx, vy, vyaw)
            time.sleep(0.08)
        self.safe_move(0, 0, 0, force=True)

    def execute_foam_jump_action(self, use_post_stair=False):
        if self.foam_action_busy:
            return
        self.foam_action_busy = True
        try:
            print(">>> 执行泡沫条冲刺 + 跳跃 <<<")
            # self.move_continuous(0.3, 0, 0, 0.8)
            self.sport.FrontJump()
            time.sleep(3.0)
            self.safe_move(0, 0, 0, force=True)
        finally:
            self.foam_action_busy = False

    def do_obstacle_avoid_tick(self, frame, now):
        """
        纯雷达避障 + 重新看到线优先退出
        返回值：
            True  - 本次已执行避障动作
            False - 没有执行动作（一般表示已经切到下个状态，或者无需处理）
        """
        h = frame.shape[0]
        cooldown = (now - self.last_turn_finish_time) < self.TURN_COOLDOWN

        # 第一优先级：检测是否重新看到线（保持2秒避障再退出）
        roi = frame[int(h * 0.9):h, :]
        best, _ = detect_track_line(roi)

        if (not cooldown) and (best is not None):
            if self.line_seen_time is None:
                self.line_seen_time = now
                print("Line reacquired! Hold OA for 2s...")
            elif now - self.line_seen_time >= 2.0:
                print("Line reacquired! Exit obstacle avoid.")
                self.line_seen_time = None
                self.safe_move(0, 0, 0, force=True)
                time.sleep(0.2)
                self.set_state(STATE_FOLLOW_TO_STAIR)
                return False
        else:
            self.line_seen_time = None

        # 第二优先级：雷达避障
        front = self.front_dist
        left = self.left_dist
        right = self.right_dist

        if front < self.STOP_FRONT:
            print("Front blocked")

            if left > right and left > self.SAFE_SIDE:
                print("Turn LEFT")
                self.safe_move(0.2, 0.0, 1.0)
            elif right > left and right > self.SAFE_SIDE:
                print("Turn RIGHT")
                self.safe_move(0.2, 0.0, -1.2)
            else:
                print("Rotate to search exit")
                self.safe_move(0.0, 0.0, 0.3)

            time.sleep(0.08)
            return True

        if left < self.SAFE_SIDE:
            print("Too close left")
            self.safe_move(0.2, 0.0, -0.5)
            time.sleep(0.08)
            return True

        if right < self.SAFE_SIDE:
            print("Too close right")
            self.safe_move(0.2, 0.0, 0.5)
            time.sleep(0.08)
            return True

        error = left - right

        if abs(error) > self.CENTER_THRESHOLD:
            yaw = self.TURN_GAIN * error
            yaw = max(-self.MAX_YAW, min(self.MAX_YAW, yaw))
            self.safe_move(self.FORWARD_SPEED, 0.0, yaw)
        else:
            self.safe_move(self.FORWARD_SPEED, 0.0, 0.0)

        time.sleep(0.08)
        return True

    def do_line_follow_tick(self, frame):
        roi = frame[int(frame.shape[0] * 0.7):, :]
        best, _ = detect_track_line(roi)
        if best:
            self.lost_line_count = 0
            error = (best[0] + best[2] // 2) - (roi.shape[1] // 2)
            self.smooth_error = 0.8 * self.smooth_error + 0.2 * error
            vyaw = max(min(-self.k * self.smooth_error, 0.9), -0.9)
            self.last_vyaw = vyaw
            self.safe_move(self.vx, 0, vyaw)
            return True
        else:
            self.lost_line_count += 1
            if self.lost_line_count < 8:
                self.safe_move(self.vx, 0, self.last_vyaw)
            else:
                self.safe_move(0, 0, 0.3)
            return False

    def do_line_follow_post_stair(self, frame):
        roi = frame[int(frame.shape[0] * 0.8):, :]
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, binary = cv2.threshold(cv2.GaussianBlur(gray, (5, 5), 0), 80, 255, cv2.THRESH_BINARY_INV)
        cnts, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cnts and cv2.contourArea(max(cnts, key=cv2.contourArea)) > 400:
            x, y, w, h = cv2.boundingRect(max(cnts, key=cv2.contourArea))
            error = (x + w // 2) - (roi.shape[1] // 2)
            self.post_smooth_error = 0.8 * self.post_smooth_error + 0.2 * error
            self.safe_move(self.post_vx, 0, max(min(-self.post_k * self.post_smooth_error, 0.9), -0.9))
            return True
        self.safe_move(0, 0, 0.6)
        return False

    def _execute_stair_sequence(self):
        self.sport.FreeWalk()
        time.sleep(0.5)

        # 爬楼
        self.move_continuous(0.3, 0, 0, 3.0)
        self.move_continuous(0.35, 0, -0.1, 2.0)
        time.sleep(1.0)

        # 原地转向
        self.move_continuous(0.23, 0, 1.4, 1.5)
        self.move_continuous(0.0, 0, 0.0, 1.0)

        # 下楼
        self.move_continuous(0.35, 0, 0.2, 1.0)
        self.move_continuous(0.35, 0, -0.2, 3.0)

    def _execute_red_dot_sequence(self):
        print(">>> 识别模式开始 <<<")
        self.move_continuous(0.35, 0, 0, 2.0)
        self.move_continuous(0, 0, 1.2, 2.0)  # 找牌
        t0 = time.time()
        while time.time() - t0 < 10.0:
            code, data = self.video_client.GetImageSample()
            if code != 0:
                continue
            if data is None or len(data) == 0:
                print("Empty image data")
                continue

            raw = cv2.imdecode(np.frombuffer(bytes(data), np.uint8), cv2.IMREAD_COLOR)
            if raw is None:
                print("Decode failed")
                continue

            head = cv2.fisheye.undistortImage(cv2.resize(raw, (640, 480)), self.K, self.D, Knew=self.NEW_K)
            hsv = cv2.cvtColor(head, cv2.COLOR_BGR2HSV)
            mask = cv2.morphologyEx(
                cv2.inRange(hsv, self.LOWER_YELLOW, self.UPPER_YELLOW),
                cv2.MORPH_OPEN,
                np.ones((5, 5), np.uint8)
            )
            cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            best_label, max_good = -1, 0
            for c in cnts:
                if cv2.contourArea(c) < 1500:
                    continue
                x, y, w, h = cv2.boundingRect(c)
                crop = head[y:y+h, x:x+w]
                if crop.size == 0:
                    continue
                roi = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
                kp_r, des_r = self.ORB.detectAndCompute(roi, None)
                if des_r is not None:
                    for lbl, (kp_t, des_t) in self.templates.items():
                        good = len([m for m in self.BF.match(des_t, des_r) if m.distance < 50])
                        if good > max_good:
                            max_good = good
                            best_label = lbl

            if max_good > self.MATCH_THRESHOLD:
                if best_label == 0:
                    self.sport.Stretch()
                    self.move_continuous(-0.3, 0.0, 0.0, 2.0)
                elif best_label == 1:
                    self.sport.Hello()
                elif best_label == 2:
                    for _ in range(3):
                        self.vui_client.SetBrightness(10)
                        time.sleep(0.5)
                        self.vui_client.SetBrightness(0)
                        time.sleep(0.5)
                time.sleep(2)
                self.sport.RecoveryStand()
                break

            time.sleep(0.1)

        self.move_continuous(0, 0, -0.8, 2.5)  # 回归

    def _start_return_sequence(self, forward_time=1.6):
        """
        进入回航时调用：
        1) 记录回航流程参数
        2) 先前进一段时间
        3) 最终回到比赛开始时的IMU姿态，也就是 rel_yaw = 0
        """
        self.return_phase = 0
        self.return_phase_start = time.time()
        self.return_target_rel_yaw = 0.0
        self.RETURN_FORWARD_TIME = forward_time
        print(f"[RETURN] target heading = {self.return_target_rel_yaw:.2f} deg (startup pose)")

    def do_return_tick(self):
        """
        回航子状态机：
        phase 0: 前进一段距离（用时间近似）
        phase 1: 原地转回最开始记录的 IMU 姿态
        """
        now = time.time()

        # phase 0: 先前进
        if self.return_phase == 0:
            if self.return_phase_start == 0.0:
                self.return_phase_start = now
                if self.return_target_rel_yaw is None:
                    self.return_target_rel_yaw = self._rel_yaw()
                print(f"[RETURN] forward start, target yaw={self.return_target_rel_yaw:.2f}")

            if now - self.return_phase_start < self.RETURN_FORWARD_TIME:
                self.safe_move(self.RETURN_FORWARD_VX, 0.0, 0.0)
                return

            self.safe_move(0, 0, 0, force=True)
            time.sleep(0.15)
            self.return_phase = 1
            print("[RETURN] switch to heading recovery")
            return

        # phase 1: 转回目标朝向
        if self.return_phase == 1:
            rel = self._rel_yaw()
            if rel is None or self.return_target_rel_yaw is None:
                self.safe_move(0, 0, 0.6)
                return

            err = self._wrap_deg(self.return_target_rel_yaw - rel)

            if abs(err) <= self.RETURN_HEADING_TOL:
                print(f"[RETURN] heading recovered: err={err:.2f}, stop.")
                self.safe_move(0, 0, 0, force=True)
                self.running = False
                return

            vyaw = self.RETURN_TURN_GAIN * err
            vyaw = max(-self.RETURN_TURN_MAX, min(self.RETURN_TURN_MAX, vyaw))

            if abs(vyaw) < self.RETURN_TURN_MIN:
                vyaw = self.RETURN_TURN_MIN if err > 0 else -self.RETURN_TURN_MIN

            self.safe_move(0.0, 0.0, vyaw)

    def run(self):
        try:
            while self.running:
                frames = self.pipeline.wait_for_frames(10000)
                frame = np.asanyarray(frames.get_color_frame().get_data())
                now = time.time()

                # 每轮先尽量更新一次IMU
                self._pull_imu_update()

                if now - self.startup_time < self.warmup_seconds:
                    time.sleep(0.05)
                    continue

                if self.state == STATE_FOLLOW_TO_OBSTACLE:
                    if (not self.foam_triggered_state1) and now >= self.foam_action_lock_until:
                        det, _, _, _, _ = detect_line_break(frame)
                        if det:
                            self.foam_triggered_state1 = True
                            self.execute_foam_jump_action()
                            continue
                    self.do_line_follow_tick(frame)
                    if self.lost_line_count > 5:
                        self.move_continuous(self.vx, 0, 0, 1.0)
                        self.set_state(STATE_OBSTACLE_AVOID)

                elif self.state == STATE_OBSTACLE_AVOID:
                    self.do_obstacle_avoid_tick(frame, now)

                elif self.state == STATE_FOLLOW_TO_STAIR:
                    self.do_line_follow_post_stair(frame)
                    if self.aruco_detector.detectMarkers(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY))[1] is not None:
                        self.set_state(STATE_DO_STAIRS)

                elif self.state == STATE_DO_STAIRS:
                    self._execute_stair_sequence()
                    self.set_state(STATE_FOLLOW_TO_RED_DOT)

                elif self.state == STATE_FOLLOW_TO_RED_DOT:
                    # phase 0：去第一个IMU点
                    if self.red_dot_phase == 0:
                        self.do_line_follow_post_stair(frame)

                        if self.imu_ready and (not self.point1_done) and self._yaw_near(0.0):
                            self.point1_done = True
                            self.safe_move(0, 0, 0, force=True)
                            self.pause_5s(">>> 到达第一个位置点（朝北），静止5秒 <<<")
                            self.execute_point1_vision_task()
                            self.red_dot_phase = 1
                            continue

                    # phase 1：去第二个IMU点
                    elif self.red_dot_phase == 1:
                        self.do_line_follow_post_stair(frame)

                        west_now = self._yaw_near(100.0)
                        trigger = False

                        # 1) 抑制期内：完全不允许触发，也不”记账”
                        if self.point2_suppress_until is not None and now < self.point2_suppress_until:
                            self.point2_prev_west = west_now
                            self.point2_wait_fresh_west = False
                        else:
                            # 2) 抑制期刚结束：进入“等待锁外新边沿”模式
                            if self.point2_suppress_until is not None and now >= self.point2_suppress_until:
                                print("[Point2] 抑制结束，等待锁外第一次新的西向进入")
                                self.point2_suppress_until = None
                                self.point2_wait_fresh_west = True
                                self.point2_prev_west = west_now   # 关键：从当前状态重新开始判边沿

                            # 3) 只有锁外出现”False -> True”的新边沿，才触发
                            if self.imu_ready and (not self.point2_done) and self.point1_done and self.point2_wait_fresh_west:
                                if west_now and (not self.point2_prev_west):
                                    trigger = True

                            self.point2_prev_west = west_now

                        if trigger:
                            self.point2_done = True
                            self.point2_suppress_until = None
                            self.safe_move(0, 0, 0, force=True)
                            time.sleep(0.3)

                            # ① 前进
                            print(f">>> 第二个点：前进 {POINT2_FORWARD_DURATION}s <<<")
                            self.move_continuous(POINT2_FORWARD_VX, 0, 0, POINT2_FORWARD_DURATION)
                            self.safe_move(0, 0, 0, force=True)
                            time.sleep(0.3)

                            # ② 向右平移
                            print(f">>> 第二个点：向右平移 {POINT2_RIGHT_DURATION}s <<<")
                            self.move_continuous(0, POINT2_RIGHT_VY, 0, POINT2_RIGHT_DURATION)
                            self.safe_move(0, 0, 0, force=True)
                            time.sleep(0.3)

                            # ③ 机械臂抓取
                            print(">>> 第二个点：启动机械臂抓取 <<<")
                            success = self._call_arm("point2", task="place_grab", direction="right",
                                                    grip_close=POSITION2_GRIP_CLOSE)
                            if success:
                                print("[Point2] 机械臂抓取成功")
                            else:
                                print("[Point2] 机械臂抓取失败/超时，继续流程")

                            # ④ 向左平移归位
                            print(f">>> 第二个点：向左平移归位 {POINT2_RIGHT_DURATION}s <<<")
                            self.move_continuous(0, -POINT2_RIGHT_VY, 0, POINT2_RIGHT_DURATION)
                            self.safe_move(0, 0, 0, force=True)
                            time.sleep(0.3)

                            self.red_dot_phase = 2
                            continue

                    # phase 2：现有红点识别逻辑
                    else:
                        self.do_line_follow_post_stair(frame)
                        if detect_red_circle(frame)[0]:
                            self.set_state(STATE_RED_DOT_ACTION)

                elif self.state == STATE_RED_DOT_ACTION:
                    self._execute_red_dot_sequence()
                    self.set_state(STATE_RETURN)

                elif self.state == STATE_RETURN:
                    # ==============================================
                    # ret_sub_phase = 0: 计时巡线 → 停住 → 机械臂
                    # ==============================================
                    if self.ret_sub_phase == 0:
                        if self.ret_sub_phase_start == 0.0:
                            self.ret_sub_phase_start = now
                            print(f"[RETURN] sub_phase=0: 巡线 {RET_SUB_PHASE0_LINE_TIME}s 后停住做机械臂")

                        if now - self.ret_sub_phase_start < RET_SUB_PHASE0_LINE_TIME:
                            self.do_line_follow_post_stair(frame)
                        else:
                            # 停住
                            self.safe_move(0, 0, 0, force=True)
                            time.sleep(0.3)

                            # 调用机械臂
                            print(">>> 位置三：启动机械臂操作 <<<")
                            success = self._call_arm("point3", task="place", direction="left")
                            if success:
                                print("[Point3] 机械臂操作成功")
                            else:
                                print("[Point3] 机械臂操作失败/超时，继续流程")

                            # 进入子阶段 1
                            self.ret_sub_phase = 1
                            self.ret_sub_phase_start = 0.0
                        continue

                    # ==============================================
                    # ret_sub_phase = 1: 泡沫条检测 + 方位回航（原逻辑）
                    # ==============================================
                    if self.ret_sub_phase == 1:
                        if (not self.foam_triggered_state7) and time.time() >= self.foam_action_lock_until:
                            detected, foam_roi, foam_binary, foam_debug, max_break = detect_line_break(frame)
                            if detected:
                                print("Foam bar detected in STATE 7")
                                self.foam_triggered_state7 = True
                                self.foam_action_lock_until = time.time() + 3.0
                                self.execute_foam_jump_action(use_post_stair=True)

                                # 进入回航子流程：先记当前朝向，再前进，再转回
                                self._start_return_sequence(forward_time=1.6)
                                continue

                        # 回航阶段由 IMU 子状态机接管
                        if self.foam_triggered_state7:
                            self.do_return_tick()
                        else:
                            self.do_line_follow_post_stair(frame)

                time.sleep(0.05)
        finally:
            self.stop()

    def stop(self):
        self.safe_move(0, 0, 0, True)
        self.pipeline.stop()

if __name__ == "__main__":
    robot = RobotFSM(sys.argv[1] if len(sys.argv) > 1 else "eth0")
    robot.init_hardware()
    robot.run()

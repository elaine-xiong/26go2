#!/usr/bin/env python3
# -*- coding: gbk -*-
"""
arm_one_shot.py — 机械臂一次性脚本
由 total10.py 在关键位置点通过 subprocess 调用。

用法:
  python3 arm_one_shot.py --serial 115222070999 --task grab --direction left

任务模式:
  grab       — 只夹取：视觉伺服 → 夹 → 收纳 → exit
  place      — 只放：移动到放物位置 → 松爪 → 归位 → exit（不需要相机）
  place_grab — 先放再夹：放物 → 视觉伺服夹取新物块 → exit

退出码:
  0 = 成功
  1 = 失败（超时 / 标记丢失 / 设备错误 / 相机未找到）
"""

import cv2
import numpy as np
import pyrealsense2 as rs
import time
import math
import sys
import os
import argparse

# servo.py 需要在同一目录
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
try:
    from servo import LobotServo
except ImportError:
    print("[ARM] 错误：找不到 servo.py，请将其放到与本脚本同一目录")
    sys.exit(1)


# =====================  舵机 ID =====================

ID_BASE     = 0
ID_SHOULDER = 1
ID_ELBOW    = 2
ID_GRIPPER  = 4

# =====================  左右姿态预设 =====================
#                BASE  SHOULDER  ELBOW  GRIPPER
# ---------- 初始待机姿态 ----------
POSE_INIT = {
    "left":  {ID_BASE: 165, ID_SHOULDER: 234, ID_ELBOW: 136, ID_GRIPPER: 470},
    "right": {ID_BASE: 890, ID_SHOULDER: 163, ID_ELBOW: 135, ID_GRIPPER: 470},
}

# ---------- 放物姿态（写死，现场调）----------
POSE_PLACE = {
    "left":  {ID_BASE: 165, ID_SHOULDER: 380, ID_ELBOW: 260, ID_GRIPPER: 700},
    "right": {ID_BASE: 890, ID_SHOULDER: 340, ID_ELBOW: 240, ID_GRIPPER: 700},
}

# =====================  ArUco 配置 =====================

MARKER_LENGTH = 20

SERVO_LIMITS = {
    ID_BASE:     (0, 1000),
    ID_SHOULDER: (0, 1000),
    ID_ELBOW:    (0, 1000),
    ID_GRIPPER:  (450, 850),
}

GRIPPER_OPEN = 470
GRIPPER_CLOSE_DEFAULT = 800   # 三棱锥可能还要调，现场改

# PID 参数
PID_HORIZONTAL = {"kp": -0.03, "ki": -0.002, "kd": -0.01, "deadzone": 10, "limit": 6}
PID_VERTICAL   = {"kp": 0.03,  "ki": 0.002,  "kd": 0.01,  "deadzone": 10, "limit": 6}
PID_DISTANCE   = {"kp": 30,     "ki": 0.2,    "kd": 1.0,   "deadzone": 0.02, "limit": 15}

CENTER_THRESHOLD   = 40
MARKER_SIZE_TARGET = 0.3
STABLE_FRAMES      = 3
MAX_LOST_FRAMES    = 30

MOVE_TIME_CORRECT  = 100
MOVE_TIME_APPROACH = 200
MOVE_TIME_GRAB     = 300
MOVE_TIME_STORAGE  = 1000


# =====================  PID =====================

class PID:
    def __init__(self, kp, ki, kd, deadzone, limit):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.deadzone = deadzone
        self.output_limit = limit
        self.reset()

    def reset(self):
        self.prev_error = 0
        self.integral = 0
        self.last_time = None

    def compute(self, error):
        if abs(error) < self.deadzone:
            return 0.0
        now = time.time()
        dt = now - self.last_time if self.last_time else 0.1
        if dt <= 0:
            dt = 0.1
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        output = max(-self.output_limit, min(self.output_limit, output))
        self.prev_error = error
        self.last_time = now
        return output


# =====================  舵机工具函数 =====================

def clamp(pos, sid):
    lo, hi = SERVO_LIMITS[sid]
    return int(max(lo, min(hi, pos)))


def move_one(servos, sid, pos, time_ms):
    pos = clamp(pos, sid)
    servos[sid].move(pos, time_ms)


def move_all(servos, positions, time_ms):
    for sid, pos in positions.items():
        move_one(servos, sid, pos, time_ms)


def goto_init(servos, direction, gripper_pos=None, time_ms=1500):
    """归位到指定方向的待机姿态。gripper_pos 为 None 则用预设值。"""
    pose = POSE_INIT[direction].copy()
    if gripper_pos is not None:
        pose[ID_GRIPPER] = gripper_pos
    print(f"[ARM]   归位 ({direction})...")
    move_all(servos, pose, time_ms)
    time.sleep(time_ms / 1000.0 + 0.5)


def init_servos(port):
    """初始化所有舵机，返回 dict"""
    servos = {}
    for sid in [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_GRIPPER]:
        servos[sid] = LobotServo(port, sid)
    return servos


def close_servos(servos):
    for s in servos.values():
        try:
            s.close()
        except Exception:
            pass


# ==================== 任务：只放 (place) ====================

def do_place(port, direction, grip_close):
    """
    放物任务：不需要相机。
    从当前姿态 → 放物位置 → 松爪 → 归位。
    """
    print("[ARM] ====== 放物任务 ======")
    print(f"[ARM]   方向: {direction}")

    servos = {}
    try:
        servos = init_servos(port)
    except Exception as e:
        print(f"[ARM] 错误：无法连接舵机串口: {e}")
        return False

    try:
        # 1) 归位（夹着物块的状态）
        print("[ARM]   1) 归位到待机姿态...")
        goto_init(servos, direction, gripper_pos=grip_close, time_ms=1500)

        # 2) 移动到放物姿态
        print("[ARM]   2) 移动到放物姿态...")
        place_pose = POSE_PLACE[direction].copy()
        place_pose[ID_GRIPPER] = grip_close  # 移动过程中保持夹紧
        move_all(servos, place_pose, 2000)
        time.sleep(2.5)

        # 3) 松爪
        print("[ARM]   3) 松爪释放...")
        move_one(servos, ID_GRIPPER, GRIPPER_OPEN, 600)
        time.sleep(1.0)

        # 4) 归位
        print("[ARM]   4) 归位...")
        goto_init(servos, direction, gripper_pos=GRIPPER_OPEN, time_ms=1500)

        print("[ARM] >>> 放物成功 <<<")
        return True
    except Exception as e:
        print(f"[ARM] 放物异常: {e}")
        return False
    finally:
        close_servos(servos)


# ==================== 任务：只夹取 (grab) ====================

def do_grab(serial_number, port, overall_timeout, direction, grip_close):
    """
    夹取任务：视觉伺服 → 夹 → 收纳。
    """
    print("[ARM] ====== 夹取任务 ======")
    print(f"[ARM]   方向: {direction}  夹力: {grip_close}")

    start_time = time.time()

    # --- 枚举设备 ---
    print("[ARM] 枚举 RealSense 设备...")
    ctx = rs.context()
    device_serials = [d.get_info(rs.camera_info.serial_number) for d in ctx.devices]
    for d in ctx.devices:
        print(f"[ARM]   发现: {d.get_info(rs.camera_info.name)} 序列号={d.get_info(rs.camera_info.serial_number)}")

    if serial_number not in device_serials:
        print(f"[ARM] 错误：未找到目标相机 序列号={serial_number}")
        print(f"[ARM]   可用设备: {device_serials}")
        return False

    # --- RealSense ---
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device(serial_number)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    try:
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    except Exception:
        print("[ARM]   警告：无法启用深度流")
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"[ARM] 错误：无法启动 RealSense: {e}")
        return False

    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([[intrinsics.fx, 0, intrinsics.ppx],
                  [0, intrinsics.fy, intrinsics.ppy],
                  [0, 0, 1]], dtype=np.float64)
    distCoeffs = np.array(intrinsics.coeffs, dtype=np.float64)
    FRAME_W, FRAME_H = intrinsics.width, intrinsics.height
    FRAME_CENTER = (FRAME_W // 2, int(FRAME_H * 2 / 3))
    FRAME_DIAG = math.sqrt(FRAME_W ** 2 + FRAME_H ** 2)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    half = MARKER_LENGTH / 2.0
    OBJ_POINTS = np.array([
        [-half, -half, 0], [ half, -half, 0],
        [ half,  half, 0], [-half,  half, 0]
    ], dtype=np.float32)

    # --- 舵机 ---
    print(f"[ARM] 初始化舵机，串口={port}")
    servos = {}
    try:
        servos = init_servos(port)
    except Exception as e:
        print(f"[ARM] 错误：无法连接舵机串口: {e}")
        pipeline.stop()
        return False

    pid_h = PID(**PID_HORIZONTAL)
    pid_v = PID(**PID_VERTICAL)
    pid_d = PID(**PID_DISTANCE)

    STATE = "CORRECTING"
    stable_count = 0
    lost_count = 0
    current_positions = POSE_INIT[direction].copy()

    print("[ARM] 视觉伺服启动，归位...")
    goto_init(servos, direction, gripper_pos=GRIPPER_OPEN, time_ms=1500)
    current_positions[ID_GRIPPER] = GRIPPER_OPEN

    try:
        while True:
            elapsed = time.time() - start_time
            if elapsed > overall_timeout:
                print(f"[ARM] 超时 ({overall_timeout}s)，归位退出")
                goto_init(servos, direction, time_ms=1500)
                return False

            frames = pipeline.wait_for_frames(10000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue
            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

            corners, ids, rejected = aruco_detector.detectMarkers(gray)

            if ids is None:
                lost_count += 1
                if lost_count >= 5 and STATE in ("CORRECTING", "APPROACHING"):
                    print("  [ARM] 标记丢失，手臂回缩寻找...")
                    current_positions[ID_SHOULDER] = clamp(current_positions[ID_SHOULDER] - 4, ID_SHOULDER)
                    current_positions[ID_ELBOW] = clamp(current_positions[ID_ELBOW] - 6, ID_ELBOW)
                    print(f"  [ARM] 肩 -4 -> {current_positions[ID_SHOULDER]} | 肘 -6 -> {current_positions[ID_ELBOW]}")
                    move_one(servos, ID_SHOULDER, current_positions[ID_SHOULDER], 300)
                    move_one(servos, ID_ELBOW, current_positions[ID_ELBOW], 300)
                    lost_count = 0
                if lost_count >= MAX_LOST_FRAMES:
                    print(f"[ARM] 连续 {MAX_LOST_FRAMES} 帧丢失标记，失败退出")
                    goto_init(servos, direction, time_ms=1500)
                    return False
                continue

            lost_count = 0

            # 位姿估算
            best_idx, best_size = 0, 0
            for i in range(len(ids)):
                cs = corners[i].reshape(4, 2).astype(np.float32)
                _, rvec, tvec = cv2.solvePnP(OBJ_POINTS, cs, K, distCoeffs)
                diag = (np.linalg.norm(cs[0] - cs[2]) + np.linalg.norm(cs[1] - cs[3])) / 2.0
                if diag > best_size:
                    best_size = diag
                    best_idx = i

            marker_corners = corners[best_idx].reshape(4, 2)
            mx, my = marker_corners.mean(axis=0)
            offset_x = mx - FRAME_CENTER[0]
            offset_y = my - FRAME_CENTER[1]
            marker_ratio = best_size / FRAME_DIAG

            h_adj = pid_h.compute(offset_x)
            v_adj = pid_v.compute(offset_y)
            d_adj = pid_d.compute(MARKER_SIZE_TARGET - marker_ratio)

            center_dist = math.sqrt(offset_x ** 2 + offset_y ** 2)
            center_ok = center_dist < CENTER_THRESHOLD
            size_ok = abs(marker_ratio - MARKER_SIZE_TARGET) < 0.04

            direction_str = "中心"
            if not center_ok:
                v_str = "上" if offset_y < -10 else ("下" if offset_y > 10 else "")
                h_str = "左" if offset_x < -10 else ("右" if offset_x > 10 else "")
                direction_str = f"{v_str}{h_str}" if (v_str or h_str) else "中心"

            delta_b, delta_s, delta_e = 0, 0, 0
            if STATE in ["CORRECTING", "APPROACHING"]:
                if abs(h_adj) >= 1:
                    delta_b += int(h_adj)
                if abs(v_adj) >= 1:
                    delta_s += int(v_adj)
                    delta_e -= int(v_adj)
                if abs(d_adj) >= 1:
                    delta_s += int(d_adj)
                    delta_e += int(d_adj)

            if STATE == "CORRECTING":
                print(f"  [CORRECTING] 目标方位: {direction_str:<4} | 偏移量: X={offset_x:+.0f}, Y={offset_y:+.0f}")
                if center_ok:
                    stable_count += 1
                    if stable_count >= STABLE_FRAMES:
                        print("  >>> 居中完成，进入逼近阶段！")
                        STATE = "APPROACHING"
                        stable_count = 0
                else:
                    stable_count = 0
                    moved = False
                    action_log = "  [纠偏动作] "
                    if delta_b != 0:
                        current_positions[ID_BASE] = clamp(current_positions[ID_BASE] + delta_b, ID_BASE)
                        action_log += f"底座(ID0) {delta_b:+} -> {current_positions[ID_BASE]} | "
                        move_one(servos, ID_BASE, current_positions[ID_BASE], MOVE_TIME_CORRECT)
                        moved = True
                    if delta_s != 0 or delta_e != 0:
                        current_positions[ID_SHOULDER] = clamp(current_positions[ID_SHOULDER] + delta_s, ID_SHOULDER)
                        current_positions[ID_ELBOW] = clamp(current_positions[ID_ELBOW] + delta_e, ID_ELBOW)
                        action_log += f"肩(ID1) {delta_s:+} -> {current_positions[ID_SHOULDER]} | "
                        action_log += f"肘(ID2) {delta_e:+} -> {current_positions[ID_ELBOW]}"
                        move_one(servos, ID_SHOULDER, current_positions[ID_SHOULDER], MOVE_TIME_CORRECT)
                        move_one(servos, ID_ELBOW, current_positions[ID_ELBOW], MOVE_TIME_CORRECT)
                        moved = True
                    if moved:
                        print(action_log)
                    time.sleep(0.08)

            elif STATE == "APPROACHING":
                # 同时满足居中 + 深度足够 → 夹取
                if center_ok and size_ok:
                    stable_count += 1
                    if stable_count >= STABLE_FRAMES:
                        print("  >>> 视觉闭环完全对准，进入夹取！")
                        STATE = "GRAB"
                        stable_count = 0
                # 逼近时偏离中心 → 退回重新纠偏
                elif not center_ok:
                    print("  [!] 逼近时偏离中心，重新纠偏...")
                    STATE = "CORRECTING"
                    stable_count = 0
                else:
                    stable_count = 0
                    moved = False
                    action_log = "  [逼近前伸] "
                    if delta_b != 0:
                        current_positions[ID_BASE] = clamp(current_positions[ID_BASE] + delta_b, ID_BASE)
                        action_log += f"底座(ID0) {delta_b:+} -> {current_positions[ID_BASE]} | "
                        move_one(servos, ID_BASE, current_positions[ID_BASE], MOVE_TIME_CORRECT)
                        moved = True
                    if delta_s != 0 or delta_e != 0:
                        current_positions[ID_SHOULDER] = clamp(current_positions[ID_SHOULDER] + delta_s, ID_SHOULDER)
                        current_positions[ID_ELBOW] = clamp(current_positions[ID_ELBOW] + delta_e, ID_ELBOW)
                        action_log += f"肩(ID1) {delta_s:+} -> {current_positions[ID_SHOULDER]} | "
                        action_log += f"肘(ID2) {delta_e:+} -> {current_positions[ID_ELBOW]}"
                        move_one(servos, ID_SHOULDER, current_positions[ID_SHOULDER], MOVE_TIME_APPROACH)
                        move_one(servos, ID_ELBOW, current_positions[ID_ELBOW], MOVE_TIME_APPROACH)
                        moved = True
                    if moved:
                        print(action_log)
                    time.sleep(0.12)

            elif STATE == "GRAB":
                print("  >>> [夹取] 闭合夹爪！")
                move_one(servos, ID_GRIPPER, grip_close, MOVE_TIME_GRAB)
                time.sleep(0.8)
                STATE = "STORAGE"

            elif STATE == "STORAGE":
                print("  >>> [收纳] 回到初始位置，保持夹紧...")
                storage = POSE_INIT[direction].copy()
                storage[ID_GRIPPER] = grip_close
                move_all(servos, storage, MOVE_TIME_STORAGE)
                time.sleep(MOVE_TIME_STORAGE / 1000.0 + 0.5)
                print("[ARM] >>> 夹取成功 <<<")
                return True

    finally:
        pipeline.stop()
        close_servos(servos)


# =====================  入口 =====================

def main():
    parser = argparse.ArgumentParser(description="机械臂一次性脚本")
    parser.add_argument("--serial", required=True, help="RealSense 相机序列号")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="舵机串口")
    parser.add_argument("--timeout", type=int, default=15, help="整体超时秒数")
    parser.add_argument("--task", default="grab", choices=["grab", "place", "place_grab"],
                        help="任务模式: grab(夹取) / place(放) / place_grab(先放再夹)")
    parser.add_argument("--direction", default="left", choices=["left", "right"],
                        help="方向: left(左) / right(右)")
    parser.add_argument("--grip-close", type=int, default=700,
                        help="夹爪闭合值 (默认700, 三棱锥=800)")
    parser.add_argument("--label", default="unknown", help="位置标签")

    args = parser.parse_args()

    print(f"[ARM] ====== 机械臂启动 ======")
    print(f"[ARM]   位置: {args.label}")
    print(f"[ARM]   任务: {args.task}  方向: {args.direction}")
    print(f"[ARM]   相机: {args.serial}  串口: {args.port}")
    print(f"[ARM]   夹力: {args.grip_close}  超时: {args.timeout}s")

    success = False

    if args.task == "place":
        # 纯放物，不需要相机
        success = do_place(args.port, args.direction, args.grip_close)

    elif args.task == "grab":
        # 纯夹取，视觉伺服
        success = do_grab(args.serial, args.port, args.timeout,
                          args.direction, args.grip_close)

    elif args.task == "place_grab":
        # 先放再夹
        print("[ARM] ---- 阶段1: 放物 ----")
        ok = do_place(args.port, args.direction, args.grip_close)
        if not ok:
            print("[ARM] 放物阶段失败，终止")
            sys.exit(1)
        print("[ARM] ---- 阶段2: 夹取 ----")
        success = do_grab(args.serial, args.port, args.timeout,
                          args.direction, args.grip_close)

    if success:
        print("[ARM] ====== 成功退出 (exit 0) ======")
        sys.exit(0)
    else:
        print("[ARM] ====== 失败退出 (exit 1) ======")
        sys.exit(1)


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
# -*- coding: gbk -*-
"""
arm.py 由 total10.py 在关键位置点通过 subprocess 调用。

内部任务映射说明：
  total10 传 grab       -> 自动执行 pos1_grab (位置一：向左，只夹)
  total10 传 place_grab -> 自动执行 pos2_place_grab (位置二：向右，先放再夹)
  total10 传 place      -> 自动执行 pos3_place (位置三：只放)

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


# ===================== 舵机 ID =====================

ID_BASE = 0
ID_SHOULDER = 1
ID_ELBOW = 2
ID_GRIPPER = 4

# ==================== 安全范围 =====================

SERVO_LIMITS = {
    ID_BASE: (0, 1000),
    ID_SHOULDER: (0, 1000),
    ID_ELBOW: (0, 1000),
    ID_GRIPPER: (450, 850),
}

# ===================== 夹爪状态 =====================

GRIPPER_OPEN = 461
GRIPPER_CLOSE_DEFAULT = 700

# ===================== 固定基准姿态 =====================

MIDDLE_BASE = 561
MIDDLE_SHOULDER = 118
MIDDLE_ELBOW = 248

# 位置一：向左，只夹
POS1_GRAB_BASE = 835
POS1_GRAB_SHOULDER = 162
POS1_GRAB_ELBOW = 248

# 位置二：向右，先放再夹
POS2_PLACE_PREP_BASE = 140
POS2_PLACE_PREP_SHOULDER = 113
POS2_PLACE_PREP_ELBOW = 248

POS2_PLACE_ACTION_BASE = 99
POS2_PLACE_ACTION_SHOULDER = 266
POS2_PLACE_ACTION_ELBOW = 316

POS2_GRAB_BASE = 232
POS2_GRAB_SHOULDER = 164
POS2_GRAB_ELBOW = 316

# 位置三：只放
POS3_PLACE_RIGHT_PREP_BASE = 140
POS3_PLACE_RIGHT_ACTION_BASE = 99

POS3_PLACE_LEFT_PREP_BASE = 890
POS3_PLACE_LEFT_ACTION_BASE = 890


# ===================== 视觉 / PID 参数 =====================

MARKER_LENGTH = 20

PID_HORIZONTAL = {"kp": -0.03, "ki": -0.002, "kd": -0.01, "deadzone": 10, "limit": 6}
PID_VERTICAL   = {"kp": 0.03,   "ki": 0.002,   "kd": 0.01,  "deadzone": 10, "limit": 6}
PID_DISTANCE   = {"kp": 30,     "ki": 0.2,    "kd": 1.0,   "deadzone": 0.02, "limit": 15}

CENTER_THRESHOLD = 40
MARKER_SIZE_TARGET = 0.3
STABLE_FRAMES = 3
MAX_LOST_FRAMES = 30

MOVE_TIME_CORRECT = 100
MOVE_TIME_APPROACH = 200
MOVE_TIME_GRAB = 300
MOVE_TIME_STORAGE = 1500


# ===================== PID =====================

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


# ===================== 姿态生成 =====================

def pose_middle_hold(grip_close):
    return {
        ID_BASE: MIDDLE_BASE,
        ID_SHOULDER: MIDDLE_SHOULDER,
        ID_ELBOW: MIDDLE_ELBOW,
        ID_GRIPPER: grip_close,
    }

def pose_middle_open():
    return {
        ID_BASE: MIDDLE_BASE,
        ID_SHOULDER: MIDDLE_SHOULDER,
        ID_ELBOW: MIDDLE_ELBOW,
        ID_GRIPPER: GRIPPER_OPEN,
    }

def pose_pos1_grab_open():
    return {
        ID_BASE: POS1_GRAB_BASE,
        ID_SHOULDER: POS1_GRAB_SHOULDER,
        ID_ELBOW: POS1_GRAB_ELBOW,
        ID_GRIPPER: GRIPPER_OPEN,
    }

def pose_pos1_grab_hold(grip_close):
    return {
        ID_BASE: POS1_GRAB_BASE,
        ID_SHOULDER: POS1_GRAB_SHOULDER,
        ID_ELBOW: POS1_GRAB_ELBOW,
        ID_GRIPPER: grip_close,
    }

def pose_pos2_place_prep(grip_close):
    return {
        ID_BASE: POS2_PLACE_PREP_BASE,
        ID_SHOULDER: POS2_PLACE_PREP_SHOULDER,
        ID_ELBOW: POS2_PLACE_PREP_ELBOW,
        ID_GRIPPER: grip_close,
    }

def pose_pos2_place_action(grip_close):
    return {
        ID_BASE: POS2_PLACE_ACTION_BASE,
        ID_SHOULDER: POS2_PLACE_ACTION_SHOULDER,
        ID_ELBOW: POS2_PLACE_ACTION_ELBOW,
        ID_GRIPPER: grip_close,
    }

def pose_pos2_grab_open():
    return {
        ID_BASE: POS2_GRAB_BASE,
        ID_SHOULDER: POS2_GRAB_SHOULDER,
        ID_ELBOW: POS2_GRAB_ELBOW,
        ID_GRIPPER: GRIPPER_OPEN,
    }

def pose_pos2_grab_hold(grip_close):
    return {
        ID_BASE: POS2_GRAB_BASE,
        ID_SHOULDER: POS2_GRAB_SHOULDER,
        ID_ELBOW: POS2_GRAB_ELBOW,
        ID_GRIPPER: grip_close,
    }

def pose_pos3_place_prep(direction, grip_close):
    if direction == "right":
        return {
            ID_BASE: POS3_PLACE_RIGHT_PREP_BASE,
            ID_SHOULDER: 113,
            ID_ELBOW: 248,
            ID_GRIPPER: grip_close,
        }
    else:
        return {
            ID_BASE: POS3_PLACE_LEFT_PREP_BASE,
            ID_SHOULDER: 113,
            ID_ELBOW: 248,
            ID_GRIPPER: grip_close,
        }

def pose_pos3_place_action(direction, grip_close):
    if direction == "right":
        return {
            ID_BASE: POS3_PLACE_RIGHT_ACTION_BASE,
            ID_SHOULDER: 266,
            ID_ELBOW: 316,
            ID_GRIPPER: grip_close,
        }
    else:
        return {
            ID_BASE: POS3_PLACE_LEFT_ACTION_BASE,
            ID_SHOULDER: 266,
            ID_ELBOW: 316,
            ID_GRIPPER: grip_close,
        }


# ===================== 舵机及硬件控制工具 =====================

def clamp(pos, sid):
    lo, hi = SERVO_LIMITS[sid]
    return int(max(lo, min(hi, pos)))


def create_servos(port):
    servos = {}
    for sid in [ID_BASE, ID_SHOULDER, ID_ELBOW, ID_GRIPPER]:
        servos[sid] = LobotServo(port, sid)
    return servos


def move_one(servos, sid, pos, time_ms):
    pos = clamp(pos, sid)
    servos[sid].move(pos, time_ms)


def move_all(servos, positions, time_ms):
    for sid, pos in positions.items():
        move_one(servos, sid, pos, time_ms)


def goto_pose(servos, pose, time_ms=1500):
    move_all(servos, pose, time_ms)
    time.sleep(time_ms / 1000.0 + 0.5)


def close_servos(servos):
    for s in servos.values():
        try:
            s.close()
        except Exception:
            pass


def setup_realsense(serial_number):
    print("[ARM] 枚举 RealSense 设备...")
    ctx = rs.context()
    device_serials = [d.get_info(rs.camera_info.serial_number) for d in ctx.devices]
    for d in ctx.devices:
        print(f"[ARM]   发现: {d.get_info(rs.camera_info.name)} 序列号={d.get_info(rs.camera_info.serial_number)}")

    if serial_number not in device_serials:
        print(f"[ARM] 错误：未找到目标相机 序列号={serial_number}")
        print(f"[ARM]   可用设备: {device_serials}")
        return None

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
        return None

    intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
    K = np.array([
        [intrinsics.fx, 0, intrinsics.ppx],
        [0, intrinsics.fy, intrinsics.ppy],
        [0, 0, 1]
    ], dtype=np.float64)
    distCoeffs = np.array(intrinsics.coeffs, dtype=np.float64)
    frame_w, frame_h = intrinsics.width, intrinsics.height
    frame_center = (frame_w // 2, int(frame_h * 2 / 3))
    frame_diag = math.sqrt(frame_w ** 2 + frame_h ** 2)

    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
    aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, cv2.aruco.DetectorParameters())

    half = MARKER_LENGTH / 2.0
    obj_points = np.array([
        [-half, -half, 0], [half, -half, 0],
        [half, half, 0], [-half, half, 0]
    ], dtype=np.float32)

    return {
        "pipeline": pipeline,
        "profile": profile,
        "K": K,
        "distCoeffs": distCoeffs,
        "frame_w": frame_w,
        "frame_h": frame_h,
        "frame_center": frame_center,
        "frame_diag": frame_diag,
        "aruco_detector": aruco_detector,
        "obj_points": obj_points,
    }


# ===================== 视觉抓取闭环 =====================

def visual_grab_loop(
    servos,
    rs_ctx,
    overall_timeout,
    grip_close,
    start_pose_open,
    return_pose_hold,
    middle_pose_hold,
    preview=False,
):
    """
    从“准备位(开)”开始执行视觉伺服抓取：
    - 进入准备位
    - 视觉闭环纠偏 / 逼近
    - 闭合夹爪
    - 回到准备位(夹紧)
    - 回到中间(夹紧)
    """
    pipeline = rs_ctx["pipeline"]
    K = rs_ctx["K"]
    distCoeffs = rs_ctx["distCoeffs"]
    frame_w = rs_ctx["frame_w"]
    frame_h = rs_ctx["frame_h"]
    frame_center = rs_ctx["frame_center"]
    frame_diag = rs_ctx["frame_diag"]
    aruco_detector = rs_ctx["aruco_detector"]
    obj_points = rs_ctx["obj_points"]

    pid_h = PID(**PID_HORIZONTAL)
    pid_v = PID(**PID_VERTICAL)
    pid_d = PID(**PID_DISTANCE)

    STATE = "CORRECTING"
    stable_count = 0
    lost_count = 0
    current_positions = start_pose_open.copy()
    start_time = time.time()

    print("[ARM] 进入抓取准备位...")
    goto_pose(servos, start_pose_open, 1500)

    try:
        while True:
            if time.time() - start_time > overall_timeout:
                print(f"[ARM] 超时 ({overall_timeout}s)，回中间退出")
                goto_pose(servos, middle_pose_hold, 3500)
                return False

            frames = pipeline.wait_for_frames(10000)
            color_frame = frames.get_color_frame()
            if not color_frame:
                continue

            frame = np.asanyarray(color_frame.get_data())
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            corners, ids, rejected = aruco_detector.detectMarkers(gray)

            if preview:
                cv2.circle(frame, frame_center, CENTER_THRESHOLD, (255, 255, 0), 1)
                cv2.line(frame, (frame_center[0], 0), (frame_center[0], frame_h), (255, 0, 0), 1)
                cv2.line(frame, (0, frame_center[1]), (frame_w, frame_center[1]), (255, 0, 0), 1)
                cv2.putText(frame, f"State: {STATE}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            if ids is None:
                lost_count += 1
                if lost_count >= 5 and STATE in ("CORRECTING", "APPROACHING"):
                    print("  [ARM] 标记丢失，手臂回缩寻找...")
                    current_positions[ID_SHOULDER] = clamp(current_positions[ID_SHOULDER] - 6, ID_SHOULDER)
                    current_positions[ID_ELBOW] = clamp(current_positions[ID_ELBOW] - 3, ID_ELBOW)
                    print(f"  [ARM] 肩 -4 -> {current_positions[ID_SHOULDER]} | 肘 -6 -> {current_positions[ID_ELBOW]}")
                    move_one(servos, ID_SHOULDER, current_positions[ID_SHOULDER], 300)
                    move_one(servos, ID_ELBOW, current_positions[ID_ELBOW], 300)
                    lost_count = 0

                if preview:
                    cv2.imshow("Visual Servoing", frame)
                    if cv2.waitKey(1) == 27:
                        print("[ARM] ESC 退出")
                        goto_pose(servos, middle_pose_hold, 3500)
                        return False
                continue

            lost_count = 0

            best_idx, best_size = 0, 0
            for i in range(len(ids)):
                cs = corners[i].reshape(4, 2).astype(np.float32)
                _, rvec, tvec = cv2.solvePnP(obj_points, cs, K, distCoeffs)
                diag = (np.linalg.norm(cs[0] - cs[2]) + np.linalg.norm(cs[1] - cs[3])) / 2.0
                if diag > best_size:
                    best_size = diag
                    best_idx = i

            marker_corners = corners[best_idx].reshape(4, 2)
            mx, my = marker_corners.mean(axis=0)
            offset_x = mx - frame_center[0]
            offset_y = my - frame_center[1]
            marker_ratio = best_size / frame_diag

            if preview:
                cv2.aruco.drawDetectedMarkers(frame, corners)
                cv2.circle(frame, (int(mx), int(my)), 5, (0, 0, 255), 2)

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

            if preview:
                cv2.putText(frame, f"DIR: {direction_str}", (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 165, 255), 2)

            print(f"  [{STATE}] 目标方位: {direction_str:<4} | 偏移量: X={offset_x:+.0f}, Y={offset_y:+.0f}")

            delta_b, delta_s, delta_e = 0, 0, 0
            if STATE in ("CORRECTING", "APPROACHING"):
                if abs(h_adj) >= 1:
                    delta_b += int(h_adj)
                if abs(v_adj) >= 1:
                    delta_s += int(v_adj)
                    delta_e -= int(v_adj)
                if abs(d_adj) >= 1:
                    delta_s += int(d_adj)
                    delta_e += int(d_adj)

            if STATE == "CORRECTING":
                if center_ok:
                    stable_count += 1
                    if stable_count >= STABLE_FRAMES:
                        print("  >>> 居中完成，进入逼近阶段！")
                        STATE = "APPROACHING"
                        stable_count = 0
                else:
                    stable_count = 0
                    action_log = "  [纠偏动作] "
                    moved = False

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
                if center_ok and size_ok:
                    stable_count += 1
                    if stable_count >= STABLE_FRAMES:
                        print("  >>> 视觉闭环完全对准，进入夹取！")
                        STATE = "GRAB"
                        stable_count = 0
                elif not center_ok:
                    print("  [!] 逼近时偏离中心，重新纠偏...")
                    STATE = "CORRECTING"
                    stable_count = 0
                else:
                    stable_count = 0
                    action_log = "  [逼近前伸] "
                    moved = False

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
                print("  >>> [收纳] 回到抓取准备状态(夹紧)...")
                goto_pose(servos, return_pose_hold, MOVE_TIME_STORAGE)
                time.sleep(MOVE_TIME_STORAGE / 1000.0 + 0.5)

                print("  >>> [回中间] 保持夹紧...")
                goto_pose(servos, middle_pose_hold, 3500)

                print("[ARM] >>> 夹取成功 <<<")
                return True

            if preview:
                cv2.imshow("Visual Servoing", frame)
                key = cv2.waitKey(1)
                if key == 27:
                    print("[ARM] ESC 退出")
                    goto_pose(servos, middle_pose_hold, 3500)
                    return False
                elif key == ord('r'):
                    print("[ARM] 手动复位")
                    goto_pose(servos, start_pose_open, 1500)
                    current_positions = start_pose_open.copy()
                    STATE = "CORRECTING"
                    stable_count = 0
                    pid_h.reset()
                    pid_v.reset()
                    pid_d.reset()
                elif key == ord('g'):
                    print("[ARM] 手动触发 GRAB")
                    STATE = "GRAB"

    finally:
        if preview:
            cv2.destroyAllWindows()


# ===================== 任务一：向左，只夹 =====================

def run_pos1_grab(serial_number, port, overall_timeout, grip_close, preview=False):
    print("[ARM] ====== 位置一：向左，只夹 ======")

    rs_ctx = setup_realsense(serial_number)
    if rs_ctx is None:
        return False

    servos = create_servos(port)
    try:
        start_pose_open = pose_pos1_grab_open()
        return_pose_hold = pose_pos1_grab_hold(grip_close)
        middle_pose_hold = pose_middle_hold(grip_close)

        ok = visual_grab_loop(
            servos=servos,
            rs_ctx=rs_ctx,
            overall_timeout=overall_timeout,
            grip_close=grip_close,
            start_pose_open=start_pose_open,
            return_pose_hold=return_pose_hold,
            middle_pose_hold=middle_pose_hold,
            preview=preview,
        )
        return ok

    except Exception as e:
        print(f"[ARM] 运行位置一发生异常: {e}")
        return False
    finally:
        try:
            rs_ctx["pipeline"].stop()
        except Exception:
            pass
        close_servos(servos)


# ===================== 任务二：向右，先放再夹 =====================

def run_pos2_place_grab(serial_number, port, overall_timeout, grip_close, preview=False):
    print("[ARM] ====== 位置二：向右，先放再夹 ======")

    rs_ctx = setup_realsense(serial_number)
    if rs_ctx is None:
        return False

    servos = create_servos(port)
    try:
        print("[ARM] ---- 阶段1：放置准备状态 ----")
        goto_pose(servos, pose_pos2_place_prep(grip_close), 1500)

        print("[ARM] ---- 阶段2：直接放 ----")
        action_pose = pose_pos2_place_action(grip_close)
        move_all(servos, action_pose, 2000)
        time.sleep(2.5)

        print("[ARM] ---- 阶段3：松爪释放 ----")
        move_one(servos, ID_GRIPPER, GRIPPER_OPEN, 600)
        time.sleep(1.0)

        print("[ARM] ---- 阶段4：回到抓取准备状态(开) ----")
        start_pose_open = pose_pos2_grab_open()
        goto_pose(servos, start_pose_open, 1500)

        print("[ARM] ---- 阶段5：夹取动作 ----")
        return_pose_hold = pose_pos2_grab_hold(grip_close)
        middle_pose_hold = pose_middle_hold(grip_close)

        ok = visual_grab_loop(
            servos=servos,
            rs_ctx=rs_ctx,
            overall_timeout=overall_timeout,
            grip_close=grip_close,
            start_pose_open=start_pose_open,
            return_pose_hold=return_pose_hold,
            middle_pose_hold=middle_pose_hold,
            preview=preview,
        )
        return ok

    except Exception as e:
        print(f"[ARM] 运行位置二发生异常: {e}")
        return False
    finally:
        try:
            rs_ctx["pipeline"].stop()
        except Exception:
            pass
        close_servos(servos)


# ===================== 任务三：只放 =====================

def run_pos3_place(serial_number, port, overall_timeout, direction, grip_close, preview=False):
    print(f"[ARM] ====== 位置三：只放（方向: {direction}）======")

    servos = create_servos(port)
    try:
        print("[ARM] ---- 阶段1：准备状态 ----")
        prep_pose = pose_pos3_place_prep(direction, grip_close)
        goto_pose(servos, prep_pose, 1500)

        print("[ARM] ---- 阶段2：直接放 ----")
        action_pose = pose_pos3_place_action(direction, grip_close)
        move_all(servos, action_pose, 2000)
        time.sleep(2.5)

        print("[ARM] ---- 阶段3：松爪释放 ----")
        move_one(servos, ID_GRIPPER, GRIPPER_OPEN, 600)
        time.sleep(1.0)

        print("[ARM] ---- 阶段4：回到中间(打开) ----")
        goto_pose(servos, pose_middle_open(), 1500)

        print("[ARM] >>> 放置成功 <<<")
        return True

    except Exception as e:
        print(f"[ARM] 运行位置三发生异常: {e}")
        return False
    finally:
        close_servos(servos)


# ===================== 入口 =====================

def main():
    parser = argparse.ArgumentParser(description="机械臂一次性脚本 (整合联动版)")
    parser.add_argument("--serial", required=True, help="RealSense 相机序列号")
    parser.add_argument("--port", default="/dev/ttyUSB0", help="舵机串口")
    parser.add_argument("--timeout", type=int, default=15, help="整体超时秒数")
    
    # 兼容原有的 "grab", "place", "place_grab"，以及新定义的任务名
    parser.add_argument("--task", default="pos1_grab", 
                        choices=["grab", "place", "place_grab", "pos1_grab", "pos2_place_grab", "pos3_place"],
                        help="任务模式 (做了兼容映射)")
    parser.add_argument("--direction", default="left", choices=["left", "right"], help="只放任务的方向")
    parser.add_argument("--grip-close", type=int, default=GRIPPER_CLOSE_DEFAULT, help="夹爪闭合值")
    parser.add_argument("--label", default="unknown", help="位置标签(旧版本兼容，不强制依赖)")
    
    # 默认无头模式（不显示 cv2 窗口），如果传入 --preview 则开启实时视觉反馈
    parser.add_argument("--preview", action="store_true", help="开启 OpenCV 视觉窗口显示")

    args = parser.parse_args()

    # ---- 任务映射核心逻辑 ----
    # 保证总代码 total10.py 用旧指令传进来时，能自动映射到你的这套新逻辑上
    actual_task = args.task
    if actual_task == "grab":
        actual_task = "pos1_grab"
    elif actual_task == "place_grab":
        actual_task = "pos2_place_grab"
    elif actual_task == "place":
        actual_task = "pos3_place"

    print(f"[ARM] ====== 机械臂启动 ======")
    print(f"[ARM]   接收参数: task={args.task}, direction={args.direction}, label={args.label}")
    print(f"[ARM]   映射任务: 执行 {actual_task}")
    print(f"[ARM]   设备配置: 相机={args.serial}, 串口={args.port}")
    print(f"[ARM]   闭合夹力: {args.grip_close}, 超时={args.timeout}s, 视觉预览={args.preview}")

    success = False

    try:
        if actual_task == "pos1_grab":
            success = run_pos1_grab(
                serial_number=args.serial,
                port=args.port,
                overall_timeout=args.timeout,
                grip_close=args.grip_close,
                preview=args.preview,
            )
        elif actual_task == "pos2_place_grab":
            success = run_pos2_place_grab(
                serial_number=args.serial,
                port=args.port,
                overall_timeout=args.timeout,
                grip_close=args.grip_close,
                preview=args.preview,
            )
        elif actual_task == "pos3_place":
            success = run_pos3_place(
                serial_number=args.serial,
                port=args.port,
                overall_timeout=args.timeout,
                direction=args.direction,
                grip_close=args.grip_close,
                preview=args.preview,
            )
        else:
            print(f"[ARM] 错误：不支持的任务类型 {actual_task}")
            success = False

        if success:
            print("[ARM] ====== 成功退出 (exit 0) ======")
            sys.exit(0)
        else:
            print("[ARM] ====== 失败退出 (exit 1) ======")
            sys.exit(1)

    except KeyboardInterrupt:
        print("\n[ARM] KeyboardInterrupt (Ctrl+C)")
        sys.exit(1)


if __name__ == "__main__":
    main()
# -*- coding: utf-8 -*-
# 从下台阶以后的线都可以完成循迹，增加黑线区域占比和线宽判定，提升鲁棒性
import pyrealsense2 as rs
import numpy as np
import cv2
import time
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.go2.sport.sport_client import SportClient

def detect_track_line(roi):
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    blur = cv2.GaussianBlur(gray, (5, 5), 0)
    _, binary = cv2.threshold(blur, 80, 255, cv2.THRESH_BINARY_INV)
    # 保留形态学操作有助于去噪，使面积计算更准确
    binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
    
    contours, _ = cv2.findContours(binary, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    roi_h, roi_w = roi.shape[:2]
    roi_area = roi_h * roi_w

    best = None
    best_area = -1  # 改为记录最大面积，因为不再需要复杂评分

    # --- 核心参数 ---
    MIN_AREA = 300                 # 最小黑色面积
    MIN_AREA_RATIO = 0.20         # 黑线区域占ROI比例

    for c in contours:
        area = cv2.contourArea(c)
        
        # 逻辑1：判断最小黑色面积
        if area < MIN_AREA:
            continue

        # 逻辑2：判断黑线区域占比
        area_ratio = area / roi_area
        if area_ratio < MIN_AREA_RATIO:
            continue

        # 如果满足以上两个条件，我们选择面积最大的那个作为目标
        # (也可以改为第一个遇到的，但选最大的通常更稳定)
        if area > best_area:
            best_area = area
            # 为了后续画框和计算中心点，仍然需要获取 boundingRect
            x, y, w, h = cv2.boundingRect(c)
            # 返回格式保持与原代码兼容，虽然 fill_ratio 和 aspect 不再用于筛选，
            # 但为了主程序不报错，我们可以填入默认值或实际计算值（仅用于显示）
            fill_ratio = area / max(w * h, 1)
            aspect = w / float(h) if h > 0 else 0
            best = (x, y, w, h, area_ratio, fill_ratio, aspect)

    return best, binary

ChannelFactoryInitialize(0, "eth0")
sport = SportClient()
sport.SetTimeout(10.0)
sport.Init()
time.sleep(2)

print("RecoveryStand")
sport.RecoveryStand()
time.sleep(3)
sport.SpeedLevel(1)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)
print("D435i ok")

vx = 0.28
k = 0.0035
smooth_error = 0.0

startup_time = time.time()
warmup_seconds = 2.0

try:
    while True:
        frames = pipeline.wait_for_frames(timeout_ms=10000)
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())
        height, width, _ = frame.shape
        roi = frame[int(height * 0.7):height, :]   # 适当缩小到更靠近地面的区域

        if time.time() - startup_time < warmup_seconds:
            cv2.imshow("roi", roi)
            print("camera warming up")
            if cv2.waitKey(1) == ord('q'):
                break
            continue

        best, binary = detect_track_line(roi)
        found_line = False

        if best is not None:
            found_line = True
            x, y, w, h, area_ratio, fill_ratio, aspect = best
            line_center = x + w // 2
            image_center = roi.shape[1] // 2
            error = line_center - image_center

            smooth_error = 0.8 * smooth_error + 0.2 * error
            vyaw = -k * smooth_error
            vyaw = max(min(vyaw, 0.9), -0.9)

            sport.Move(vx, 0, vyaw)

            print(f"error={int(error)} smooth={int(smooth_error)} vyaw={round(vyaw,3)} area={round(area_ratio,4)} fill={round(fill_ratio,3)} asp={round(aspect,2)}")

            cv2.rectangle(roi, (x, y), (x + w, y + h), (0, 255, 0), 2)
            cv2.circle(roi, (line_center, y + h // 2), 5, (0, 0, 255), -1)
            cv2.putText(roi, f"a={area_ratio:.3f} f={fill_ratio:.2f} asp={aspect:.2f}", (x, max(20, y - 10)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        if not found_line:
            sport.Move(0, 0, 0)
            print("lost line")

        cv2.imshow("roi", roi)
        cv2.imshow("binary", binary)

        if cv2.waitKey(1) == ord('q'):
            break

finally:
    print("stop")
    sport.Move(0, 0, 0)
    pipeline.stop()
    cv2.destroyAllWindows()
    time.sleep(1)
    sport.StopMove()
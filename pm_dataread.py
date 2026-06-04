# -*- coding: utf-8 -*-

import cv2
import numpy as np
import pyrealsense2 as rs

BLACK_THRESH = 70

ROI_TOP = 0.55
ROI_BOTTOM = 0.90

pipeline = rs.pipeline()

config = rs.config()

config.enable_stream(
    rs.stream.color,
    424,
    240,
    rs.format.bgr8,
    30
)

pipeline.start(config)

print("debug started")

try:

    while True:

        frames = pipeline.wait_for_frames(timeout_ms=10000)

        color_frame = frames.get_color_frame()

        if not color_frame:
            continue

        frame = np.asanyarray(color_frame.get_data())

        h, w = frame.shape[:2]

        roi = frame[
            int(h * ROI_TOP):int(h * ROI_BOTTOM),
            :
        ]

        roi_h, roi_w = roi.shape[:2]

        # =========================
        # 黑线提取
        # =========================

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        blur = cv2.GaussianBlur(gray, (5,5), 0)

        _, binary = cv2.threshold(
            blur,
            BLACK_THRESH,
            255,
            cv2.THRESH_BINARY_INV
        )

        # =========================
        # 每一行黑像素比例
        # =========================

        break_count = 0
        max_break = 0

        debug = cv2.cvtColor(binary, cv2.COLOR_GRAY2BGR)

        for y in range(roi_h):

            row = binary[y, :]

            black_pixels = cv2.countNonZero(row)

            ratio = black_pixels / float(roi_w)

            # 打印每行数值（可取消）
            # print(f"row={y} ratio={ratio:.3f}")

            # 小于阈值认为这一行“断”
            if ratio < 0.08:

                break_count += 1

                cv2.line(
                    debug,
                    (0, y),
                    (roi_w, y),
                    (0,0,255),
                    1
                )

            else:
                break_count = 0

            max_break = max(max_break, break_count)

        # =========================
        # 输出当前数值
        # =========================

        print("max_break =", max_break)

        cv2.putText(
            debug,
            f"max_break={max_break}",
            (20,40),
            cv2.FONT_HERSHEY_SIMPLEX,
            1,
            (0,255,0),
            2
        )

        cv2.imshow("roi", roi)

        cv2.imshow("binary", binary)

        cv2.imshow("debug", debug)

        if cv2.waitKey(1) == ord('q'):
            break

finally:

    pipeline.stop()

    cv2.destroyAllWindows()
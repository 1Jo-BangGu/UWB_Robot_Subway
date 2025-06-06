import cv2
import numpy as np
import cv2.aruco as aruco
import math

def calculate_yaw_from_aruco(corners):
    c = corners[0]
    top_center = (c[0] + c[1]) / 2.0
    bottom_center = (c[2] + c[3]) / 2.0
    dx = bottom_center[0] - top_center[0]
    dy = bottom_center[1] - top_center[1]
    yaw_rad = math.atan2(dy, dx)
    yaw_deg = math.degrees(yaw_rad)
    if yaw_deg < 0:
        yaw_deg += 360
    return yaw_deg

def detect_robot_position(frame, aruco_dict, parameters, M_pixel2real):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        for corner in corners:
            c = corner[0]
            cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
            px = np.array([[[cx, cy]]], dtype=np.float32)
            real = cv2.perspectiveTransform(px, M_pixel2real)
            rx, ry = real[0][0]
            yaw = calculate_yaw_from_aruco(corner)
            return (rx, ry, yaw), (cx, cy)
    return None, None
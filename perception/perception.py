# import cv2
# import numpy as np
# import math
# import cv2.aruco as aruco

# aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
# parameters = aruco.DetectorParameters()

# def calculate_yaw_from_aruco(corners):
#     c = corners[0]
#     top_center = (c[0] + c[1]) / 2.0
#     bottom_center = (c[2] + c[3]) / 2.0
#     dx = bottom_center[0] - top_center[0]
#     dy = bottom_center[1] - top_center[1]
#     yaw_rad = math.atan2(dy, dx)
#     yaw_deg = math.degrees(yaw_rad)
#     return yaw_deg + 360 if yaw_deg < 0 else yaw_deg

# def detect_robot_position(frame, M_pixel2real):
#     gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
#     corners, ids, _ = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
#     if ids is not None:
#         for corner, id_ in zip(corners, ids.flatten()):
#             c = corner[0]
#             cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
#             px = np.array([[[cx, cy]]], dtype=np.float32)
#             real = cv2.perspectiveTransform(px, M_pixel2real)
#             rx, ry = real[0][0]
#             yaw = calculate_yaw_from_aruco(corner)
#             return (rx, ry, yaw), (cx, cy)
#     return None, None


import cv2
import numpy as np
import math
import cv2.aruco as aruco
from packaging import version

# ArUco 마커 탐지기 초기화
if version.parse(cv2.__version__) >= version.parse("4.7.0"):
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, parameters)
else:
    aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_50)
    parameters = aruco.DetectorParameters_create()
    detector = None

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

def detect_robot_position(frame, M_pixel2real):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if detector is not None:
        corners, ids, rejected = detector.detectMarkers(gray)
    else:
        corners, ids, rejected = aruco.detectMarkers(gray, aruco_dict, parameters=parameters)
    if ids is not None:
        for corner, id_ in zip(corners, ids.flatten()):
            c = corner[0]
            cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
            px = np.array([[[cx, cy]]], dtype=np.float32)
            real = cv2.perspectiveTransform(px, M_pixel2real)
            rx, ry = real[0][0]
            yaw = calculate_yaw_from_aruco(corner)
            return (rx, ry, yaw), (cx, cy)
    return None, (None, None)

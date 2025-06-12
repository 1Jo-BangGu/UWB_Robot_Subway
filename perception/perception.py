# perception.py

import cv2
import numpy as np
import math
import os
import cv2.aruco as aruco

class Perception:
    def __init__(self, world_points):
        # 좌표 및 상태
        self.world_points = world_points
        self.clicked_points = []
        self.obstacle_clicks = []
        self.goal_clicks = []
        self.transform_calculated = False
        self.obstacle_input_done = False
        self.goal_input_done = False

        # 보정 파라미터
        this_dir = os.path.dirname(__file__)
        self.K = np.load(os.path.join(this_dir, "camera_matrix.npy"))
        self.dist = np.load(os.path.join(this_dir, "dist_coeffs.npy"))
        self.new_K = None

        self.M_pixel2real = None
        self.M_real2pixel = None

        # 아루코 딕셔너리
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.parameters = aruco.DetectorParameters()

    def init_new_K(self, frame):
        h, w = frame.shape[:2]
        self.new_K, _ = cv2.getOptimalNewCameraMatrix(self.K, self.dist, (w, h), 1, (w, h))

    def undistort_frame(self, frame):
        return cv2.undistort(frame, self.K, self.dist, None, self.new_K)

    def handle_mouse_event(self, event, x, y):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        if not self.transform_calculated:
            if len(self.clicked_points) < 4:
                self.clicked_points.append([x, y])
                print(f"📌 클릭 {len(self.clicked_points)}/4: ({x}, {y})")
            if len(self.clicked_points) == 4:
                src_pts = np.array(self.clicked_points, dtype=np.float32)
                self.M_pixel2real = cv2.getPerspectiveTransform(src_pts, self.world_points)
                self.M_real2pixel = np.linalg.inv(self.M_pixel2real)
                self.transform_calculated = True
                print("✅ 4점 클릭 완료 및 변환 행렬 계산 완료")
        elif not self.obstacle_input_done:
            self.obstacle_clicks.append([x, y])
            print(f"🟡 장애물 클릭: ({x}, {y})")
        elif not self.goal_input_done:
            self.goal_clicks.append([x, y])
            print(f"🎯 목표 클릭: ({x}, {y})")

    def calculate_yaw_from_aruco(self, corners):
        c = corners[0]
        top_center = (c[0] + c[1]) / 2.0
        bottom_center = (c[2] + c[3]) / 2.0
        dx = bottom_center[0] - top_center[0]
        dy = bottom_center[1] - top_center[1]
        yaw_rad = math.atan2(dy, dx)
        yaw_deg = math.degrees(yaw_rad)
        # return yaw_deg + 360 if yaw_deg < 0 else yaw_deg
        yaw_deg = ((yaw_deg + 180) % 360) - 180
        return yaw_deg

    def detect_robot_position(self, frame):
        if not self.transform_calculated or self.M_pixel2real is None:
            return None, None
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)
        if ids is not None:
            for corner, id_ in zip(corners, ids.flatten()):
                c = corner[0]
                cx, cy = int(np.mean(c[:, 0])), int(np.mean(c[:, 1]))
                px = np.array([[[cx, cy]]], dtype=np.float32)
                real = cv2.perspectiveTransform(px, self.M_pixel2real)
                rx, ry = real[0][0]
                yaw = self.calculate_yaw_from_aruco(corner)
                return (rx, ry, yaw), (cx, cy)
        return None, None

    def get_obstacle_list(self):
        if self.M_pixel2real is None:
            return []
        return [tuple(cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), self.M_pixel2real)[0][0]) + (5,) for x, y in self.obstacle_clicks]

    def get_goal_list(self):
        if self.M_pixel2real is None:
            return []
        return [tuple(cv2.perspectiveTransform(np.array([[[x, y]]], dtype=np.float32), self.M_pixel2real)[0][0]) for x, y in self.goal_clicks]

    def reset(self):
        self.clicked_points.clear()
        self.obstacle_clicks.clear()
        self.goal_clicks.clear()
        self.transform_calculated = False
        self.obstacle_input_done = False
        self.goal_input_done = False
        self.M_pixel2real = None
        self.M_real2pixel = None
        self.new_K = None

    def perception_def(self, frame):
        if self.new_K is None:
            raise ValueError("new_K is not initialized. Call init_new_K() first.")
        frame = self.undistort_frame(frame)
        robot_pose = None
        robot_center = None
        result = self.detect_robot_position(frame)
        if result is not None and result[0] is not None:
            robot_pose = result[0]
            robot_center = result[1]
        obstacle_list = self.get_obstacle_list()
        goal_list_cm = self.get_goal_list()
        return robot_pose, obstacle_list, goal_list_cm, frame, robot_center

    
    def draw_visuals(self, frame, robot_pose, path, robot_center=None):
        # =================== 🔴 클릭한 4점 (빨간 점만 path 없을 때 표시) ===================
        if not path:
            for pt in self.clicked_points:
                cv2.circle(frame, tuple(pt), 8, (0, 0, 255), -1)

        # ✅ 초록 사각형 라인 (협조 표시)
        if len(self.clicked_points) == 4:
            pts = np.array(self.clicked_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # =================== 🟡 장애물 표시 ===================
        if self.M_real2pixel is not None:
            for (ox, oy, r) in self.get_obstacle_list():
                center_px = cv2.perspectiveTransform(np.array([[[ox, oy]]], dtype=np.float32), self.M_real2pixel)[0][0].astype(int)
                cv2.circle(frame, tuple(center_px), int(r), (0, 255, 255), 2)
                cv2.putText(frame, "Obstacle", (center_px[0]+5, center_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

        # =================== 🔸 목표들 표시 ===================
        if not self.goal_input_done and self.goal_clicks:
            for pt in self.goal_clicks:
                cv2.circle(frame, tuple(pt), 8, (255, 0, 255), -1)
                cv2.putText(frame, "Goal", (pt[0]+5, pt[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1)

        # =================== 🕃 경로 표시 ===================
        if path and self.M_real2pixel is not None:
            for i in range(1, len(path)):
                pt1 = cv2.perspectiveTransform(np.array([[[path[i-1][0], path[i-1][1]]]], dtype=np.float32), self.M_real2pixel)[0][0]
                pt2 = cv2.perspectiveTransform(np.array([[[path[i][0], path[i][1]]]], dtype=np.float32), self.M_real2pixel)[0][0]
                cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (255, 0, 255), 2)

            # 시작점 & 도착점
            start_px = cv2.perspectiveTransform(np.array([[[path[0][0], path[0][1]]]], dtype=np.float32), self.M_real2pixel)[0][0]
            end_px = cv2.perspectiveTransform(np.array([[[path[-1][0], path[-1][1]]]], dtype=np.float32), self.M_real2pixel)[0][0]
            cv2.circle(frame, tuple(start_px.astype(int)), 6, (0, 255, 0), -1)
            cv2.putText(frame, "Start", (int(start_px[0]+5), int(start_px[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 1)
            cv2.circle(frame, tuple(end_px.astype(int)), 6, (0, 0, 255), -1)
            cv2.putText(frame, "Goal", (int(end_px[0]+5), int(end_px[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,0,255), 1)

        # =================== 🔵 로벏 자세 표시 ===================
        if robot_pose and robot_center:
            x, y, yaw = robot_pose
            cx, cy = robot_center
            cv2.putText(frame, f"X:{x:.1f} Y:{y:.1f} Yaw:{yaw:.1f}",
                        (int(cx)+10, int(cy)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

        # =================== 📋 상태 안내 텍스트 ===================
        if not path:
            cv2.putText(frame, f"Points: {len(self.clicked_points)}/4", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

        if not self.transform_calculated:
            cv2.putText(frame, "Click 4 corners to define map area", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        elif not self.obstacle_input_done:
            cv2.putText(frame, "Click to add obstacles (Press 'd' when done)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        elif not self.goal_input_done:
            cv2.putText(frame, "Click to add goal points (Press 'g' when done)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

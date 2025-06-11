import cv2
import numpy as np
import time
from perception.perception import detect_robot_position
from planning.planner import a_star, find_nearest_goal
from control.controller import follow_path
from network.client import RaspiVisionClient
from perception.CamCalibration import undistort_frame, get_new_K
from ultralytics import YOLO

# 보정용 매핑 좌표계
world_points = np.array([[0, 0], [0, 90], [180, 90], [180, 0]], dtype=np.float32)

# 전역 상태
clicked_points = []
obstacle_clicks = []
goal_clicks = []
M_pixel2real = None
M_real2pixel = None
transform_calculated = False
obstacle_input_done = False
goal_input_done = False
robot_position = None
latest_path = []

# 학습 파일 로드
model = YOLO("C:/Users/User/Desktop/Pinky_Git/UWB_Robot_Subway/best/fire_fall_integrated_best.pt")

def mouse_callback(event, x, y, flags, param):
    global clicked_points, obstacle_clicks, goal_clicks
    global M_pixel2real, M_real2pixel, transform_calculated, obstacle_input_done, goal_input_done
    if event == cv2.EVENT_LBUTTONDOWN:
        if not transform_calculated:
            if len(clicked_points) < 4:
                clicked_points.append([x, y])
                print(f"Clicked point {len(clicked_points)}: ({x}, {y})")
            if len(clicked_points) == 4:
                src_pts = np.array(clicked_points, dtype=np.float32)
                M_pixel2real = cv2.getPerspectiveTransform(src_pts, world_points)
                M_real2pixel = np.linalg.inv(M_pixel2real)
                transform_calculated = True
                print("✅ Perspective transform calculated")
        elif not obstacle_input_done:
            obstacle_clicks.append([x, y])
            print(f"🟡 Obstacle point added: ({x}, {y})")
        elif not goal_input_done:
            goal_clicks.append([x, y])
            print(f"🎯 Goal point added: ({x}, {y})")

if __name__ == "__main__":
    # ▶️ 클라이언트 1: Robot View용 (global stream)
    client = RaspiVisionClient("192.168.201.193")
    client.start()

    # ▶️ 클라이언트 2: Raspi View용 (raspi stream)
    raspi_client = RaspiVisionClient("192.168.201.193")
    raspi_client.stream_url = f"http://192.168.201.193:5000/stream/raspi"
    raspi_client.start()

    while client.latest_frame is None:
        time.sleep(0.1)

    new_K = get_new_K(client.latest_frame)
    cv2.namedWindow("Robot View")
    cv2.namedWindow("Raspi View")
    cv2.setMouseCallback("Robot View", mouse_callback)

    while True:
        # 🖼 Robot View 프레임 처리
        with client.queue_lock:
            frame = client.latest_frame.copy() if client.latest_frame is not None else None
        if frame is None:
            continue
        frame = undistort_frame(frame, new_K)

        # 좌표계 정의 시각화
        for pt in clicked_points:
            cv2.circle(frame, tuple(pt), 8, (0, 0, 255), -1)
        if len(clicked_points) == 4:
            pts = np.array(clicked_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # 장애물 및 목표 리스트 (실세계 좌표)
        obstacle_list = []
        goal_list_cm = []
        if M_pixel2real is not None:
            for pt in obstacle_clicks:
                real = cv2.perspectiveTransform(np.array([[[pt[0], pt[1]]]], dtype=np.float32), M_pixel2real)[0][0]
                obstacle_list.append((real[0], real[1], 10))
            for pt in goal_clicks:
                real = cv2.perspectiveTransform(np.array([[[pt[0], pt[1]]]], dtype=np.float32), M_pixel2real)[0][0]
                goal_list_cm.append((real[0], real[1]))

        # 키 입력 처리
        key = cv2.waitKey(1)
        if key == ord('d'):
            obstacle_input_done = True
        elif key == ord('g'):
            goal_input_done = True
        elif key == ord('r'):
            clicked_points.clear()
            obstacle_clicks.clear()
            goal_clicks.clear()
            M_pixel2real = None
            M_real2pixel = None
            robot_position = None
            latest_path = []
            transform_calculated = False
            obstacle_input_done = False
            goal_input_done = False
        elif key == 27:
            break

        # 로봇 위치 및 경로 계산
        if transform_calculated and obstacle_input_done and goal_input_done and M_pixel2real is not None:
            robot_pose, (cx, cy) = detect_robot_position(frame, M_pixel2real)
            if robot_pose is not None:
                robot_position = robot_pose
                cv2.putText(frame, f"X:{robot_pose[0]:.1f} Y:{robot_pose[1]:.1f} Yaw:{robot_pose[2]:.1f}", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)
                if goal_list_cm:
                    goal = find_nearest_goal((robot_pose[0], robot_pose[1]), goal_list_cm)
                    latest_path = a_star((robot_pose[0], robot_pose[1]), goal, (0, 180, 0, 90), obstacle_list)
                    follow_path(robot_pose, latest_path)

        # 경로 시각화
        if latest_path and M_real2pixel is not None:
            for i in range(1, len(latest_path)):
                pt1 = cv2.perspectiveTransform(np.array([[[latest_path[i-1][0], latest_path[i-1][1]]]], dtype=np.float32), M_real2pixel)[0][0]
                pt2 = cv2.perspectiveTransform(np.array([[[latest_path[i][0], latest_path[i][1]]]], dtype=np.float32), M_real2pixel)[0][0]
                cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (255, 0, 255), 2)
            start_pt = cv2.perspectiveTransform(np.array([[[latest_path[0][0], latest_path[0][1]]]], dtype=np.float32), M_real2pixel)[0][0]
            end_pt = cv2.perspectiveTransform(np.array([[[latest_path[-1][0], latest_path[-1][1]]]], dtype=np.float32), M_real2pixel)[0][0]
            cv2.circle(frame, tuple(start_pt.astype(int)), 6, (0, 255, 0), -1)
            cv2.putText(frame, "Start", (int(start_pt[0]+5), int(start_pt[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.circle(frame, tuple(end_pt.astype(int)), 6, (0, 0, 255), -1)
            cv2.putText(frame, "Goal", (int(end_pt[0]+5), int(end_pt[1]-5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # 장애물 시각화
        if M_real2pixel is not None:
            for (ox, oy, r) in obstacle_list:
                center_px = cv2.perspectiveTransform(np.array([[[ox, oy]]], dtype=np.float32), M_real2pixel)[0][0].astype(int)
                cv2.circle(frame, tuple(center_px), int(r), (0, 255, 255), 2)
                cv2.putText(frame, "Obstacle", (center_px[0]+5, center_px[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)

        # 안내 텍스트
        cv2.putText(frame, f"Points: {len(clicked_points)}/4", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if not transform_calculated:
            cv2.putText(frame, "Click 4 corners to define map area", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        elif not obstacle_input_done:
            cv2.putText(frame, "Click to add obstacles (Press 'd' when done)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        elif not goal_input_done:
            cv2.putText(frame, "Click to add goal points (Press 'g' when done)", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Robot View", frame)

        # 🖼 Raspi View 표시
        with raspi_client.queue_lock:
            raspi_frame = raspi_client.latest_frame.copy() if raspi_client.latest_frame is not None else None
        if raspi_frame is not None:
            # YOLO 추론
            results = model.predict(source=raspi_frame, imgsz=640, conf=0.5, verbose=False)
            annotated_raspi = raspi_frame.copy()

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0])
                    conf = float(box.conf[0])
                    cls_id = int(box.cls[0])
                    class_name = model.names[cls_id]  # ex: 'fire', 'fall'

                    cv2.rectangle(annotated_raspi, (x1, y1), (x2, y2), (0, 255, 0), 2)
                    cv2.putText(annotated_raspi, f"{class_name} {conf:.2f}", (x1, y1 - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            cv2.imshow("Raspi View", annotated_raspi)

    cv2.destroyAllWindows()

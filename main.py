import cv2
import numpy as np
import time

from perception.perception import detect_robot_position
from planning.planner import a_star
from network.client import RaspiVisionClient
from perception.CamCalibration import undistort_frame, get_new_K
from control.controller import follow_path

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
path_planned = False

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
    client = RaspiVisionClient("192.168.201.193")
    client.start()
    while client.latest_frame is None:
        time.sleep(0.1)

    new_K = get_new_K(client.latest_frame)
    cv2.namedWindow("Robot Position Detection")
    cv2.setMouseCallback("Robot Position Detection", mouse_callback)

    while True:
        with client.queue_lock:
            frame = client.latest_frame.copy() if client.latest_frame is not None else None
        if frame is None:
            continue

        frame = undistort_frame(frame, new_K)

        # 클릭 포인트 표시
        for idx, pt in enumerate(clicked_points):
            cv2.circle(frame, tuple(pt), 8, (0, 0, 255), -1)
        if len(clicked_points) == 4:
            pts = np.array(clicked_points, np.int32).reshape((-1, 1, 2))
            cv2.polylines(frame, [pts], True, (0, 255, 0), 2)

        # 장애물 좌표 변환
        obstacle_list = []
        if M_pixel2real is not None:
            for pt in obstacle_clicks:
                px = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
                real = cv2.perspectiveTransform(px, M_pixel2real)[0][0]
                obstacle_list.append((real[0], real[1], 4))

        # 목표점 좌표 변환
        goal_list_cm = []
        if M_pixel2real is not None:
            for pt in goal_clicks:
                px = np.array([[[pt[0], pt[1]]]], dtype=np.float32)
                real = cv2.perspectiveTransform(px, M_pixel2real)[0][0]
                goal_list_cm.append((real[0], real[1]))

        # 로봇 위치 검출 및 경로 생성 (입력 완료 후 한 번만)
        if transform_calculated and obstacle_input_done and goal_input_done and M_pixel2real is not None and not path_planned:
            robot_pose, (cx, cy) = detect_robot_position(frame, M_pixel2real)
            if robot_pose is not None:
                robot_position = robot_pose
                rx, ry, yaw = robot_pose
                cv2.putText(frame, f"X:{rx:.1f} Y:{ry:.1f} Yaw:{yaw:.1f}", (cx+10, cy), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,0), 2)

                # 목표점 순차 방문 경로 생성 (paste-2.txt 방식)
                curr_pos = (rx, ry)
                segment_path = []
                for goal in goal_list_cm:
                    path = a_star(curr_pos, goal, (0, 180, 0, 90), obstacle_list)
                    if not path:
                        print(f"경로 없음: {curr_pos} → {goal}")
                        continue
                    if segment_path and path[0] == segment_path[-1]:
                        segment_path += path[1:]
                    else:
                        segment_path += path
                    curr_pos = goal
                latest_path = segment_path
                path_planned = True

        # 경로 시각화
        if latest_path and M_real2pixel is not None:

            # control test
            left, right = follow_path(robot_pose, latest_path)
            msg = f'{left}, {right}'
            client.send_message(topic='control', message=msg)
            # control test

            for i in range(1, len(latest_path)):
                pt1 = cv2.perspectiveTransform(
                    np.array([[[latest_path[i-1][0], latest_path[i-1][1]]]], dtype=np.float32),
                    M_real2pixel
                )[0][0]
                pt2 = cv2.perspectiveTransform(
                    np.array([[[latest_path[i][0], latest_path[i][1]]]], dtype=np.float32),
                    M_real2pixel
                )[0][0]
                cv2.line(frame, tuple(pt1.astype(int)), tuple(pt2.astype(int)), (255, 0, 255), 2)
            if latest_path:
                start_pt = cv2.perspectiveTransform(
                    np.array([[[latest_path[0][0], latest_path[0][1]]]], dtype=np.float32),
                    M_real2pixel
                )[0][0]
                end_pt = cv2.perspectiveTransform(
                    np.array([[[latest_path[-1][0], latest_path[-1][1]]]], dtype=np.float32),
                    M_real2pixel
                )[0][0]
                cv2.circle(frame, tuple(start_pt.astype(int)), 6, (0, 255, 0), -1)
                cv2.putText(frame, "Start", (int(start_pt[0]+5), int(start_pt[1]-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.circle(frame, tuple(end_pt.astype(int)), 6, (0, 0, 255), -1)
                cv2.putText(frame, "End Point", (int(end_pt[0]+5), int(end_pt[1]-5)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)

        # GUI 요소 표시
        if M_real2pixel is not None:
            for (ox, oy, r) in obstacle_list:
                center = np.array([[[ox, oy]]], dtype=np.float32)
                center_px = cv2.perspectiveTransform(center, M_real2pixel)[0][0].astype(int)
                cv2.circle(frame, tuple(center_px), int(r), (0, 255, 255), 2)
                cv2.putText(frame, "Obstacle", (center_px[0]+5, center_px[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,255), 1)
            for idx, (gx, gy) in enumerate(goal_list_cm):
                center = np.array([[[gx, gy]]], dtype=np.float32)
                center_px = cv2.perspectiveTransform(center, M_real2pixel)[0][0].astype(int)
                cv2.circle(frame, tuple(center_px), 6, (0, 128, 255), -1)
                cv2.putText(frame, f"Goal {idx+1}", (center_px[0]+5, center_px[1]),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,128,255), 1)

        cv2.putText(frame, f"Points: {len(clicked_points)}/4", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        if not transform_calculated:
            cv2.putText(frame, "Click 4 corners to define map area", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 255), 2)
        elif not obstacle_input_done:
            cv2.putText(frame, "Click to add obstacles (Press 'd' when done)", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 200, 255), 2)
        elif not goal_input_done:
            cv2.putText(frame, "Click to add goal points (Press 'g' when done)", (10, 70),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        cv2.imshow("Robot Position Detection", frame)
        key = cv2.waitKey(1)
        if key == ord('d'):
            obstacle_input_done = True
        elif key == ord('g'):
            goal_input_done = True
        elif key == 27:
            break
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

    cv2.destroyAllWindows()
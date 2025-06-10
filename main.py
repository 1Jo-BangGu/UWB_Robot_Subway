import cv2
import numpy as np
import time
from perception.perception import Perception
from planning.planning import Planning
from control.controller import follow_path
from network.client import RaspiVisionClient

# 초기화
world_points = np.array([[0, 0], [0, 90], [180, 90], [180, 0]], dtype=np.float32)
perception = Perception(world_points)
planning = Planning()
robot_position = None
latest_path = []
stop_signal = False

# 마우스 콜백 함수
def mouse_callback(event, x, y, flags, param):
    perception.handle_mouse_event(event, x, y)

if __name__ == "__main__":
    client = RaspiVisionClient("192.168.200.221")
    client.start()
    while client.latest_frame is None:
        time.sleep(0.1)

    perception.init_new_K(client.latest_frame)

    cv2.namedWindow("Robot View")
    cv2.setMouseCallback("Robot View", mouse_callback)

    while True:
        with client.queue_lock:
            frame = client.latest_frame.copy() if client.latest_frame is not None else None
        if frame is None:
            continue

        # Perception 처리
        robot_pose, obstacle_list, goal_list_cm, frame, robot_center = perception.perception_def(frame)

        # 키 입력 처리
        key = cv2.waitKey(1)
        if key == ord('d'):
            perception.obstacle_input_done = True
        elif key == ord('g'):
            perception.goal_input_done = True
        elif key == ord('r'):
            perception.reset()
            robot_position = None
            latest_path = []
            stop_signal = False
        elif key == 27:  # ESC 종료
            break

        # 경로 계획 수행
        if robot_pose is not None and perception.goal_input_done:
            robot_position = robot_pose
            map_size = (180, 90)
            patrol_mode = False

            latest_path, stop_signal = planning.plan(
                robot_pose=robot_position,
                obstacle_list=obstacle_list,
                map_size=map_size,
                goals=goal_list_cm,
                patrol_signal=patrol_mode
            )

            if stop_signal:
                print("🟥 정지 조건 발생 또는 목표 도착")

            # 제어 수행 (필요 시 주석 해제)
            # if latest_path:
            #     left, right = follow_path(robot_position, latest_path)
            #     client.send_message(topic='control', message=f'{left}, {right}')

        # ✅ 시각화 함수 (경로 및 상태 모두 포함)
        perception.draw_visuals(
            frame=frame,
            robot_pose=robot_position,
            path=latest_path,
            robot_center=robot_center
        )

        cv2.imshow("Robot View", frame)

    cv2.destroyAllWindows()

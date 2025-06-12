# # main.py

# import cv2
# import numpy as np
# import time

# from perception.perception import Perception
# from planning.planning import Planning
# from control.controller import Control
# from server.client import Client
# from socketIO.communicate import ClientSocketIO

# # 📌 초기 설정
# world_points = np.array([[0, 0], [0, 90], [180, 90], [180, 0]], dtype=np.float32)
# perception = Perception(world_points)
# planning = Planning()
# robot_position = None
# latest_path = []
# stop_signal = False

# initial_robot_pose = None

# # 🖱️ 마우스 콜백 등록
# def mouse_callback(event, x, y, flags, param):
#     perception.handle_mouse_event(event, x, y)

# if __name__ == "__main__":
#     # 🔌 라즈베리 카메라 클라이언트 연결
#     client = Client("192.168.200.221")
#     client.start()

#     client_socketio = ClientSocketIO('192.168.200.221', 6001)
#     client_socketio.start()

#     while client.latest_frame is None:
#         time.sleep(0.01)

#     # 🧠 Control 클래스에 client 주입
#     control = Control(client)

#     # 🔧 카메라 보정 초기화
#     perception.init_new_K(client.latest_frame)

#     # 🎥 창 설정 및 마우스 콜백 지정
#     cv2.namedWindow("Robot View")
#     cv2.setMouseCallback("Robot View", mouse_callback)
    

#     while True:
#         # 🎞️ 최신 프레임 가져오기
#         with client.queue_lock:
#             frame = client.latest_frame.copy() if client.latest_frame is not None else None
#         if frame is None:
#             continue

#         # ======================== 👁️ Perception 단계 ========================
#         robot_pose, obstacle_list, goal_list_cm, frame, robot_center = perception.perception_def(frame)

#         if initial_robot_pose is None and robot_pose is not None:
#             initial_robot_pose = robot_pose

#         # ======================== ⌨️ 키보드 입력 처리 ========================
#         key = cv2.waitKey(1)
#         if key == ord('d'):
#             perception.obstacle_input_done = True
#         elif key == ord('g'):
#             perception.goal_input_done = True
#         elif key == ord('r'):
#             perception.reset()
#             robot_position = None
#             latest_path = []
#             stop_signal = False
#         elif key == 27:  # ESC
#             break

#         # ======================== 📍 Planning 단계 ========================
#         if robot_pose is not None and perception.goal_input_done:
#             robot_position = robot_pose
#             map_size = (180, 90)

#             latest_path, stop_signal, arrive_flag = planning.plan(
#                 initial_robot_pose,
#                 robot_pose=robot_position,
#                 obstacle_list=obstacle_list,
#                 map_size=map_size,
#                 goals=goal_list_cm,
#                 return_flag=client_socketio.return_flag,
#                 fire_signal=client_socketio.fire,
#                 fall_signal=client_socketio.fall
#             )

#             # ======================== 🎮 Control 단계 ========================
#             control.update_and_send(robot_position, latest_path, stop_signal, arrive_flag)

#         # ======================== 🖼️ Visualization 단계 ========================
#         perception.draw_visuals(
#             frame=frame,
#             robot_pose=robot_position,
#             path=latest_path,
#             robot_center=robot_center
#         )

#         cv2.imshow("Robot View", frame)
#         # 🌐 시각화된 프레임 서버로 전송
#         client.upload_frame(frame)

#     cv2.destroyAllWindows()


# main.py (수정 후) - 최상단에 추가
# 🔥 윈도우 이벤트 루프 정책 설정 (추가)
import platform
import asyncio
if platform.system() == 'Windows':
    asyncio.set_event_loop_policy(asyncio.WindowsSelectorEventLoopPolicy())

import cv2
import numpy as np
import time

from perception.perception import Perception
from planning.planning import Planning
from control.controller import Control
from server.client import Client
from socketIO.communicate import ClientSocketIO

# 📌 초기 설정
world_points = np.array([[0, 0], [0, 90], [180, 90], [180, 0]], dtype=np.float32)
perception = Perception(world_points)
planning = Planning()
robot_position = None
latest_path = []
stop_signal = False

initial_robot_pose = None

# 🖱️ 마우스 콜백 등록
def mouse_callback(event, x, y, flags, param):
    perception.handle_mouse_event(event, x, y)

if __name__ == "__main__":
    # 🔌 라즈베리 카메라 클라이언트 연결
    client = Client("192.168.200.221")
    client.start()

    client_socketio = ClientSocketIO('192.168.200.221', 6001)
    client_socketio.start()

    while client.latest_frame is None:
        time.sleep(0.01)

    # 🧠 Control 클래스에 client 주입
    control = Control(client)

    # 🔧 카메라 보정 초기화
    perception.init_new_K(client.latest_frame)

    # 🎥 창 설정 및 마우스 콜백 지정
    cv2.namedWindow("Robot View")
    cv2.setMouseCallback("Robot View", mouse_callback)
    

    while True:
        # 🎞️ 최신 프레임 가져오기
        with client.queue_lock:
            frame = client.latest_frame.copy() if client.latest_frame is not None else None
        if frame is None:
            continue

        # ======================== 👁️ Perception 단계 ========================
        robot_pose, obstacle_list, goal_list_cm, frame, robot_center = perception.perception_def(frame)

        if initial_robot_pose is None and robot_pose is not None:
            initial_robot_pose = robot_pose

        # ======================== ⌨️ 키보드 입력 처리 ========================
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
        elif key == 27:  # ESC
            break

        # ======================== 📍 Planning 단계 ========================
        if robot_pose is not None and perception.goal_input_done:
            robot_position = robot_pose
            map_size = (180, 90)

            latest_path, stop_signal, arrive_flag, *remain_= planning.plan(
                initial_robot_pose,
                robot_pose=robot_position,
                obstacle_list=obstacle_list,
                map_size=map_size,
                goals=goal_list_cm,
                return_flag=client_socketio.return_flag,
                fire_signal=client_socketio.fire,
                fall_signal=client_socketio.fall
            )
            if len(remain_) == 3:
                client_socketio.fire = remain_[0]
                client_socketio.fall = remain_[1]
                client_socketio.return_flag = remain_[2]

            # ======================== 🎮 Control 단계 ========================
            control.update_and_send(robot_position, latest_path, stop_signal, arrive_flag)

        # ======================== 🖼️ Visualization 단계 ========================
        perception.draw_visuals(
            frame=frame,
            robot_pose=robot_position,
            path=latest_path,
            robot_center=robot_center
        )

        cv2.imshow("Robot View", frame)
        # 🌐 시각화된 프레임 서버로 전송
        client.upload_frame(frame)
        # time.sleep(0.01)

    cv2.destroyAllWindows()

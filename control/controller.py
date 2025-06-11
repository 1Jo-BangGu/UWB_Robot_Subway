# control/controller.py

import math

class Control:
    def __init__(self, client, wheel_base=5.0, base_speed=30, lookahead_dist=15.0):
        self.client = client
        self.WHEEL_BASE = wheel_base
        self.BASE_SPEED = base_speed
        self.LOOKAHEAD_DIST = lookahead_dist

    def follow_path(self, robot_pose, path):
        """
        입력:
            robot_pose: (x, y, yaw in degrees)
            path: list of (x, y)

        출력:
            (PWM_left, PWM_right)
        """
        rx, ry, yaw_deg = robot_pose
        yaw_rad = math.radians(yaw_deg)

        # 🔹 1. 가장 가까운 포인트 인덱스 찾기
        min_dist = float('inf')
        closest_index = 0
        for i, pt in enumerate(path):
            dist = math.hypot(pt[0] - rx, pt[1] - ry)
            if dist < min_dist:
                min_dist = dist
                closest_index = i
        # print('closest waypoint idx : ',closest_index)
        # 🔹 2. 가까운 포인트부터 Lookahead 포인트 탐색
        goal = path[-1]
        for pt in path[closest_index:]:
            dist = math.hypot(pt[0] - rx, pt[1] - ry)
            if dist >= self.LOOKAHEAD_DIST:
                goal = pt
                break
        gx, gy = goal

        # 로봇 기준 좌표계로 변환
        dx = gx - rx
        dy = gy - ry
        alpha = math.atan2(dy, dx) - yaw_rad
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi  # -pi ~ pi 정규화

        # 곡률 및 회전 계산
        curvature = (2 * math.sin(alpha)) / self.LOOKAHEAD_DIST
        omega = self.BASE_SPEED * curvature
        right_speed = self.BASE_SPEED + omega * (self.WHEEL_BASE / 2)
        left_speed = self.BASE_SPEED - omega * (self.WHEEL_BASE / 2)

        # PWM 클리핑
        right_speed = max(20, min(40, right_speed))
        left_speed = max(20, min(40, left_speed))

        return int(left_speed), int(right_speed)

    def update_and_send(self, robot_pose, path, stop_signal):
        if stop_signal:
            print("🟥 정지 조건 발생 또는 목표 도착")
            return

        if not path:
            print("[Control] 경로 없음 → 제어 생략")
            return

        left, right = self.follow_path(robot_pose, path)
        print(f"[Control] PWM 계산 완료 → Left: {left}, Right: {right}")
        msg = f"{left}, {right}"

        try:
            self.client.send_message(topic='control', message=msg)
            print(f"[Control] 제어 명령 전송 완료 → {msg}")
        except Exception as e:
            print(f"[Control Error] 전송 실패: {e}")

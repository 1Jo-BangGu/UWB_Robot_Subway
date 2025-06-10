# control/controller.py

import math

class Control:
    def __init__(self, client, wheel_base=10.0, base_speed=30, lookahead_dist=20.0):
        self.client = client
        self.WHEEL_BASE = wheel_base
        self.BASE_SPEED = base_speed
        self.LOOKAHEAD_DIST = lookahead_dist

    def follow_path(self, robot_pose, path):
        rx, ry, yaw_deg = robot_pose
        yaw_rad = math.radians(yaw_deg)

        # Lookahead 지점 찾기
        goal = path[-1]
        for pt in path:
            dist = math.hypot(pt[0] - rx, pt[1] - ry)
            if dist >= self.LOOKAHEAD_DIST:
                goal = pt
                break
        gx, gy = goal

        # 로봇 기준 좌표계 변환
        dx = gx - rx
        dy = gy - ry
        alpha = math.atan2(dy, dx) - yaw_rad
        alpha = (alpha + math.pi) % (2 * math.pi) - math.pi

        # 곡률 계산
        curvature = (2 * math.sin(alpha)) / self.LOOKAHEAD_DIST
        omega = self.BASE_SPEED * curvature
        right_speed = self.BASE_SPEED + omega * (self.WHEEL_BASE / 2)
        left_speed = self.BASE_SPEED - omega * (self.WHEEL_BASE / 2)

        # PWM 클리핑
        right_speed = max(20, min(40, right_speed))
        left_speed = max(20, min(40, left_speed))

        return int(left_speed), int(right_speed)

    def update_and_send(self, robot_pose, path):
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

# 로봇 위치와 경로를 받아 모터 제어를 수행하는 공간입니다.
# 실제 제어 알고리즘 및 PWM 출력 코드는 하드웨어에 맞게 구현 필요.

# def follow_path(robot_pose, path):
#     """
#     입력:
#         robot_pose: (x, y, yaw)
#         path: [(x1, y1), (x2, y2), ...]

#     출력:
#         PWM 신호 또는 모터 명령 (현재는 placeholder)
#     """
#     # TODO: 경로를 따라가는 제어 알고리즘 구현
#     # 예: Pure Pursuit, PID, 등등
#     # 예: PWM 출력: pwm_left, pwm_right = compute_pwm(...)
#     pass

import math

# ➤ 로봇 설정
WHEEL_BASE = 10.0       # 바퀴 간 거리 (cm)
BASE_SPEED = 30         # 기준 속도 (PWM), 모터 범위 20~40
LOOKAHEAD_DIST = 20.0   # cm


def follow_path(robot_pose, path):
    """
    입력:
        robot_pose: (x, y, yaw in degrees)
        path: list of (x, y)

    출력:
        (PWM_left, PWM_right)
    """
    rx, ry, yaw_deg = robot_pose
    yaw_rad = math.radians(yaw_deg)

    # Lookahead 지점 찾기
    goal = path[-1]
    for pt in path:
        dist = math.hypot(pt[0] - rx, pt[1] - ry)
        if dist >= LOOKAHEAD_DIST:
            goal = pt
            break
    gx, gy = goal

    # 로봇 기준 좌표계로 변환
    dx = gx - rx
    dy = gy - ry
    alpha = math.atan2(dy, dx) - yaw_rad
    alpha = (alpha + math.pi) % (2 * math.pi) - math.pi  # -pi ~ pi 정규화

    # 곡률 및 회전 계산
    curvature = (2 * math.sin(alpha)) / LOOKAHEAD_DIST
    omega = BASE_SPEED * curvature
    right_speed = BASE_SPEED + omega * (WHEEL_BASE / 2)
    left_speed = BASE_SPEED - omega * (WHEEL_BASE / 2)

    # PWM 클리핑
    right_speed = max(20, min(40, right_speed))
    left_speed = max(20, min(40, left_speed))

    # print(f"[Follow Path] Goal: {goal} | α: {round(math.degrees(alpha), 2)}° | PWM: L={int(left_speed)} R={int(right_speed)}")
    return int(left_speed), int(right_speed)

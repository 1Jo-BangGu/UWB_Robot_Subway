# 로봇 위치와 경로를 받아 모터 제어를 수행하는 공간입니다.
# 실제 제어 알고리즘 및 PWM 출력 코드는 하드웨어에 맞게 구현 필요.

def follow_path(robot_pose, path):
    """
    입력:
        robot_pose: (x, y, yaw)
        path: [(x1, y1), (x2, y2), ...]

    출력:
        PWM 신호 또는 모터 명령 (현재는 placeholder)
    """
    # TODO: 경로를 따라가는 제어 알고리즘 구현
    # 예: Pure Pursuit, PID, 등등
    # 예: PWM 출력: pwm_left, pwm_right = compute_pwm(...)
    pass
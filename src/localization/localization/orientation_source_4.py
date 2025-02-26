import math


def update_orientation(theta, v, steer, L, dt):
    """
    현재 orientation(theta)와 주어진 속도(v), 조향각(steer), 휠베이스(L), 시간 간격(dt)를 이용해 새로운 orientation을 계산합니다.
    theta: 현재 orientation (라디안)
    v: 전진 속도 (m/s)
    steer: 조향각 (라디안)
    L: 휠베이스 (m)
    dt: 시간 간격 (s)
    """
    dtheta = (v / L) * math.tan(steer) * dt
    theta_new = theta + dtheta

    # orientation을 -pi ~ pi 범위로 정규화 (선택 사항)
    theta_new = math.atan2(math.sin(theta_new), math.cos(theta_new))
    return theta_new


# 예시 사용
current_theta = math.radians(10)  # 초기 10도
v = 5.0  # 5 m/s
steer = math.radians(5)  # 5도 조향각
L = 2.5  # 휠베이스 2.5m
dt = 0.1  # 0.1초 시간 간격

new_theta = update_orientation(current_theta, v, steer, L, dt)
print("New orientation (degrees):", math.degrees(new_theta))

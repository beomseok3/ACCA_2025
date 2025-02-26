import math
import time


# heading 보정- 상보필터 --> localization - EKF 좋은데
class ComplementaryFilter:
    def __init__(self, alpha=0.98):
        self.alpha = alpha
        self.filtered_heading = 0.0  # 초기 heading (라디안)
        self.last_time = time.time()

    def update(self, gyro_rate, mag_heading):
        """
        자이로 각속도와 마그네토미터 heading을 받아 상보 필터를 업데이트합니다.

        Parameters:
            gyro_rate : float
                자이로 각속도 (rad/s)
            mag_heading : float
                마그네토미터를 통해 계산한 절대 heading (rad)

        Returns:
            float: 필터링된 heading (rad)
        """
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        # 자이로를 적분하여 단기 heading 추정
        gyro_heading = self.filtered_heading + gyro_rate * dt

        # 상보 필터: 고주파는 자이로, 저주파는 마그네토미터
        self.filtered_heading = (
            self.alpha * gyro_heading + (1 - self.alpha) * mag_heading
        )

        # heading을 -pi ~ pi 범위로 정규화
        self.filtered_heading = math.atan2(
            math.sin(self.filtered_heading), math.cos(self.filtered_heading)
        )

        return self.filtered_heading


# 사용 예시
if __name__ == "__main__":
    cf = ComplementaryFilter(alpha=0.98)

    # 예시: 센서로부터 데이터를 받는다고 가정
    # 이 값들은 실제 센서 콜백이나 루프에서 갱신되어야 함
    gyro_rate_example = 0.05  # rad/s (자이로 각속도)
    mag_heading_example = math.radians(
        30
    )  # 마그네토미터에서 계산된 heading, 30도 (rad)

    # 반복적으로 업데이트
    for i in range(100):
        filtered = cf.update(gyro_rate_example, mag_heading_example)
        print(f"Filtered Heading: {math.degrees(filtered):.2f} degrees")
        time.sleep(0.1)

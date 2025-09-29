#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <nav_msgs/msg/odometry.hpp>  // 추가


#include <deque>
#include <cmath>

class Rotate : public rclcpp::Node
{
public:
  Rotate()
  : Node("rotate_yaw"),
    delta_yaw_(this->declare_parameter<double>("delta_yaw", 0.0))
  {
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
      // History Depth 10 (추정)
    qos.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
    qos.durability(RMW_QOS_POLICY_DURABILITY_VOLATILE);

    sub_imu_ = create_subscription<sensor_msgs::msg::Imu>(
      "imu/data", qos,
      std::bind(&Rotate::imuCallback, this, std::placeholders::_1));

    sub_gps_vel_ = create_subscription<geometry_msgs::msg::TwistWithCovarianceStamped>(
      "ublox_gps_node/fix_velocity", qos,
      std::bind(&Rotate::gpsVelCallback, this, std::placeholders::_1));

    sub_gps_fix_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", qos,
      std::bind(&Rotate::gpsFixCallback, this, std::placeholders::_1));
    // ★ 추가: GPS 재밍 오도메트리 구독
    sub_gps_jam_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/gps_jamming", qos,
      std::bind(&Rotate::gpsJammingCallback, this, std::placeholders::_1));


    pub_imu_rotated_ = create_publisher<sensor_msgs::msg::Imu>("imu/rotated", qos);
    pub_mean_quat_  = create_publisher<geometry_msgs::msg::Quaternion>("mean", qos);
  }

private:
  /* ---------- 보조 함수 ---------- */
// 수치 안정성을 위한 clamp
static inline double clamp(double v, double lo, double hi) {
  return std::max(lo, std::min(v, hi));
}

// geometry_msgs::msg::Quaternion -> (roll,pitch,yaw)
// 회전 순서는 roll(X) -> pitch(Y) -> yaw(Z) 가정 (ROS tf와 동일 관습)
  static inline void eulerFromQuat(const geometry_msgs::msg::Quaternion& q,
                                  double& roll, double& pitch, double& yaw)
  {
    const double x = q.x;
    const double y = q.y;
    const double z = q.z;
    const double w = q.w;

    // Tait-Bryan XYZ (roll-pitch-yaw) 표준 공식
    const double sinr_cosp = 2.0 * (w * x + y * z);
    const double cosr_cosp = 1.0 - 2.0 * (x * x + y * y);
    roll = std::atan2(sinr_cosp, cosr_cosp);

    double sinp = 2.0 * (w * y - z * x);
    sinp = clamp(sinp, -1.0, 1.0);
    pitch = std::asin(sinp);

    const double siny_cosp = 2.0 * (w * z + x * y);
    const double cosy_cosp = 1.0 - 2.0 * (y * y + z * z);
    yaw = std::atan2(siny_cosp, cosy_cosp);
  }

  // (roll,pitch,yaw) -> geometry_msgs::msg::Quaternion
  static inline geometry_msgs::msg::Quaternion quatFromEuler(double roll, double pitch, double yaw)
  {
    const double cr = std::cos(roll * 0.5);
    const double sr = std::sin(roll * 0.5);
    const double cp = std::cos(pitch * 0.5);
    const double sp = std::sin(pitch * 0.5);
    const double cy = std::cos(yaw * 0.5);
    const double sy = std::sin(yaw * 0.5);

    geometry_msgs::msg::Quaternion q;
    q.w = cr * cp * cy + sr * sp * sy;
    q.x = sr * cp * cy - cr * sp * sy;
    q.y = cr * sp * cy + sr * cp * sy;
    q.z = cr * cp * sy - sr * sp * cy;
    return q;
  }


  bool decisionStraight()
  {
    if (forward_.size() < 10) return false;

    double sum = 0.0;
    for (double y : forward_) sum += y;
    double mean = sum / forward_.size();

    for (double y : forward_)
      if (std::abs((mean - y) * 180.0 / M_PI) > 1.0)  // 2 deg 허용
        return false;

    mean_ = mean;
    return true;
  }

  /* ---------- 콜백 ---------- */
  void gpsFixCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    cov_ = msg->position_covariance[0];
  }
  void gpsJammingCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // cov[0] 사용 가정: pose.covariance[0]
    jamming_cov0_ = msg->pose.covariance[0];
  }

  void gpsVelCallback(const geometry_msgs::msg::TwistWithCovarianceStamped::SharedPtr msg)
  {
    double vx = msg->twist.twist.linear.x;
    double vy = msg->twist.twist.linear.y;
    v_ = std::hypot(vx, vy);               // 속도 크기
    gps_yaw_ = std::atan2(vy, vx);         // GPS 방향

    forward_.push_back(gps_yaw_);
    if (forward_.size() > 10) forward_.pop_front();
  }

  void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg)
  {
    /* ① IMU → (roll,pitch,yaw) */
    double roll, pitch, yaw;
    eulerFromQuat(msg->orientation, roll, pitch, yaw);

    /* ② GPS‑IMU 차이 */
    double delta   = gps_yaw_ - yaw;
    double abs_deg = std::abs(delta) * 180.0 / M_PI;

    if (abs_deg > 2.0 && abs_deg < 90.0            // 2° < |Δ| < 90°
        && v_ > 0.50                               // 최소 속도
        && cov_ < 0.0004
        && jamming_cov0_ < 0.0004
      )                          // GPS 분산
    {
      if (decisionStraight()) {
        delta_ = mean_ - yaw;

        // GPS 평균 방향을 별도 topic으로 publish
        geometry_msgs::msg::Quaternion q_msg = quatFromEuler(0,0,gps_yaw_);
        q_msg.x = q_gps.x(); q_msg.y = q_gps.y();
        q_msg.z = q_gps.z(); q_msg.w = q_gps.w();
        pub_mean_quat_->publish(q_msg);
      }
    }

    /* ③ 보정된 yaw 적용 */
    double yaw_prev = yaw;
    yaw = yaw + delta_yaw_ + delta_;

    geometry_msgs::msg::Quaternion q_new = quatFromEuler(0,0,yaw);

    sensor_msgs::msg::Imu imu_out = *msg;   // 원본 복사
    imu_out.orientation = q_new;
    imu_out.header.stamp = this->get_clock()->now();

    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 100,
        "Yaw prev: %.2f°, new: %.2f°, delta: %.2f°",
        yaw_prev * 180.0 / M_PI,
        yaw      * 180.0 / M_PI,
        delta_   * 180.0 / M_PI);

    pub_imu_rotated_->publish(imu_out);
  }

  /* ---------- 멤버 변수 ---------- */
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr   sub_imu_;
  rclcpp::Subscription<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr sub_gps_vel_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr sub_gps_fix_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_gps_jam_; // 추가


  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu_rotated_;
  rclcpp::Publisher<geometry_msgs::msg::Quaternion>::SharedPtr pub_mean_quat_;

  /* 상태 저장 */
  std::deque<double> forward_;   // 최근 10 개의 GPS yaw
  double mean_{0.0};
  double delta_{0.0};

  double delta_yaw_;  // 파라미터
  double gps_yaw_{0.0};
  double v_{0.0};
  double cov_{0.0};
  double jamming_cov0_{0.0};
};

/* ---------- main ---------- */
int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Rotate>());
  rclcpp::shutdown();
  return 0;
}

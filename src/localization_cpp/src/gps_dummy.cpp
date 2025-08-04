// gps_jamming_filter.cpp
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

class GpsJammingFilter : public rclcpp::Node
{
public:
  GpsJammingFilter()
  : Node("gps_jamming_filter")
  {
    // ─────────────── Parameter ───────────────
    this->declare_parameter<bool>("gps_jamming_mode", false);

    // ─────────────── QoS ───────────────
    auto qos = rclcpp::SystemDefaultsQoS().keep_last(10);

    // ─────────────── Pub/Sub ───────────────
    sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/odometry/gps", qos,
      std::bind(&GpsJammingFilter::odomCallback, this, std::placeholders::_1));

    pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
      "/odometry/gps_jamming", qos);

    // ─────────────── Dynamic parameter callback ───────────────
    param_cb_handle_ = this->add_on_set_parameters_callback(
      std::bind(&GpsJammingFilter::paramCallback, this, std::placeholders::_1));
  }

private:
  // ------- Parameter helper -------
  bool gpsJammingMode() const
  {
    return this->get_parameter("gps_jamming_mode").as_bool();
  }

  // ------- Parameter change callback -------
  rcl_interfaces::msg::SetParametersResult
  paramCallback(const std::vector<rclcpp::Parameter>&)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;                 // accept all changes
    result.reason = "Parameter accepted";
    return result;
  }

  // ------- Odometry subscription callback -------
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    nav_msgs::msg::Odometry out_msg = *msg;   // copy

    if (gpsJammingMode())
    {
      // pose.covariance는 6×6 행렬이 1‑D 배열(36)로 직렬화되어 있음
      for (size_t i = 0; i < 36; i += 7)      // 0,7,14,21,28,35: 대각 원소
      {
        out_msg.pose.covariance[i] = 9.99999999999e11;
      }
    }
    // 그대로 또는 수정된 메시지를 새 토픽으로 발행
    pub_->publish(out_msg);
  }

  // ------- Members -------
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pub_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_cb_handle_;
};

// ────────────────────────────────────────────
int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GpsJammingFilter>());
  rclcpp::shutdown();
  return 0;
}

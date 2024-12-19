#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "builtin_interfaces/msg/duration.hpp"

class Float32ToMarkerNode : public rclcpp::Node {
public:
  Float32ToMarkerNode()
  : Node("bs_cone_marker") {
    yellow_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "poses/yellow", 10,
      std::bind(&Float32ToMarkerNode::yellow_callback, this, std::placeholders::_1)
    );

    yellow_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "yellow_cone_marker", 10
    );

    blue_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
      "poses/blue", 10,
      std::bind(&Float32ToMarkerNode::blue_callback, this, std::placeholders::_1)
    );

    blue_marker_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "blue_cone_marker", 10
    );
  }

private:
  int yellow_past_id = 0;
  int blue_past_id = 0;

  void yellow_callback(const geometry_msgs::msg::PoseArray::SharedPtr yellow_msg) {
    auto yellow_marker_array = visualization_msgs::msg::MarkerArray();
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0.1;
    duration.nanosec = 0; // 0.01 seconds
    int yellow_id = 0;

    for (size_t i = 0; i < yellow_msg->poses.size(); ++i) {
      const auto &pose = yellow_msg->poses[i];

      auto yellow_marker = visualization_msgs::msg::Marker();
      yellow_marker.header.frame_id = "velodyne";
      yellow_marker.header.stamp = this->get_clock()->now();
      yellow_marker.id = yellow_id++;
      yellow_marker.type = visualization_msgs::msg::Marker::CUBE;  // Change as needed
      yellow_marker.action = visualization_msgs::msg::Marker::ADD;
      yellow_marker.pose.position.x = pose.position.x;
      yellow_marker.pose.position.y = pose.position.y;
      yellow_marker.pose.position.z = pose.position.z;
      yellow_marker.pose.orientation = pose.orientation;
      yellow_marker.scale.x = 0.2;
      yellow_marker.scale.y = 0.2;
      yellow_marker.scale.z = 0.2;
      yellow_marker.color.a = 1.0;  // Don't forget to set the alpha!
      yellow_marker.color.r = 1.0;
      yellow_marker.color.g = 1.0;
      yellow_marker.color.b = 0.0;
      yellow_marker.lifetime = duration;
      yellow_marker_array.markers.push_back(yellow_marker);
    }

    yellow_marker_publisher_->publish(yellow_marker_array);

    if (yellow_id < this->yellow_past_id) {
      auto delete_marker_array = visualization_msgs::msg::MarkerArray();
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = "velodyne";
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker_array.markers.push_back(delete_marker);
      yellow_marker_publisher_->publish(delete_marker_array);
    }
    this->yellow_past_id = yellow_id;
  }

  void blue_callback(const geometry_msgs::msg::PoseArray::SharedPtr blue_msg) {
    auto blue_marker_array = visualization_msgs::msg::MarkerArray();
    builtin_interfaces::msg::Duration duration;
    duration.sec = 0.1;
    duration.nanosec = 0; // 0.01 seconds
    int blue_id = 0;

    for (size_t i = 0; i < blue_msg->poses.size(); ++i) {
      const auto &pose = blue_msg->poses[i];

      auto blue_marker = visualization_msgs::msg::Marker();
      blue_marker.header.frame_id = "velodyne";
      blue_marker.header.stamp = this->get_clock()->now();
      blue_marker.id = blue_id++;
      blue_marker.type = visualization_msgs::msg::Marker::CUBE;  // Change as needed
      blue_marker.action = visualization_msgs::msg::Marker::ADD;
      blue_marker.pose.position.x = pose.position.x;
      blue_marker.pose.position.y = pose.position.y;
      blue_marker.pose.position.z = pose.position.z;
      blue_marker.pose.orientation = pose.orientation;
      blue_marker.scale.x = 0.2;
      blue_marker.scale.y = 0.2;
      blue_marker.scale.z = 0.2;
      blue_marker.color.a = 1.0;  // Don't forget to set the alpha!
      blue_marker.color.r = 0.0;
      blue_marker.color.g = 0.0;
      blue_marker.color.b = 1.0;
      blue_marker.lifetime = duration;
      blue_marker_array.markers.push_back(blue_marker);
    }

    blue_marker_publisher_->publish(blue_marker_array);

    if (blue_id < this->blue_past_id) {
      auto delete_marker_array = visualization_msgs::msg::MarkerArray();
      auto delete_marker = visualization_msgs::msg::Marker();
      delete_marker.header.frame_id = "velodyne";
      delete_marker.header.stamp = this->get_clock()->now();
      delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;
      delete_marker_array.markers.push_back(delete_marker);
      blue_marker_publisher_->publish(delete_marker_array);
    }
    this->blue_past_id = blue_id;
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr yellow_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr yellow_marker_publisher_;
  rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr blue_subscription_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr blue_marker_publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Float32ToMarkerNode>());
  rclcpp::shutdown();
  return 0;
}

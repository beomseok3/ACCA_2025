#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>

class PathGenerator : public rclcpp::Node {
public:
    PathGenerator() : Node("path_generator") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/clustered_line", 10, std::bind(&PathGenerator::lane_callback, this, std::placeholders::_1));

        marker_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/paht_line", 10);
    }

private:
    void lane_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::vector<std::array<float, 3>> lane_points;
        
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            lane_points.push_back({*iter_x, *iter_y, *iter_z});
        }

        if (lane_points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No lane points received.");
            return;
        }

        std::vector<std::array<float, 3>> path_points;
        float lane_offset = 2.0;  // 차선 간 거리 (기본값 2m)

        if (!lane_points.empty()) {
            for (const auto &pt : lane_points) {
                float x_new, y_new, z_new;

                if (pt[1] > 0) {  // 왼쪽 차선만 인식됨
                    x_new = pt[0] - lane_offset;
                    y_new = pt[1] - lane_offset;
                    z_new = pt[2];
                } else {  // 오른쪽 차선만 인식됨
                    x_new = pt[0] + lane_offset;
                    y_new = pt[1] + lane_offset;
                    z_new = pt[2];
                }

                path_points.push_back({x_new, y_new, z_new});
            }
        }

        publish_path(msg->header, path_points);
    }

    void publish_path(const std_msgs::msg::Header &header, const std::vector<std::array<float, 3>> &path_points) {
        visualization_msgs::msg::Marker path_marker;
        path_marker.header = header;
        path_marker.ns = "generated_path";
        path_marker.id = 1;
        path_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::msg::Marker::ADD;
        path_marker.scale.x = 0.1;
        path_marker.color.r = 0.0;
        path_marker.color.g = 1.0;
        path_marker.color.b = 1.0;
        path_marker.color.a = 1.0;
        path_marker.pose.orientation.w = 1.0;

        for (const auto &point : path_points) {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];

            path_marker.points.push_back(p);
        }

        marker_publisher_->publish(path_marker);
        RCLCPP_INFO(this->get_logger(), "Published path with %zu points", path_points.size());
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PathGenerator>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

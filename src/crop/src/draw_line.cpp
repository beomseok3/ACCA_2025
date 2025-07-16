#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

class LidarLineConnector : public rclcpp::Node {
public:
    LidarLineConnector() : Node("draw_line_filter") {
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/lane_filtered", 10, std::bind(&LidarLineConnector::lidar_callback, this, std::placeholders::_1));

        left_line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/left_line", 10);
        right_line_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/right_line", 10);
        points_publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/clustered_points", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        std::vector<std::array<float, 3>> points;

        // PointCloud2 데이터를 (x, y, z) 리스트로 변환
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
            points.push_back({*iter_x, *iter_y, *iter_z});
        }

        if (points.empty()) {
            RCLCPP_WARN(this->get_logger(), "No points received");
            return;
        }

        // 1️⃣ 포인트 클러스터링 후 평균 좌표 계산
        std::vector<std::array<float, 3>> clustered_points = cluster_and_average(points);

        // 디버깅: 클러스터링된 점 출력
        for (const auto &p : clustered_points) {
            RCLCPP_INFO(this->get_logger(), "Clustered Point: (%f, %f, %f)", p[0], p[1], p[2]);
        }

        // 2️⃣ 클러스터링된 점들 중 2.5m 이내에 있는 점들만 선으로 연결
        std::vector<std::pair<std::array<float, 3>, std::array<float, 3>>> line_segments = connect_close_points(clustered_points, 2.5);

        // 3️⃣ 차선 분리: 평균 y값을 기준으로 왼쪽/오른쪽 차선을 구분
        std::vector<std::array<float, 3>> left_line, right_line;
        classify_lines(line_segments, left_line, right_line);

        // 4️⃣ 클러스터링된 점 & 왼쪽/오른쪽 차선 시각화
        publish_clustered_points(msg->header, clustered_points);
        publish_line(msg->header, left_line, true);  // 왼쪽 차선
        publish_line(msg->header, right_line, false); // 오른쪽 차선
    }

    std::vector<std::array<float, 3>> cluster_and_average(const std::vector<std::array<float, 3>> &points) {
        const float cluster_distance = 0.5f;
        std::vector<std::array<float, 3>> clustered_points;
        std::vector<bool> visited(points.size(), false);

        for (size_t i = 0; i < points.size(); ++i) {
            if (visited[i]) continue;

            float sum_x = points[i][0], sum_y = points[i][1], sum_z = points[i][2];
            int count = 1;

            for (size_t j = i + 1; j < points.size(); ++j) {
                float dx = points[i][0] - points[j][0];
                float dy = points[i][1] - points[j][1];
                float dz = points[i][2] - points[j][2];
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance < cluster_distance) {
                    sum_x += points[j][0];
                    sum_y += points[j][1];
                    sum_z += points[j][2];
                    count++;
                    visited[j] = true;
                }
            }

            clustered_points.push_back({sum_x / count, sum_y / count, sum_z / count});
        }

        return clustered_points;
    }

    std::vector<std::pair<std::array<float, 3>, std::array<float, 3>>> connect_close_points(
        const std::vector<std::array<float, 3>> &clustered_points, float max_distance) {
        
        std::vector<std::pair<std::array<float, 3>, std::array<float, 3>>> line_segments;

        for (size_t i = 0; i < clustered_points.size(); ++i) {
            for (size_t j = i + 1; j < clustered_points.size(); ++j) {
                float dx = clustered_points[i][0] - clustered_points[j][0];
                float dy = clustered_points[i][1] - clustered_points[j][1];
                float dz = clustered_points[i][2] - clustered_points[j][2];
                float distance = std::sqrt(dx * dx + dy * dy + dz * dz);

                if (distance >= max_distance) {
                    continue;
                }

                line_segments.push_back({clustered_points[i], clustered_points[j]});
            }
        }

        return line_segments;
    }

    void classify_lines(
        const std::vector<std::pair<std::array<float, 3>, std::array<float, 3>>> &line_segments,
        std::vector<std::array<float, 3>> &left_line,
        std::vector<std::array<float, 3>> &right_line) {
        
        for (const auto &line : line_segments) {
            float avg_y = (line.first[1] + line.second[1]) / 2.0;
            if (avg_y > 0) {
                left_line.push_back(line.first);
                left_line.push_back(line.second);
            } else {
                right_line.push_back(line.first);
                right_line.push_back(line.second);
            }
        }
    }

    void publish_clustered_points(const std_msgs::msg::Header &header, const std::vector<std::array<float, 3>> &clustered_points) {
        visualization_msgs::msg::Marker point_marker;
        point_marker.header = header;
        point_marker.ns = "clustered_points";
        point_marker.id = 2;
        point_marker.type = visualization_msgs::msg::Marker::POINTS;
        point_marker.scale.x = 0.2;
        point_marker.scale.y = 0.2;
        point_marker.color.r = 1.0;
        point_marker.color.a = 1.0;

        for (const auto &point : clustered_points) {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            point_marker.points.push_back(p);
        }

        points_publisher_->publish(point_marker);
    }

    void publish_line(const std_msgs::msg::Header &header, const std::vector<std::array<float, 3>> &line_points, bool is_left) {
        if (line_points.empty()) return;

        visualization_msgs::msg::Marker line_marker;
        line_marker.header = header;
        line_marker.ns = is_left ? "left_lane" : "right_lane";
        line_marker.id = is_left ? 1 : 2;
        line_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        line_marker.scale.x = 0.1;
        line_marker.color.g = is_left ? 1.0 : 0.0;
        line_marker.color.r = is_left ? 0.0 : 1.0;
        line_marker.color.a = 1.0;

        for (const auto &point : line_points) {
            geometry_msgs::msg::Point p;
            p.x = point[0];
            p.y = point[1];
            p.z = point[2];
            line_marker.points.push_back(p);
        }

        if (is_left) left_line_publisher_->publish(line_marker);
        else right_line_publisher_->publish(line_marker);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr left_line_publisher_, right_line_publisher_, points_publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarLineConnector>());
    rclcpp::shutdown();
    return 0;
}

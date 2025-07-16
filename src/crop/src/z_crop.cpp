#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class ZFixedPointCloud : public rclcpp::Node {
public:
    ZFixedPointCloud() : Node("z_fixed_pointcloud") {
        // LiDAR 데이터 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/cropbox_filtered",  // 사용할 LiDAR 토픽
            10,
            std::bind(&ZFixedPointCloud::lidar_callback, this, std::placeholders::_1));

        // 변환된 데이터 발행
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/z_crop", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 새로운 PointCloud2 메시지 생성
        sensor_msgs::msg::PointCloud2 fixed_msg;
        fixed_msg.header = msg->header;
        fixed_msg.header.frame_id = "velodyne";  // 필요시 변경 가능
        fixed_msg.height = 1;
        fixed_msg.is_dense = false;

        // 입력된 PointCloud2 데이터를 순회하면서 z 값을 0으로 변경
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, "intensity");

        std::vector<float> x_vals, y_vals, z_vals, intensity_vals;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (*iter_z >= -3.0 && *iter_z <= -1.2) {  // z 값이 -3 ~ 3 범위
                x_vals.push_back(*iter_x);
                y_vals.push_back(*iter_y);
                z_vals.push_back(0.0f);  // z 값을 0으로 변경
                intensity_vals.push_back(*iter_intensity);
            }
        }

        // 필터링된 포인트 수
        size_t num_points = x_vals.size();
        if (num_points == 0) {
            RCLCPP_WARN(this->get_logger(), "No points found in the valid z range");
            return;
        }

        // 필터링된 데이터를 PointCloud2 형식으로 변환
        fixed_msg.width = num_points;
        fixed_msg.row_step = fixed_msg.point_step * num_points;

        fixed_msg.fields = msg->fields;
        fixed_msg.point_step = msg->point_step;
        fixed_msg.data.resize(num_points * fixed_msg.point_step);

        // 새로운 PointCloud2 데이터에 복사
        sensor_msgs::PointCloud2Iterator<float> iter_out_x(fixed_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_out_y(fixed_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_out_z(fixed_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_out_intensity(fixed_msg, "intensity");

        for (size_t i = 0; i < num_points; ++i, ++iter_out_x, ++iter_out_y, ++iter_out_z, ++iter_out_intensity) {
            *iter_out_x = x_vals[i];
            *iter_out_y = y_vals[i];
            *iter_out_z = z_vals[i];  // z = 0
            *iter_out_intensity = intensity_vals[i];
        }

        // 변환된 데이터 발행
        publisher_->publish(fixed_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu points with fixed z = 0", num_points);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ZFixedPointCloud>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

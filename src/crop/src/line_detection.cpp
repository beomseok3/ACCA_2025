#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

class LidarReflectivityFilter : public rclcpp::Node {
public:
    LidarReflectivityFilter() : Node("lidar_reflectivity_filter") {
        // LiDAR 데이터 구독
        subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/z_crop",  // 사용할 LiDAR 토픽
            10,
            std::bind(&LidarReflectivityFilter::lidar_callback, this, std::placeholders::_1));

        // 필터링된 데이터 발행
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/lane_filtered", 10);
        
        // 반사도 임계값
        reflectivity_threshold_ = 100;
    }

private:
    void lidar_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        // 새로운 PointCloud2 메시지 생성
        sensor_msgs::msg::PointCloud2 filtered_msg;
        filtered_msg.header = msg->header;
        filtered_msg.height = 1;
        filtered_msg.is_dense = false;

        // 포인트 클라우드 데이터 필터링
        sensor_msgs::PointCloud2Iterator<float> iter_x(*msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(*msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(*msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_intensity(*msg, "intensity");

        std::vector<float> x_vals, y_vals, z_vals, intensity_vals;

        for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z, ++iter_intensity) {
            if (*iter_intensity > reflectivity_threshold_) {  // 반사도 100 이상 필터링
                x_vals.push_back(*iter_x);
                y_vals.push_back(*iter_y);
                z_vals.push_back(*iter_z);
                intensity_vals.push_back(*iter_intensity);
            }
        }

        // 필터링된 포인트 수 확인
        size_t num_points = x_vals.size();
        if (num_points == 0) {
            RCLCPP_WARN(this->get_logger(), "No points found with reflectivity > %d", reflectivity_threshold_);
            return;
        }

        // 필터링된 데이터를 PointCloud2 형식으로 변환
        filtered_msg.width = num_points;
        filtered_msg.row_step = filtered_msg.point_step * num_points;

        filtered_msg.fields = msg->fields;
        filtered_msg.point_step = msg->point_step;
        filtered_msg.data.resize(num_points * filtered_msg.point_step);

        // 새로운 PointCloud2 데이터에 복사
        sensor_msgs::PointCloud2Iterator<float> iter_out_x(filtered_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_out_y(filtered_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_out_z(filtered_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_out_intensity(filtered_msg, "intensity");

        for (size_t i = 0; i < num_points; ++i, ++iter_out_x, ++iter_out_y, ++iter_out_z, ++iter_out_intensity) {
            *iter_out_x = x_vals[i];
            *iter_out_y = y_vals[i];
            *iter_out_z = z_vals[i];
            *iter_out_intensity = intensity_vals[i];
        }

        // 필터링된 데이터 발행
        publisher_->publish(filtered_msg);
        RCLCPP_INFO(this->get_logger(), "Published %zu high-reflectivity points", num_points);
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    int reflectivity_threshold_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<LidarReflectivityFilter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

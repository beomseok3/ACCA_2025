#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <iostream>
#include <vector>

using std::placeholders::_1;

class VelodyneSubscriber : public rclcpp::Node
{
public:
    VelodyneSubscriber() : Node("cropbox_filter")
    {
        velodyne_raw_subscriber_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/velodyne_points", 10,
            std::bind(&VelodyneSubscriber::velodyneCallback, this, _1));

        cropbox_publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/cropbox_filtered", 10);
    }

private:
    void velodyneCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        std::vector<uint8_t> filtered_data;

        int x_offset = -1;
        int y_offset = -1;
        for (const auto &field : msg->fields) {
            if (field.name == "x") {
                x_offset = field.offset;
            } else if (field.name == "y") {
                y_offset = field.offset;
            }
        }
        std::cout << "offset_xyz : "<<x_offset <<" "<<y_offset <<std::endl;
        if (x_offset == -1 || y_offset == -1) {
            RCLCPP_ERROR(this->get_logger(), "PointCloud2 message does not have 'x' or 'y' fields.");
            return;
        }

        // const size_t float_size = 4;
        // const float min_x = -0.5;
        // const float max_x = 10;  //4
        // const float min_y = -4;   //-1.8
        // const float max_y = 4;    //1.8

        const size_t float_size = 4; //line detection용
        const float min_x = -3;
        const float max_x = 20;  //4
        const float min_y = -3.5;   //-1.8
        const float max_y = 3.5;    //1.8

        // const size_t float_size = 4;
        // const float min_x = 20;
        // const float max_x = 0;  //4
        // const float min_y = -10;   //-1.8
        // const float max_y = 10;    //1.8

        for (size_t i = 0; i < msg->data.size(); i += msg->point_step)
        {
            float x, y;
            std::memcpy(&x, &msg->data[i + x_offset], float_size);
            std::memcpy(&y, &msg->data[i + y_offset], float_size);
            //std::cout << "x : " << x <<"," << "y :" << y << std::endl; 
            if (x >= min_x && x <= max_x && y >= min_y && y <= max_y)
            {
                for (size_t j = 0; j < msg->point_step; ++j)
                {
                    filtered_data.push_back(msg->data[i + j]);
                }
            }
        }

        auto processed_msg = std::make_shared<sensor_msgs::msg::PointCloud2>();
        processed_msg->header = msg->header;

        processed_msg->height = 1;
        processed_msg->width = filtered_data.size() / msg->point_step;
        processed_msg->fields = msg->fields;
        processed_msg->is_bigendian = msg->is_bigendian;
        processed_msg->point_step = msg->point_step;
        processed_msg->row_step = processed_msg->point_step * processed_msg->width;
        processed_msg->data = filtered_data;
        processed_msg->is_dense = true;

        if (processed_msg->width * processed_msg->point_step != processed_msg->data.size()) {
            RCLCPP_ERROR(this->get_logger(), "Data size (%zu bytes) does not match width (%u) times point_step (%u).",
                         processed_msg->data.size(), processed_msg->width, processed_msg->point_step);
        } else {
            cropbox_publisher_->publish(*processed_msg);
        }
    }
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cropbox_publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr velodyne_raw_subscriber_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<VelodyneSubscriber>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}

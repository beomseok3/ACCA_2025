#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "yolo_msg/msg/bounding_box_array.hpp"
#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <string>

struct ConeInfo {
    double x, y, z;         // 3D 원본 좌표
    double u, v, k;         // 투영값
    double pixel_x, pixel_y; // 세그먼트 중심 픽셀
    double bbox_x, bbox_y, bbox_w, bbox_h; // 바운딩 박스 정보
    std::string cone_class; // "yellow" 또는 "blue"
};

class SingleCameraTestNode : public rclcpp::Node {
public:
    SingleCameraTestNode() : Node("single_camera_test_node") {
        cone_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "cone_poses", 10, std::bind(&SingleCameraTestNode::coneCallback, this, std::placeholders::_1));
        
        pixel_yellow_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "yellow_cone", 10, std::bind(&SingleCameraTestNode::pixelYellowCallback, this, std::placeholders::_1));
        
        pixel_blue_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "blue_cone", 10, std::bind(&SingleCameraTestNode::pixelBlueCallback, this, std::placeholders::_1));

        pixel_circle_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "pixel_circle", 10, std::bind(&SingleCameraTestNode::pixelCircleCallback, this, std::placeholders::_1));
        
        bbox_sub_ = this->create_subscription<yolo_msg::msg::BoundingBoxArray>(
            "bounding_box", 10, std::bind(&SingleCameraTestNode::bboxCallback, this, std::placeholders::_1));

        yellow_point_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>("point/yellow", 10);
        blue_point_pub_   = this->create_publisher<geometry_msgs::msg::PointStamped>("point/blue", 10);

        // External Parameters: R_RTlc (Rotation + Translation)
        Eigen::Matrix4d R_RTlc;
        R_RTlc << -0.494572094519085, -0.868790783828116, 0.0245156533327822, -0.258490146877099,
                  -0.213263328576726, 0.0939621557924048, -0.972465868790889, -0.114441846255876,
                  0.842565840755119, -0.486182771408215, -0.231752274591745, -0.179935292086361,
                  0., 0., 0., 1.;

        Eigen::Matrix4d R_Mc;
        R_Mc << 504.825019262389, 0.0, 331.810631799415, 0.0,
                0.0, 503.956602039494, 245.404192420775, 0.0,
                0.000000, 0.000000, 1.000000, 0.0,
                0.0, 0.0, 0.0, 1.0;
        R_result = R_Mc * R_RTlc;
    }

private:
    geometry_msgs::msg::PointStamped::SharedPtr last_pixel_yellow_;
    geometry_msgs::msg::PointStamped::SharedPtr last_pixel_blue_;
    geometry_msgs::msg::PointStamped::SharedPtr last_pixel_circle_;
    yolo_msg::msg::BoundingBoxArray::SharedPtr last_bboxes_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pixel_yellow_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pixel_blue_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr pixel_circle_sub_;
    rclcpp::Subscription<yolo_msg::msg::BoundingBoxArray>::SharedPtr bbox_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_point_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_point_pub_;

    // Callback functions for incoming messages
    void pixelYellowCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) { last_pixel_yellow_ = msg; }
    void pixelBlueCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) { last_pixel_blue_ = msg; }
    void pixelCircleCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg) { last_pixel_circle_ = msg; }
    void bboxCallback(const yolo_msg::msg::BoundingBoxArray::SharedPtr msg) { last_bboxes_ = msg; }

    // Cone callback
    void coneCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) {
        double threshold = 1e-4;
        std::vector<ConeInfo> result_list;

        processConeClass(msg, last_pixel_yellow_, "yellow", yellow_point_pub_, result_list, threshold);
        processConeClass(msg, last_pixel_blue_, "blue", blue_point_pub_, result_list, threshold);

        // Print the result list
        printResultList(result_list);
    }

    // Function to print the ConeInfo
    void printResultList(const std::vector<ConeInfo>& result_list) {
        for (const auto& cone : result_list) {
            RCLCPP_INFO(this->get_logger(), "Cone Info: ");
            RCLCPP_INFO(this->get_logger(), "  Class: %s", cone.cone_class.c_str());
            RCLCPP_INFO(this->get_logger(), "  Position (x, y, z): (%.3f, %.3f, %.3f)", cone.x, cone.y, cone.z);
            RCLCPP_INFO(this->get_logger(), "  Projection (u, v): (%.3f, %.3f)", cone.u, cone.v);
            RCLCPP_INFO(this->get_logger(), "  Pixel Position (x, y): (%.3f, %.3f)", cone.pixel_x, cone.pixel_y);
            RCLCPP_INFO(this->get_logger(), "  Bounding Box (x, y, w, h): (%.3f, %.3f, %.3f, %.3f)", cone.bbox_x, cone.bbox_y, cone.bbox_w, cone.bbox_h);
        }
    }

    // Function to process each cone class (yellow, blue)
    void processConeClass(const geometry_msgs::msg::PoseArray::SharedPtr& msg,
                      const geometry_msgs::msg::PointStamped::SharedPtr& last_pixel,
                      const std::string& cone_class,
                      const rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr& point_pub,
                      std::vector<ConeInfo>& result_list,
                      double threshold) 
{
    RCLCPP_INFO(this->get_logger(), "Processing %s cone", cone_class.c_str());
    if ((!last_pixel && !last_pixel_circle_) || !last_bboxes_ || last_bboxes_->boxes.empty()) {
        RCLCPP_WARN(this->get_logger(), "No pixel data or bounding boxes available.");
        return;
    }

    // Set pixel_x and pixel_y based on last received point data
    double pixel_x = last_pixel_circle_ ? last_pixel_circle_->point.x : last_pixel->point.x;
    double pixel_y = last_pixel_circle_ ? last_pixel_circle_->point.y : last_pixel->point.y;

    for (const auto& pose : msg->poses) {
        RCLCPP_DEBUG(this->get_logger(), "Checking cone at position: (%.3f, %.3f, %.3f)", pose.position.x, pose.position.y, pose.position.z);

        Eigen::Vector4d pt3d(pose.position.x, pose.position.y, pose.position.z, 1.0);
        Eigen::Vector4d transformed = R_result * pt3d;

        double x = transformed(0);
        double y = transformed(1);
        double z = transformed(2);

        if (std::abs(z) < 1e-6) {
            RCLCPP_DEBUG(this->get_logger(), "Skipping cone due to small z value.");
            continue;
        }

        double u = x / z;
        double v = y / z;
        double k = z;

        if (u > 0 && u < 640) {
            int bbox_count = 0;
            int bbox_index = -1;
            for (size_t i = 0; i < last_bboxes_->boxes.size(); ++i) {
                const auto& bbox = last_bboxes_->boxes[i];
                RCLCPP_DEBUG(this->get_logger(), "Checking bounding box: (%.3f, %.3f, %.3f, %.3f)", bbox.x, bbox.y, bbox.width, bbox.height);
                if (u >= bbox.x && u <= bbox.x + bbox.width &&
                    v >= bbox.y && v <= bbox.y + bbox.height) {
                    ++bbox_count;
                    bbox_index = static_cast<int>(i);
                }
            }

            if (bbox_count == 1 && bbox_index >= 0) {
                RCLCPP_INFO(this->get_logger(), "Adding cone to result list.");
                result_list.push_back({
                    pose.position.x, pose.position.y, pose.position.z,
                    u, v, k,
                    pixel_x, pixel_y,
                    last_bboxes_->boxes[bbox_index].x,
                    last_bboxes_->boxes[bbox_index].y,
                    last_bboxes_->boxes[bbox_index].width,
                    last_bboxes_->boxes[bbox_index].height,
                    cone_class
                });

                geometry_msgs::msg::PointStamped point_msg;
                point_msg.header.stamp = this->now();
                point_msg.header.frame_id = "velodyne";
                point_msg.point.x = pose.position.x;
                point_msg.point.y = pose.position.y;
                point_msg.point.z = pose.position.z;
                point_pub->publish(point_msg);
            }
        }
    }

    RCLCPP_INFO(this->get_logger(), "Processed %zu cones for %s class.", result_list.size(), cone_class.c_str());
}

    Eigen::Matrix4d R_RTlc;   // 외부 매개변수
    Eigen::Matrix4d R_Mc;      // 카메라 내부 매개변수
    Eigen::Matrix4d R_result;  // 최종 변환 행렬
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SingleCameraTestNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

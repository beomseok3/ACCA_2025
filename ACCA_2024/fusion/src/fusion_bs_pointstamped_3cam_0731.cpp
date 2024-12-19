#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <omp.h>
#include <mutex>

using std::placeholders::_1;

class FusionNode : public rclcpp::Node {
public:
    FusionNode() : Node("bs_Camera_LiDAR_Fusion") {
        cone_poses_subscription = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/cone_poses", 10,
            std::bind(&FusionNode::ConePoseCallback, this, _1));

        bounding_boxes_subscription = this->create_subscription<darknet_ros_msgs::msg::BoundingBoxes>(
            "/bounding_boxes", 10,
            std::bind(&FusionNode::boundingBoxesCallback, this, _1));

        yellow_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("poses/yellow", 10);
        blue_publisher = this->create_publisher<geometry_msgs::msg::PoseArray>("poses/blue", 10);

        R_RTlc << -0.347624192331869, -0.937628203123319, 0.00328231828922415, -0.211912809724974,
                  -0.272522800473639, 0.0976866492508762, -0.957177434846926, -0.207068664109966,
                   0.897155919630262, -0.333632539279178, -0.289483306265675, -0.0934837954434088,
                   0., 0., 0., 1.;


        R_Mc << 535.675025668494, 0.000000, 333.913092315142, 0.0,
                0.000000, 534.736852975067, 250.724983581, 0.0,
                0.000000, 0.000000, 1.000000, 0.0;


        C_Mc << 627.735492099247, 0., 336.539893173783, 0.,
                  0., 626.560686842897, 231.417340787929, 0.,
                  0., 0., 1., 0.;

        C_RTlc << -0.0186639176860175, -0.999758559244839, -0.0115966112848825, 0.0500832719924129,
                   0.045164508844289, 0.0107437549862607, -0.998921788164443, 0.271694171397735,
                   0.998805198883878, -0.0191675492820104, 0.0449530837324648, 0.425836353178122,
                   0., 0., 0., 1.;


        L_RTlc << 0.39939818452164, -0.916776024574278, 0.001676593749425,	0.133101702016476,
                  -0.252929410395892, -0.111947441943875, -0.96098620364655, -0.159941862200059,
                  0.881196801831239, 0.38339208521826, -0.276591224434888,	-0.157837019027453,
                  0., 0., 0., 1.;


        L_Mc << 516.862336852068, 0., 323.465071695582, 0.,
                0., 517.983030345846, 266.831396862175, 0.,
                0., 0., 1., 0.;


        L_result = L_Mc * L_RTlc;
        C_result = C_Mc * C_RTlc;
        R_result = R_Mc * R_RTlc;
    }

private:
    void ConePoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        geometry_msgs::msg::PoseArray yellow_arr;
        geometry_msgs::msg::PoseArray blue_arr;

        for (const auto& pose : msg->poses)
        {
            bool C_poj = false;
            bool R_poj = false;
            bool L_poj = false;

            bool yellow_matching = false;
            bool blue_matching = false;

            Eigen::Vector4d velodyne_3D;
            velodyne_3D << pose.position.x, pose.position.y, pose.position.z, 1.0;
            Eigen::Vector3d projected_LiDAR_C = C_result * velodyne_3D;
            Eigen::Vector3d projected_LiDAR_R = R_result * velodyne_3D;
            Eigen::Vector3d projected_LiDAR_L = L_result * velodyne_3D;

            float scale_factor_C = projected_LiDAR_C(2);
            float scale_factor_R = projected_LiDAR_R(2);
            float scale_factor_L = projected_LiDAR_L(2);

            projected_LiDAR_C = projected_LiDAR_C / scale_factor_C;
            projected_LiDAR_R = projected_LiDAR_R / scale_factor_R;
            projected_LiDAR_L = projected_LiDAR_L / scale_factor_L;

            if (projected_LiDAR_C(0) >= 0 && projected_LiDAR_C(0) <= 640 && projected_LiDAR_C(1) >= 0 && projected_LiDAR_C(1) <= 480) C_poj = true;
            if (projected_LiDAR_R(0) >= 0 && projected_LiDAR_R(0) <= 640 && projected_LiDAR_R(1) >= 0 && projected_LiDAR_R(1) <= 480) R_poj = true;
            if (projected_LiDAR_L(0) >= 0 && projected_LiDAR_L(0) <= 640 && projected_LiDAR_L(1) >= 0 && projected_LiDAR_L(1) <= 480) L_poj = true;


            if(boxes_ != nullptr)
            {
                
                if(C_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {   
                        if (projected_LiDAR_C(0) >= box.xmin - 640 && projected_LiDAR_C(0) <= box.xmax - 640 && projected_LiDAR_C(1) >= box.ymin && projected_LiDAR_C(1) <= box.ymax)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    } 

                    if (blue_matching && !yellow_matching) blue_arr.poses.push_back(pose);
                    else if (!blue_matching && yellow_matching) yellow_arr.poses.push_back(pose);
                    else if (blue_matching && yellow_matching) std::cout << "C: duplicate matching" << std::endl;
                    else std::cout << "C: not matching" << std::endl;
                }

                else if (R_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_R(0) >= box.xmin - 1280 && projected_LiDAR_R(0) <= box.xmax - 1280 && projected_LiDAR_R(1) >= box.ymin && projected_LiDAR_R(1) <= box.ymax)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;

                        }

                    }

                    if (blue_matching && !yellow_matching) blue_arr.poses.push_back(pose);
                    else if (!blue_matching && yellow_matching) yellow_arr.poses.push_back(pose);
                    else if (blue_matching && yellow_matching) std::cout << "R: duplicate matching" << std::endl;
                    else std::cout << "R: not matching" << std::endl;
                }

                else if (L_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_L(0) >= box.xmin && projected_LiDAR_L(0) <= box.xmax && projected_LiDAR_L(1) >= box.ymin && projected_LiDAR_L(1) <= box.ymax)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    }

                    if (blue_matching && !yellow_matching) blue_arr.poses.push_back(pose);
                    else if (!blue_matching && yellow_matching) yellow_arr.poses.push_back(pose);
                    else if (blue_matching && yellow_matching) std::cout << "L: duplicate matching" << std::endl;
                    else std::cout << "L: not matching" << std::endl;

                }

                else std::cout << "not in camera angle" << std::endl;

            }
        }

        yellow_arr.header.stamp = this->now();
        yellow_arr.header.frame_id ="velodyne";
        yellow_publisher->publish(yellow_arr);

        blue_arr.header.stamp = this->now();
        blue_arr.header.frame_id ="velodyne";
        blue_publisher->publish(blue_arr);

        boxes_ = nullptr;
    }

    void boundingBoxesCallback(const darknet_ros_msgs::msg::BoundingBoxes::SharedPtr msg) 
    {
        if (boxes_ == nullptr) boxes_ = msg;
    }


    Eigen::Matrix<double, 3, 4> L_Mc;
    Eigen::Matrix<double, 3, 4> R_Mc;
    Eigen::Matrix<double, 3, 4> C_Mc;

    Eigen::Matrix4d R_RTlc;
    Eigen::Matrix4d C_RTlc;
    Eigen::Matrix4d L_RTlc;

    
    Eigen::Matrix<double, 3, 4> R_result;
    Eigen::Matrix<double, 3, 4> C_result;
    Eigen::Matrix<double, 3, 4> L_result;
    
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr cone_poses_subscription;
    rclcpp::Subscription<darknet_ros_msgs::msg::BoundingBoxes>::SharedPtr bounding_boxes_subscription;

    darknet_ros_msgs::msg::BoundingBoxes::SharedPtr boxes_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr yellow_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr blue_publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}








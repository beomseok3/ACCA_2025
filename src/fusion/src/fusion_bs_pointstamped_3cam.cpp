#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "darknet_ros_msgs/msg/bounding_boxes.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
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

        yellow_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/yellow", 10);
        blue_publisher = this->create_publisher<geometry_msgs::msg::PointStamped>("point/blue", 10);


        //1018
        // R_RTlc << -0.494572094519085,	-0.868790783828116,	0.0245156533327822,	-0.258490146877099,
        //           -0.213263328576726,	0.0939621557924048,	-0.972465868790889,	-0.114441846255876,
        //            0.842565840755119,	-0.486182771408215,	-0.231752274591745,	-0.179935292086361,
        //            0., 0., 0., 1.;


        // R_Mc << 504.825019262389,	0.0,	331.810631799415,  0.0,
        //         0.0,	503.956602039494,	245.404192420775, 0.0,
        //         0.000000, 0.000000, 1.000000, 0.0;




        // C_RTlc << 0.0219087204896778,	-0.999707258216931,	0.0102667343830515,	-0.0279157166545777,
        //            -0.237333870712555,	-0.0151763017066082,	-0.971309586938740,	-0.145387423857104,
        //            0.971181055096887,	0.0188435064384779,	-0.237596886524217,	-0.0492311382578551,
        //            0., 0., 0., 1.;


        // C_Mc << 625.147805669636,	0.0,	329.488400900984, 0.,
        //           0.0, 624.496914634015, 	238.751886816586, 0.,
        //           0., 0., 1., 0.;


        // L_RTlc << 0.449539807061320,	-0.893192741378495,	-0.0109858370660250,	0.182665535123044,
        //           -0.282566945491583,	-0.130526102496527,	-0.950325658857332,	-0.00839018970426207,
        //           0.847390041942215,	0.430313447751562,	-0.311063423599330,	-0.217170757834700,
        //           0., 0., 0., 1.;


        // L_Mc << 499.170379415067,	0.0,	316.703211542153, 0.,
        //         0.0,	501.782495894438,	259.763423697133, 0.,
        //         0., 0., 1., 0.;
        
        //0309

        // Transformation matrix Rtform
        R_RTlc << -0.258213671681845,	-0.964342407567990,	0.058046711556751,	-0.246695652137148,
        -0.324116745139498,	0.029870691462926,	-0.945545386172290,	-0.119482435620032,
        0.910095618754842,	-0.262966657121200,	-0.320272543259242,	-0.167583798275389,
        0.0,	0.0,	0.0,	1.000000000000000;
        

        // Intrinsic matrix L_intrinsic
        R_Mc << 501.1398860386980,	0.0,	313.1408063353412, 0.0,
        0.0,	502.7523607012868,	263.9275020222599, 0.0,
        0.0,	0.0,	1.0, 0.0;

        // Transformation matrix L_RTlc
        L_RTlc << 0.467241296148468,	-0.884101949015081,	-0.007022458346980,	0.235473230253985,
        -0.345499072351193,	-0.175270950127970,	-0.921905898151055,	-0.051488286244499,
        0.813827968417137,	0.433178759623523,	-0.387350229163559,	-0.081947679209619,
        0.0,	0.0,	0.0,	1.000000000000000;

        // Intrinsic matrix R_intrinsic
        L_Mc << 519.2213279054756,	0.0,	330.6650295893565, 0.0,
        0.0,	524.8087398291260,	237.7200188582734, 0.0,
        0.0,	0.0,	1.0, 0.0;

        // Transformation matrix Ctform
        C_RTlc << 0.0572,   -0.9980,    0.0270,    0.0271,
        0.1006,   -0.0211,   -0.9947,   -0.2647,
        0.9933,   0.0596,    0.0992,   -0.1308,
             0.0,         0.0,         0.0,    1.0000;

        // Intrinsic matrix C_intrinsic
        C_Mc << 600.0215532765091,	0.0,	328.4470315774296, 0.0,
        0.0,	600.7558135485723,	236.1876388527033, 0.0,
        0.0,	0.0,	1.0, 0.0;
      

        L_result = L_Mc * L_RTlc;
        C_result = C_Mc * C_RTlc;
        R_result = R_Mc * R_RTlc;
    }

private:
    void ConePoseCallback(const geometry_msgs::msg::PoseArray::SharedPtr msg) 
    {
        for (const auto& pose : msg->poses)
        {
            float offset = 0;

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


            if (boxes_ != nullptr)
            {
                if (C_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {   
                        if (projected_LiDAR_C(0) >= box.xmin - 640 - offset && projected_LiDAR_C(0) <= box.xmax - 640 + offset && projected_LiDAR_C(1) >= box.ymin - offset && projected_LiDAR_C(1) <= box.ymax + offset)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    } 

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "C: duplicate matching" << std::endl;
                    else std::cout << "C: not matching" << std::endl;
                }

                else if (R_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_R(0) >= box.xmin - 1280 - offset && projected_LiDAR_R(0) <= box.xmax - 1280 + offset && projected_LiDAR_R(1) >= box.ymin - offset && projected_LiDAR_R(1) <= box.ymax + offset)
                        {
                            if (box.class_id == "y_cone") yellow_matching = true;
                            else blue_matching = true;
                        }
                    }

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "R: duplicate matching" << std::endl;
                    else std::cout << "R: not matching" << std::endl;
                }

                else if (L_poj) 
                {
                    for (const auto& box : boxes_->bounding_boxes)
                    {
                        if (projected_LiDAR_L(0) >= box.xmin - offset && projected_LiDAR_L(0) <= box.xmax + offset && projected_LiDAR_L(1) >= box.ymin - offset && projected_LiDAR_L(1) <= box.ymax + offset)
                        {
                            if (box.class_id == "b_cone") blue_matching = true;
                            else yellow_matching = true;
                        }
                    }

                    if (blue_matching && !yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        blue_publisher->publish(point);
                    }
                    else if (!blue_matching && yellow_matching) {
                        geometry_msgs::msg::PointStamped point;
                        point.header = msg->header;
                        // point.header.stamp = this->now();
                        point.point = pose.position;
                        yellow_publisher->publish(point);
                    }
                    else if (blue_matching && yellow_matching) std::cout << "L: duplicate matching" << std::endl;
                    else std::cout << "L: not matching" << std::endl;
                }

                else std::cout << "not in camera angle" << std::endl;
            }
        }

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

    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr yellow_publisher;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr blue_publisher;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FusionNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

//TODO 

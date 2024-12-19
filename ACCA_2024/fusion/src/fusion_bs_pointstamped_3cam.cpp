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

        //0815
        // R_RTlc << -0.540780753246683,	-0.841095795037971,	-0.0106789740796408,	-0.230803099681106,
        //           -0.209697033646174,	0.147097186451735,	-0.966638283856986,	-0.0698769049482540,
        //            0.814606242916137,	-0.520500030074377,	-0.255922620521570,	-0.106486984809243,
        //            0., 0., 0., 1.;


        // R_Mc << 511.991365106542,	0.,	330.746238600349, 0.0,
        //         0.,	511.481569576966,	245.903202607243, 0.0,
        //         0.000000, 0.000000, 1.000000, 0.0;


        // C_Mc << 630.016651346934,	0.,	330.144208074156, 0.,
        //           0.,	629.701384248169,	235.569441144491, 0.,
        //           0., 0., 1., 0.;

        // C_RTlc << -0.0241650755896753,	-0.999650057277854, 0.0107616033251847,	-0.00435194941401235,
        //            -0.228112919644660,	-0.00496710614653574,	-0.973622012768671,	-0.190633757573411,
        //            0.973334754857204,	-0.0259825102888923,	-0.227913062693110,	-0.0285812390646237,
        //            0., 0., 0., 1.;


        // L_RTlc << 0.525843727780622,	-0.850558209743799,	0.00625346233720610,	0.223127929195404,
        //           -0.271150336499009,	-0.174593627104406,	-0.946569891973647,	-0.0351900569032066,
        //           0.806204607385900,	0.496052212183306,	-0.322438108507346,	-0.107113896751372,
        //           0., 0., 0., 1.;


        // L_Mc << 511.061950884841,	0.,	312.926603750756, 0.,
        //         0.,	510.841873103909,	262.370422349730, 0.,
        //         0., 0., 1., 0.;

        //1009
        // R_RTlc << -0.547355160528228,	-0.836845607800186,	0.00957898474030898,	-0.231684346799313,
        //           -0.189450969040536,	0.112749195555144,	-0.975395278454468,	-0.111951985997715,
        //            0.81517523181995,	-0.535702387158398,	-0.220255065367576,	-0.0735547310357550,
        //            0., 0., 0., 1.;


        // R_Mc << 523.280487283662,	0.,	331.396939222695, 0.0,
        //         0.,	523.004485229485,	244.267780675046, 0.0,
        //         0.000000, 0.000000, 1.000000, 0.0;


        // C_Mc << 620.054910601751,	0.,	326.435303872741, 0.,
        //           0.,	618.468984940890,	237.365071179434, 0.,
        //           0., 0., 1., 0.;

        // C_RTlc << -0.0340364002844616,	-0.999362897020636,	0.0107388786285244,	0.0322736349306722,
        //            -0.236410473662428,	-0.155902402452389,	-0.959064402869943,	-0.0237211450414423,
        //            0.972107002377615,	-0.0355991939756286,	-0.231820347072239,	-0.0859509874131871,
        //            0., 0., 0., 1.;


        // L_RTlc << 0.489642867319695,	-0.871670196539528,	0.0209983558329517,	0.211221904591083,
        //           -0.236410473662428,	-0.155902402452389,	-0.959064402869943,	-0.0237211450414423,
        //           0.839261550665616,	0.464634812916890,	-0.282408463400148,	-0.218519261089965,
        //           0., 0., 0., 1.;


        // L_Mc << 496.144731786776,	0.,	314.557236612844, 0.,
        //         0.,	496.483242254617,	262.924196368225, 0.,
        //         0., 0., 1., 0.;


        //1018
        // R_RTlc << -0.515040991266866,	-0.857150658155733,	-0.00505237944311653,	-0.214695163399048,
        //           -0.216078001679291,	0.135535439287721,	-0.966923183033362,	-0.00485539591902331,
        //            0.829483619190354,	-0.496913366614622,	-0.255017708354918,	-0.108731177990117,
        //            0., 0., 0., 1.;


        // R_Mc << 510.192310097154,	0.0,	331.452784321718,  0.0,
        //         0.0,	508.177252597106,	244.611886940820, 0.0,
        //         0.000000, 0.000000, 1.000000, 0.0;


        // C_RTlc << 0.0219087204896778,	-0.999707258216931,	0.0102667343830515,	-0.0279157166545777,
        //            -0.237333870712555,	-0.0151763017066082,	-0.971309586938740,	-0.145387423857104,
        //            0.971181055096887,	0.0188435064384779,	-0.237596886524217,	-0.0492311382578551,
        //            0., 0., 0., 1.;


        // C_Mc << 625.147805669636,	0.0,	329.488400900984, 0.,
        //           0.0, 624.496914634015, 	238.751886816586, 0.,
        //           0., 0., 1., 0.;


        // L_RTlc << 0.520089876550331,	-0.854098280067479,	0.00475902255051573,	0.240033645191868,
        //           -0.233724190728693,	-0.147677517287043,	-0.961022556216113,	-0.0662877370792616,
        //           0.821510513005208,	0.498705803930243,	-0.276429011046618,	-0.113865684473383,
        //           0., 0., 0., 1.;


        // L_Mc << 511.996745815621,	0.0,	317.263304552098, 0.,
        //         0.0,	512.063345681316,	263.137623635348, 0.,
        //         0., 0., 1., 0.;

        //1018
        R_RTlc << -0.494572094519085,	-0.868790783828116,	0.0245156533327822,	-0.258490146877099,
                  -0.213263328576726,	0.0939621557924048,	-0.972465868790889,	-0.114441846255876,
                   0.842565840755119,	-0.486182771408215,	-0.231752274591745,	-0.179935292086361,
                   0., 0., 0., 1.;


        R_Mc << 504.825019262389,	0.0,	331.810631799415,  0.0,
                0.0,	503.956602039494,	245.404192420775, 0.0,
                0.000000, 0.000000, 1.000000, 0.0;




        C_RTlc << 0.0219087204896778,	-0.999707258216931,	0.0102667343830515,	-0.0279157166545777,
                   -0.237333870712555,	-0.0151763017066082,	-0.971309586938740,	-0.145387423857104,
                   0.971181055096887,	0.0188435064384779,	-0.237596886524217,	-0.0492311382578551,
                   0., 0., 0., 1.;


        C_Mc << 625.147805669636,	0.0,	329.488400900984, 0.,
                  0.0, 624.496914634015, 	238.751886816586, 0.,
                  0., 0., 1., 0.;


        L_RTlc << 0.449539807061320,	-0.893192741378495,	-0.0109858370660250,	0.182665535123044,
                  -0.282566945491583,	-0.130526102496527,	-0.950325658857332,	-0.00839018970426207,
                  0.847390041942215,	0.430313447751562,	-0.311063423599330,	-0.217170757834700,
                  0., 0., 0., 1.;


        L_Mc << 499.170379415067,	0.0,	316.703211542153, 0.,
                0.0,	501.782495894438,	259.763423697133, 0.,
                0., 0., 1., 0.;






        


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

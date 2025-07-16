#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/float32.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <fstream>
#include <cmath>
#include <limits>
#include <string>
#include <vector>
#include <proj.h>

class ObstacleAvoid : public rclcpp::Node
{
public:
    ObstacleAvoid(std::string frame_id, double period)
        : Node("obstacle_avoid"),
          frame_id_(frame_id),
          tf_buffer_(this->get_clock()),
          tf_listener_(tf_buffer_)
    {
        this->declare_parameter<std::string>("file_path1", "/home/gjs/obstacle_path/path1.txt");
        this->declare_parameter<std::string>("file_path2", "/home/gjs/obstacle_path/path2.txt");
        // this->declare_parameter<std::string>("global_path", "/home/gjs/global_path/school_speed.txt");


        file_path1_ = this->get_parameter("file_path1").as_string();
        file_path2_ = this->get_parameter("file_path2").as_string();
        // global_path_ = this->get_parameter("global_path").as_string();

        obstacle_pose_sub_ = this->create_subscription<geometry_msgs::msg::Pose>("obstacle_pose", 10, std::bind(&ObstacleAvoid::obstacle_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10, std::bind(&ObstacleAvoid::odom_callback, this, std::placeholders::_1));

        path1_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path1/obstacle", 10);
        path2_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path2/obstacle", 10);
        // path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path/global", 10);

        speed1_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_speed1", 10);
        speed2_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_speed2", 10);
        // speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_speed", 10);
        

        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / period)), std::bind(&ObstacleAvoid::timer_callback, this));

        odom_position_ = nullptr;
        obstacle_position_ = nullptr;

        map_lat_ = 37.4966945; //lat
        map_lon_ = 126.9575076; //lon
        distance_threshold_ = 1.5; // [m]

        P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr);
        if (P == nullptr)
        {
            // RCLCPP_ERROR(this->get_logger(), "Failed to create projection.");
            throw std::runtime_error("Failed to create projection.");
        }

        PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
        PJ_COORD result = proj_trans(P, PJ_FWD, origin);

        // 왜 인지는 모르겠지만 cpp에서 proj 모듈 쓰면 x와 y가 반대로 나옴
        map_y_ = result.xy.x;
        map_x_ = result.xy.y;
    }

    ~ObstacleAvoid()
    {
        if (P)
        {
            proj_destroy(P);
        }
    }

private:
    void obstacle_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        obstacle_position_ = msg;
        // RCLCPP_INFO(this->get_logger(), "Obstacle position updated: x: %f, y: %f", obstacle_position_->position.x, obstacle_position_->position.y);
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_position_ = std::make_shared<geometry_msgs::msg::Point>(msg->pose.pose.position);
        // RCLCPP_INFO(this->get_logger(), "Odom position updated: x: %f, y: %f", odom_position_->x, odom_position_->y);
    }

    std::vector<geometry_msgs::msg::Pose> file_opener(const std::string &file_path, std::vector<float> &speeds)
    {
        std::ifstream infile(file_path);
        if (!infile.is_open())
        {
            // RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", file_path.c_str());
            return {};
        }

        std::vector<geometry_msgs::msg::Pose> poses;
        std::string line;
        while (std::getline(infile, line))
        {
            std::replace(line.begin(), line.end(), ',', ' ');
            std::istringstream iss(line);
            double x, y, yaw, speed;
            if (!(iss >> x >> y >> yaw >> speed))
            {
                // RCLCPP_ERROR(this->get_logger(), "Failed to parse line: %s", line.c_str());
                break;
            }

            geometry_msgs::msg::Pose pose;
            pose.position.x = x - map_x_;
            pose.position.y = y - map_y_;
            pose.position.z = 0.0;

            tf2::Quaternion q;
            q.setRPY(0, 0, yaw * M_PI / 180.0);

            pose.orientation.x = q.x();
            pose.orientation.y = q.y();
            pose.orientation.z = q.z();
            pose.orientation.w = q.w();

            poses.push_back(pose);
            speeds.push_back(static_cast<float>(speed));
        }
        infile.close();
        return poses;
    }

    void timer_callback()
    {
        // if (!odom_position_ || !obstacle_position_)
        if (!odom_position_)
        {
            return;
        }

        if (poses1_.empty())
        {
            poses1_ = file_opener(file_path1_, speeds1_);
        }

        if (poses2_.empty())
        {
            poses2_ = file_opener(file_path2_, speeds2_);
        }

        // double distance = std::sqrt(std::pow(obstacle_position_->position.x - odom_position_->x, 2) + std::pow(obstacle_position_->position.y - odom_position_->y, 2));
        // if (distance > distance_threshold_)
        // {
        //     publish_path(frame_id_, poses1_, path1_pub_);
        //     publish_target_speed(speeds1_[0], speed1_pub_);
        // }
        // else
        // {
        //     publish_path(frame_id_, poses2_, path2_pub_);
        //     publish_target_speed(speeds2_[0], speed2_pub_);
        // }
      
        publish_path(frame_id_, poses1_, path1_pub_);
        publish_target_speed(speeds1_[0], speed1_pub_);
        publish_path(frame_id_, poses2_, path2_pub_);
        publish_target_speed(speeds2_[0], speed2_pub_);

    }

    void publish_path(const std::string &frame_id, const std::vector<geometry_msgs::msg::Pose> &poses, rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pub)
    {
        auto msg = std::make_shared<geometry_msgs::msg::PoseArray>();
        msg->header.stamp = this->now();
        msg->header.frame_id = frame_id;
        msg->poses = poses;
        pub->publish(*msg);
    }

    void publish_target_speed(float speed, rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub)
    {
        auto msg = std::make_shared<std_msgs::msg::Float32>();
        msg->data = speed;
        pub->publish(*msg);
    }

    std::string frame_id_;
    std::string file_path1_;
    std::string file_path2_;

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr obstacle_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path1_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path2_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed1_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed2_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> poses1_;
    std::vector<float> speeds1_;
    std::vector<geometry_msgs::msg::Pose> poses2_;
    std::vector<float> speeds2_;

    std::shared_ptr<geometry_msgs::msg::Pose> obstacle_position_;
    std::shared_ptr<geometry_msgs::msg::Point> odom_position_;

    PJ* P;
    double map_lat_, map_lon_;
    double map_x_, map_y_;
    double distance_threshold_;

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoid>("map", 8.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

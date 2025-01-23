#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/point.hpp>
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
          tf_listener_(tf_buffer_),
          right_path_active_(false),
          left_path_active_(false),
          obstacle_pose_valid_(false)
    {
        this->declare_parameter<std::string>("right_path", "/home/gjs/obstacle_path/right_path.txt");
        this->declare_parameter<std::string>("left_path", "/home/gjs/obstacle_path/left_path.txt");
        this->declare_parameter<std::string>("global_path", "/home/gjs/global_path/global_path.txt");

        right_path_ = this->get_parameter("right_path").as_string();
        left_path_ = this->get_parameter("left_path").as_string();
        global_path_ = this->get_parameter("global_path").as_string();

        obstacle_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>("obstacle_pose_array", 10, std::bind(&ObstacleAvoid::obstacle_callback, this, std::placeholders::_1));
        // odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/localization/kinematic_state", 10, std::bind(&ObstacleAvoid::odom_callback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("odometry/kalman", 10, std::bind(&ObstacleAvoid::odom_callback, this, std::placeholders::_1));
        
        right_path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("right_path/obstacle", 10);
        left_path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("left_path/obstacle", 10);
        path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path/global", 10);
        smooth_path_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("path/smooth", 10);

        right_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_right_speed", 10);
        left_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_left_speed", 10);
        speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_global_speed", 10);
        smooth_speed_pub_ = this->create_publisher<std_msgs::msg::Float32>("target_smooth_speed", 10);

        timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000 / period)), std::bind(&ObstacleAvoid::timer_callback, this));

        odom_position_ = nullptr;

        map_lat_ = 37.4966945; // lat
        map_lon_ = 126.9575076; // lon

        P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr);
        if (P == nullptr)
        {
            throw std::runtime_error("Failed to create projection.");
        }

        PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
        PJ_COORD result = proj_trans(P, PJ_FWD, origin);

        count_ = 0;

        map_y_ = result.xy.x;
        map_x_ = result.xy.y;

        // point1 is obstacle mission start point
        point1_.x = 31.767679441;
        point1_.y = -1.911709253;
        point1_.z = 0.0;

        // point2 is obstacle mission end point
        point2_.x = -12.704760669;
        point2_.y = 3.106722791;
        point2_.z = 0.0;

        obstacle_mission_ = false;
    }

    ~ObstacleAvoid()
    {
        if (P)
        {
            proj_destroy(P);
        }
    }

private:
    void obstacle_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        obstacle_poses_.clear();
        for (const auto &pose : msg->poses)
        {
            // distance_to_car : distance between car and obstacle
            double distance_to_car = std::sqrt(std::pow(pose.position.x - odom_position_->x, 2) + std::pow(pose.position.y - odom_position_->y, 2));
            if (distance_to_car <= 3.0)
            {
                obstacle_poses_.push_back(pose);
            }
        }

        if (!obstacle_poses_.empty())
        {
            double min_distance = std::numeric_limits<double>::max();
            for (const auto &pose : obstacle_poses_)
            {
                double distance_to_car = std::sqrt(std::pow(pose.position.x - odom_position_->x, 2) + std::pow(pose.position.y - odom_position_->y, 2));
                if (distance_to_car < min_distance)
                {
                    min_distance = distance_to_car;
                    obstacle_pose_ = pose;
                    obstacle_pose_valid_ = true;  // Set the flag to true
                }
            }
        }
    }

    void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_position_ = std::make_shared<geometry_msgs::msg::Point>(msg->pose.pose.position);
    }

    std::vector<geometry_msgs::msg::Pose> file_opener(const std::string &file_path, std::vector<float> &speeds)
    {
        std::ifstream infile(file_path);
        if (!infile.is_open())
        {
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

    std::pair<double, int> get_closest_distance(const std::vector<geometry_msgs::msg::Pose> &poses)
    {
        double min_distance = std::numeric_limits<double>::max();
        int closest_index = -1;

        for (int i = 0; i < poses.size(); ++i)
        {
            double distance = std::sqrt(std::pow(poses[i].position.x - odom_position_->x, 2) + std::pow(poses[i].position.y - odom_position_->y, 2));
            if (distance < min_distance)
            {
                min_distance = distance;
                closest_index = i;
            }
        }

        return std::make_pair(min_distance, closest_index);
    }

    std::vector<geometry_msgs::msg::Pose> generate_smooth_path(const geometry_msgs::msg::Pose &start, const geometry_msgs::msg::Pose &goal)
    {
        //bezier curve algorithm
        std::vector<geometry_msgs::msg::Pose> smooth_path;

        int num_points = 30; // Number of points in the smooth path

        // Calculate midpoint
        geometry_msgs::msg::Pose mid;
        mid.position.x = (start.position.x + goal.position.x) / 2;
        mid.position.y = (start.position.y + goal.position.y) / 2;
        mid.position.z = 0.0;

        // Adjust the midpoint to create a curve. Here, we simply add a fixed offset. 
        // You might want to use a more sophisticated method to calculate this offset.
        double offset = 3.0; // You can adjust the offset value to control the curvature
        mid.position.x += offset;

        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;

            geometry_msgs::msg::Pose pose;

            // Cubic Bezier curve formula
            pose.position.x = std::pow(1 - t, 2) * start.position.x + 2 * (1 - t) * t * mid.position.x + std::pow(t, 2) * goal.position.x;
            pose.position.y = std::pow(1 - t, 2) * start.position.y + 2 * (1 - t) * t * mid.position.y + std::pow(t, 2) * goal.position.y;
            pose.position.z = 0.0;

            tf2::Quaternion q_start, q_goal, q_interp;
            tf2::fromMsg(start.orientation, q_start);
            tf2::fromMsg(goal.orientation, q_goal);
            q_interp = q_start.slerp(q_goal, t);

            pose.orientation = tf2::toMsg(q_interp);

            smooth_path.push_back(pose);
        }

        return smooth_path;
    }

    void timer_callback()
    {
        if (!odom_position_)
        {
            return;
        }

        if (right_poses_.empty())
        {
            right_poses_ = file_opener(right_path_, right_speeds_);
        }

        if (left_poses_.empty())
        {
            left_poses_ = file_opener(left_path_, left_speeds_);
        }

        double distance_to_point1 = std::sqrt(std::pow(point1_.x - odom_position_->x, 2) + std::pow(point1_.y - odom_position_->y, 2));
        double distance_to_point2 = std::sqrt(std::pow(point2_.x - odom_position_->x, 2) + std::pow(point2_.y - odom_position_->y, 2));

        if (distance_to_point1 <= 3.0)
        {
            obstacle_mission_ = true;
        }

        if (obstacle_mission_)
        {
            if (obstacle_pose_valid_)
            {
                count_++;
                right_path_active_ = false;
                left_path_active_ = false;

                auto right_path_info = get_closest_distance(right_poses_);
                auto left_path_info = get_closest_distance(left_poses_);

                double distance_to_right_path = right_path_info.first;
                double distance_to_left_path = left_path_info.first;

                if (count_ == 1) //smooth_path는 obstacle이 처음 감지되었을 때 한 번만 계산 / 갱신x
                {
                    if (distance_to_right_path < distance_to_left_path)
                    {
                        int start_index = right_path_info.second + 1;
                        int goal_index = left_path_info.second + 50;

                        geometry_msgs::msg::Pose start_pose = right_poses_[start_index];
                        geometry_msgs::msg::Pose goal_pose = left_poses_[goal_index];

                        smooth_path_ = generate_smooth_path(start_pose, goal_pose);
                    }

                    else
                    {
                        int start_index = left_path_info.second + 1;
                        int goal_index = right_path_info.second + 50;

                        geometry_msgs::msg::Pose start_pose = left_poses_[start_index];
                        geometry_msgs::msg::Pose goal_pose = right_poses_[goal_index];

                        smooth_path_ = generate_smooth_path(start_pose, goal_pose);
                    }    
                }

                std::cout << "smooth path active" << std::endl;
                publish_path(frame_id_, smooth_path_, smooth_path_pub_);
                publish_target_speed(3.0, smooth_speed_pub_);
                auto smooth_path_info = get_closest_distance(smooth_path_);
                if (smooth_path_info.second == smooth_path_.size() - 1) 
                {
                    count_ = 0;
                    obstacle_pose_valid_ = false;
                }
            }

            else
            {
                auto right_path_info = get_closest_distance(right_poses_);
                auto left_path_info = get_closest_distance(left_poses_);

                double distance_to_right_path = right_path_info.first;
                double distance_to_left_path = left_path_info.first;

                if (!right_path_active_ && !left_path_active_)
                {
                    if (distance_to_right_path < distance_to_left_path)
                    {
                        left_path_active_ = false;
                        right_path_active_ = true;
                    }
                    else
                    {
                        right_path_active_ = false;
                        left_path_active_ = true;
                    }
                }

                if (right_path_active_)
                {
                    std::cout << "right path active" << std::endl;
                    publish_path(frame_id_, right_poses_, right_path_pub_);
                    publish_target_speed(right_speeds_[right_path_info.second], right_speed_pub_);
                }
                else
                {
                    std::cout << "left path active" << std::endl;
                    publish_path(frame_id_, left_poses_, left_path_pub_);
                    publish_target_speed(left_speeds_[left_path_info.second], left_speed_pub_);
                }
            }
        }

        else
        {
            std::cout << "global path active" << std::endl;
            if (global_poses_.empty())
            {
                global_poses_ = file_opener(global_path_, global_speeds_);
            }
            auto global_path_info = get_closest_distance(global_poses_);

            publish_path(frame_id_, global_poses_, path_pub_);
            // publish_target_speed(global_speeds_[global_path_info.second], speed_pub_);
            publish_target_speed(2.0, speed_pub_);
        }

        if (distance_to_point2 <= 3.0)
        {
            obstacle_mission_ = false;
        }
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
    std::string right_path_;
    std::string left_path_;
    std::string global_path_;

    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr obstacle_pose_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr right_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr left_path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr path_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr smooth_path_pub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr speed_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr smooth_speed_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<geometry_msgs::msg::Pose> right_poses_;
    std::vector<float> right_speeds_;
    std::vector<geometry_msgs::msg::Pose> left_poses_;
    std::vector<float> left_speeds_;
    std::vector<geometry_msgs::msg::Pose> global_poses_;
    std::vector<float> global_speeds_;

    geometry_msgs::msg::Pose obstacle_pose_;
    bool obstacle_pose_valid_;
    std::shared_ptr<geometry_msgs::msg::Point> odom_position_;

    PJ *P;
    double map_lat_, map_lon_;
    double map_x_, map_y_;
    int count_;

    geometry_msgs::msg::Point point1_;
    geometry_msgs::msg::Point point2_;
    bool obstacle_mission_;
    bool right_path_active_;
    bool left_path_active_;

    

    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    std::vector<geometry_msgs::msg::Pose> obstacle_poses_;
    std::vector<geometry_msgs::msg::Pose> smooth_path_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ObstacleAvoid>("map", 8.0);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

#include "plane_ground_filter.hpp"

bool point_cmp(VPoint a, VPoint b)
{
    return a.z < b.z;
}

PlaneGroundFilter::PlaneGroundFilter()
    : Node("plane_ground_filter")
{
    this->declare_parameter<std::string>("input_topic", "/cropbox_filtered");
    this->declare_parameter<std::string>("no_ground_point_topic", "/points_no_ground");
    this->declare_parameter<std::string>("ground_point_topic", "/points_ground");
    this->declare_parameter<std::string>("all_points_topic", "/all_points");
    this->declare_parameter<double>("clip_height", 4.0);
    this->declare_parameter<double>("sensor_height", 1.77);
    this->declare_parameter<double>("min_distance", 2.0);
    this->declare_parameter<double>("max_distance", 75.0);
    this->declare_parameter<int>("sensor_model", 32);
    this->declare_parameter<int>("num_iter", 3);
    this->declare_parameter<int>("num_lpr", 20);
    this->declare_parameter<double>("th_seeds", 1.2);
    this->declare_parameter<double>("th_dist", 0.3);

    std::string input_topic;
    this->get_parameter("input_topic", input_topic);
    RCLCPP_INFO(this->get_logger(), "input_topic: %s", input_topic.c_str());

    std::string no_ground_point_topic;
    this->get_parameter("no_ground_point_topic", no_ground_point_topic);
    RCLCPP_INFO(this->get_logger(), "no_ground_point_topic: %s", no_ground_point_topic.c_str());

    std::string ground_point_topic;
    this->get_parameter("ground_point_topic", ground_point_topic);
    RCLCPP_INFO(this->get_logger(), "ground_point_topic: %s", ground_point_topic.c_str());

    std::string all_points_topic;
    this->get_parameter("all_points_topic", all_points_topic);
    RCLCPP_INFO(this->get_logger(), "all_points_topic: %s", all_points_topic.c_str());


    this->get_parameter("clip_height", clip_height_);
    RCLCPP_INFO(this->get_logger(), "clip_height: %f", clip_height_);
    this->get_parameter("sensor_height", sensor_height_);
    RCLCPP_INFO(this->get_logger(), "sensor_height: %f", sensor_height_);
    this->get_parameter("min_distance", min_distance_);
    RCLCPP_INFO(this->get_logger(), "min_distance: %f", min_distance_);
    this->get_parameter("max_distance", max_distance_);
    RCLCPP_INFO(this->get_logger(), "max_distance: %f", max_distance_);
    this->get_parameter("sensor_model", sensor_model_);
    RCLCPP_INFO(this->get_logger(), "sensor_model: %d", sensor_model_);
    this->get_parameter("num_iter", num_iter_);
    RCLCPP_INFO(this->get_logger(), "num_iter: %d", num_iter_);
    this->get_parameter("num_lpr", num_lpr_);
    RCLCPP_INFO(this->get_logger(), "num_lpr: %d", num_lpr_);
    this->get_parameter("th_seeds", th_seeds_);
    RCLCPP_INFO(this->get_logger(), "th_seeds: %f", th_seeds_);
    this->get_parameter("th_dist", th_dist_);
    RCLCPP_INFO(this->get_logger(), "th_dist: %f", th_dist_);




    sub_point_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        input_topic, 10, std::bind(&PlaneGroundFilter::point_cb, this, std::placeholders::_1));

    pub_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_ground", 10);
    pub_no_ground_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/points_no_ground", 10);
    pub_all_points_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/all_points", 10);

    g_seeds_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_not_ground_pc = pcl::PointCloud<VPoint>::Ptr(new pcl::PointCloud<VPoint>);
    g_all_pc = pcl::PointCloud<SLRPointXYZIRL>::Ptr(new pcl::PointCloud<SLRPointXYZIRL>);
}

void PlaneGroundFilter::point_cb(const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud)
{
    pcl::PointCloud<VPoint> laserCloudIn;
    pcl::fromROSMsg(*in_cloud, laserCloudIn);

    pcl::PointCloud<VPoint> laserCloudIn_org;
    pcl::fromROSMsg(*in_cloud, laserCloudIn_org);

    SLRPointXYZIRL point;
    for (size_t i = 0; i < laserCloudIn.points.size(); i++)
    {
        point.x = laserCloudIn.points[i].x;
        point.y = laserCloudIn.points[i].y;
        point.z = laserCloudIn.points[i].z;
        point.intensity = laserCloudIn.points[i].intensity;
        point.ring = laserCloudIn.points[i].ring;
        point.label = 0u;
        g_all_pc->points.push_back(point);
    }

    std::sort(laserCloudIn.points.begin(), laserCloudIn.points.end(), point_cmp);

    pcl::PointCloud<VPoint>::iterator it = laserCloudIn.points.begin();
    for (int i = 0; i < laserCloudIn.points.size(); i++)
    {
        if (laserCloudIn.points[i].z < -1.5 * sensor_height_)
        {
            it++;
        }
        else
        {
            break;
        }
    }
    laserCloudIn.points.erase(laserCloudIn.points.begin(), it);

    extract_initial_seeds_(laserCloudIn);
    g_ground_pc = g_seeds_pc;

    for (int i = 0; i < num_iter_; i++)
    {
        estimate_plane_();
        g_ground_pc->clear();
        g_not_ground_pc->clear();

        MatrixXf points(laserCloudIn_org.points.size(), 3);
        int j = 0;
        for (auto p : laserCloudIn_org.points)
        {
            points.row(j++) << p.x, p.y, p.z;
        }

        VectorXf result = points * normal_;

        for (int r = 0; r < result.rows(); r++)
        {
            if (result[r] < th_dist_d_)
            {
                g_all_pc->points[r].label = 1u;
                g_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
            else
            {
                g_all_pc->points[r].label = 0u;
                g_not_ground_pc->points.push_back(laserCloudIn_org[r]);
            }
        }
    }

    pcl::PointCloud<VPoint>::Ptr final_no_ground(new pcl::PointCloud<VPoint>);
    post_process(g_not_ground_pc, final_no_ground);

    sensor_msgs::msg::PointCloud2 ground_msg;
    pcl::toROSMsg(*g_ground_pc, ground_msg);
    ground_msg.header.stamp = in_cloud->header.stamp;
    ground_msg.header.frame_id = in_cloud->header.frame_id;
    pub_ground_->publish(ground_msg);

    sensor_msgs::msg::PointCloud2 groundless_msg;
    pcl::toROSMsg(*final_no_ground, groundless_msg);
    groundless_msg.header.stamp = in_cloud->header.stamp;
    groundless_msg.header.frame_id = in_cloud->header.frame_id;
    pub_no_ground_->publish(groundless_msg);

    sensor_msgs::msg::PointCloud2 all_points_msg;
    pcl::toROSMsg(*g_all_pc, all_points_msg);
    all_points_msg.header.stamp = in_cloud->header.stamp;
    all_points_msg.header.frame_id = in_cloud->header.frame_id;
    pub_all_points_->publish(all_points_msg);

    g_all_pc->clear();
}

void PlaneGroundFilter::clip_above(const pcl::PointCloud<VPoint>::Ptr in,
                                   pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    for (size_t i = 0; i < in->points.size(); i++)
    {
        if (in->points[i].z > clip_height_)
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(std::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
}

void PlaneGroundFilter::remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in,
                                            pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::ExtractIndices<VPoint> cliper;

    cliper.setInputCloud(in);
    pcl::PointIndices indices;
    for (size_t i = 0; i < in->points.size(); i++)
    {
        double distance = sqrt(in->points[i].x * in->points[i].x + in->points[i].y * in->points[i].y);

        if ((distance < min_distance_) || (distance > max_distance_))
        {
            indices.indices.push_back(i);
        }
    }
    cliper.setIndices(std::make_shared<pcl::PointIndices>(indices));
    cliper.setNegative(true);
    cliper.filter(*out);
}

void PlaneGroundFilter::estimate_plane_()
{
    Eigen::Matrix3f cov;
    Eigen::Vector4f pc_mean;
    pcl::computeMeanAndCovarianceMatrix(*g_ground_pc, cov, pc_mean);

    JacobiSVD<MatrixXf> svd(cov, Eigen::DecompositionOptions::ComputeFullU);
    normal_ = (svd.matrixU().col(2));

    Eigen::Vector3f seeds_mean = pc_mean.head<3>();

    d_ = -(normal_.transpose() * seeds_mean)(0, 0);
    th_dist_d_ = th_dist_ - d_;
}

void PlaneGroundFilter::extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted)
{
    double sum = 0;
    int cnt = 0;

    for (int i = 0; i < p_sorted.points.size() && cnt < num_lpr_; i++)
    {
        sum += p_sorted.points[i].z;
        cnt++;
    }
    double lpr_height = cnt != 0 ? sum / cnt : 0;

    g_seeds_pc->clear();
    for (int i = 0; i < p_sorted.points.size(); i++)
    {
        if (p_sorted.points[i].z < lpr_height + th_seeds_)
        {
            g_seeds_pc->points.push_back(p_sorted.points[i]);
        }
    }
}

void PlaneGroundFilter::post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out)
{
    pcl::PointCloud<VPoint>::Ptr cliped_pc_ptr(new pcl::PointCloud<VPoint>);
    clip_above(in, cliped_pc_ptr);
    pcl::PointCloud<VPoint>::Ptr remove_close(new pcl::PointCloud<VPoint>);
    remove_close_far_pt(cliped_pc_ptr, out);
}

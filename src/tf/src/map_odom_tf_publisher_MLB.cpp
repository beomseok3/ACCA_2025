#include <memory>
#include <cmath>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <proj.h>
#include <Eigen/Geometry>

class MapOdomTFPublisherStatic : public rclcpp::Node
{
public:
    MapOdomTFPublisherStatic()
        : Node("map_odom_tf_static_node")
    {
        tf_publisher_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
        gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
            "ublox_gps_node/fix", 10, std::bind(&MapOdomTFPublisherStatic::callback_gps, this, std::placeholders::_1));

        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odometry/navsat", 10, #include <memory>
#include <cmath>
#include <vector>
#include <numeric>
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <proj.h>

class MapOdomTFPublisherStatic : public rclcpp::Node
{
public:
  MapOdomTFPublisherStatic()
  : Node("map_odom_tf_static_node"),
    P_(proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr))
  {
    if (!P_)
      throw std::runtime_error("Failed to create projection");

    /*-------------- 현장 고정 map 앵커 ----------------*/
    // school-bunsudae
    map_lat_ = 37.4963858;
    map_lon_ = 126.95692539999999;

    // kcity-bs
    // map_lat_ = 37.2388925;
    // map_lon_ = 126.77293309999999;

    // kcity-ys
    // map_lat_ = 37.2392369;
    // map_lon_ = 126.77316379999999;


    /*-------------- 파라미터 ----------------*/
    cov_threshold_ = 0.000196;     // 14 mm 1-σ^2
    samples_needed_ = 100;         // 평균에 쓸 Fix 개수

    /*-------------- 미리 map 앵커를 투영 ----------------*/
    PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
    PJ_COORD res    = proj_trans(P_, PJ_FWD, origin);
    map_x_ = res.xy.x;
    map_y_ = res.xy.y;

    /*-------------- ROS 통신 ----------------*/
    tf_pub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    gps_sub_ = create_subscription<sensor_msgs::msg::NavSatFix>(
      "ublox_gps_node/fix", 10,
      std::bind(&MapOdomTFPublisherStatic::gpsCallback, this, std::placeholders::_1));

    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>(
      "odometry/navsat", 10,
      std::bind(&MapOdomTFPublisherStatic::odomCallback, this, std::placeholders::_1));
  }

  ~MapOdomTFPublisherStatic() override
  {
    if (P_) proj_destroy(P_);
  }

  /* 매 루프 TF 전송 */
  void publishTF()
  {
    if (!odom_ || !transform_ready_) return;

    geometry_msgs::msg::TransformStamped tf;
    tf.header.stamp    = odom_->header.stamp;
    tf.header.frame_id = "map";
    tf.child_frame_id  = "odom";
    tf.transform.translation.x = dy_;   // 2097: x=Northing, y=Easting
    tf.transform.translation.y = dx_;
    tf.transform.translation.z = 0.0;
    tf.transform.rotation.w    = 1.0;   // 회전 0
    tf_pub_->sendTransform(tf);
  }

private:
  /*---------------------- GPS ----------------------*/
  void gpsCallback(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
  {
    /* 1) 정확도 필터 */
    if (msg->position_covariance[0] > cov_threshold_
     || msg->position_covariance[4] > cov_threshold_)
      return;

    /* 2) 샘플 버퍼에 push */
    lat_buf_.push_back(msg->latitude);
    lon_buf_.push_back(msg->longitude);

    RCLCPP_INFO(get_logger(), "sample_num: %zu", lat_buf_.size());  // 추천

    /* 3) 100 개 쌓이면 평균 → dx,dy 계산 */
    if (!transform_ready_ && lat_buf_.size() >= samples_needed_) {
      double lat_mean = std::accumulate(lat_buf_.begin(), lat_buf_.end(), 0.0) / lat_buf_.size();
      double lon_mean = std::accumulate(lon_buf_.begin(), lon_buf_.end(), 0.0) / lon_buf_.size();

      PJ_COORD ll  = proj_coord(lat_mean, lon_mean, 0, 0);
      PJ_COORD xy  = proj_trans(P_, PJ_FWD, ll);

      dx_ = xy.xy.x - map_x_;
      dy_ = xy.xy.y - map_y_;

      transform_ready_ = true;

      RCLCPP_INFO(get_logger(),
        "Averaged %zu fixes → map→odom: dx=%.3f m, dy=%.3f m",
        lat_buf_.size(), dx_, dy_);
    }
  }

  /*---------------------- Odometry ----------------------*/
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    odom_ = msg;
  }

  /* ------------ members ------------ */
  /* ROS */
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr     odom_sub_;
  std::shared_ptr<tf2_ros::TransformBroadcaster>               tf_pub_;
  nav_msgs::msg::Odometry::SharedPtr                           odom_;

  /* PROJ */
  PJ* P_;

  /* map 앵커(고정) */
  double map_lat_, map_lon_, map_x_, map_y_;

  /* 평균 샘플 버퍼 */
  std::vector<double> lat_buf_, lon_buf_;

  /* 설정값 */
  double cov_threshold_;
  std::size_t samples_needed_;

  /* 결과 변환 */
  double dx_{0.0}, dy_{0.0};
  bool   transform_ready_{false};
};

int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MapOdomTFPublisherStatic>();
  rclcpp::Rate r(10);

  while (rclcpp::ok()) {
    rclcpp::spin_some(node);
    node->publishTF();
    r.sleep();
  }
  rclcpp::shutdown();
  return 0;
}
std::bind(&MapOdomTFPublisherStatic::callback_odom, this, std::placeholders::_1));

        dx_ = 0.0;
        dy_ = 0.0;

        // 기준점 (K-City BS)
        map_lat_ = 37.2388925;
        map_lon_ = 126.77293309999999;

        threshold_ = 0.05;

        // ✅ Molodensky-Badekas 변환 설정 (Proj7 이상)
        const char *helmert_def = 
            "+proj=helmert +x=-115.8 +y=-54.3 +z=-79.6 "
            "+rx=-0.000001004 +ry=-0.000001887 +rz=-0.000005103 "
            "+s=-0.00000001116 "
            "+convention=coordinate_frame";

        P = proj_create_crs_to_crs(PJ_DEFAULT_CTX, "EPSG:4326", "EPSG:2097", nullptr);
        if (P == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create projection.");
            throw std::runtime_error("Failed to create projection.");
        }

        P_helmert = proj_create(PJ_DEFAULT_CTX, helmert_def);
        if (P_helmert == nullptr)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to create Helmert transformation.");
            throw std::runtime_error("Failed to create Helmert transformation.");
        }

        PJ_COORD origin = proj_coord(map_lat_, map_lon_, 0, 0);
        PJ COORD transformed = proj_trans(P_helmert, PJ_FWD, origin);
        PJ_COORD result = proj_trans(P, PJ_FWD, transformed);
        map_x_ = result.xy.x;
        map_y_ = result.xy.y;
    }

    ~MapOdomTFPublisherStatic()origin
    {
        if (P)
        {
            proj_destroy(P);
        }
        if (P_helmert)
        {
            proj_destroy(P_helmert);
        }
    }

    void publish_tf()
    {
        if (!odom_)
            return;

        geometry_msgs::msg::TransformStamped tf_msg;

        tf_msg.header.frame_id = "map";
        tf_msg.header.stamp = odom_->header.stamp;
        tf_msg.child_frame_id = "odom";

        double trans_x = dx_;
        double trans_y = dy_;

        tf_msg.transform.translation.x = trans_y;
        tf_msg.transform.translation.y = trans_x;
        tf_msg.transform.translation.z = 0.0;

        tf_msg.transform.rotation.x = 0.;
        tf_msg.transform.rotation.y = 0.;
        tf_msg.transform.rotation.z = 0.;
        tf_msg.transform.rotation.w = 1.;

        tf_publisher_->sendTransform(tf_msg);
    }

private:
    void callback_gps(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
    {
        if (dx_ == 0.0 && dy_ == 0.0)
        {
            if (msg->position_covariance[0] < threshold_ && msg->position_covariance[4] < threshold_)
            {
                PJ_COORD latlon = proj_coord(msg->latitude, msg->longitude, 0, 0);
                
                // ✅ Molodensky-Badekas 변환 적용
                PJ_COORD transformed = proj_trans(P_helmert, PJ_FWD, latlon);
                PJ_COORD xy = proj_trans(P, PJ_FWD, transformed);
                
                dx_ = xy.xy.x - map_x_;
                dy_ = xy.xy.y - map_y_;
                
                RCLCPP_INFO(this->get_logger(), "Map origin: x = %f, y = %f", map_x_, map_y_);
                RCLCPP_INFO(this->get_logger(), "odom origin: x = %f, y = %f", xy.xy.x, xy.xy.y);
                RCLCPP_INFO(this->get_logger(), "Origin calculated: dx = %f, dy = %f", dx_, dy_);
            }
        }
    }

    void callback_odom(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        odom_ = msg;
    }

    rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_publisher_;
    nav_msgs::msg::Odometry::SharedPtr odom_;

    PJ *P;
    PJ *P_helmert;  // Molodensky-Badekas 변환용
    double map_lat_, map_lon_;
    double map_x_, map_y_;
    double dx_, dy_;
    double threshold_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapOdomTFPublisherStatic>();

    rclcpp::Rate rate(10);
    while (rclcpp::ok())
    {
        rclcpp::spin_some(node);
        node->publish_tf();
        rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}

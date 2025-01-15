#pragma once

#include <rclcpp/rclcpp.hpp>

#define PCL_NO_PRECOMPILE

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/common/centroid.h>
#include <pcl/filters/extract_indices.h>

#include <Eigen/Dense>

#include <sensor_msgs/msg/point_cloud2.hpp>

namespace velodyne_pointcloud
{
/** Euclidean Velodyne coordinate, including intensity and ring number. */
struct PointXYZIR
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace velodyne_pointcloud

POINT_CLOUD_REGISTER_POINT_STRUCT(velodyne_pointcloud::PointXYZIR,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring))

namespace plane_ground_filter
{
/** Euclidean Velodyne coordinate, including intensity and ring number, and label. */
struct PointXYZIRL
{
  PCL_ADD_POINT4D;                // quad-word XYZ
  float intensity;                ///< laser intensity reading
  uint16_t ring;                  ///< laser ring number
  uint16_t label;                 ///< point label
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW // ensure proper alignment
} EIGEN_ALIGN16;

}; // namespace plane_ground_filter

#define SLRPointXYZIRL plane_ground_filter::PointXYZIRL
#define VPoint velodyne_pointcloud::PointXYZIR
#define RUN pcl::PointCloud<SLRPointXYZIRL>

POINT_CLOUD_REGISTER_POINT_STRUCT(plane_ground_filter::PointXYZIRL,
                                  (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)(uint16_t, ring, ring)(uint16_t, label, label))

using Eigen::JacobiSVD;
using Eigen::MatrixXf;
using Eigen::VectorXf;

class PlaneGroundFilter : public rclcpp::Node
{
public:
  PlaneGroundFilter();

private:
  void point_cb(const sensor_msgs::msg::PointCloud2::SharedPtr in_cloud);
  void clip_above(const pcl::PointCloud<VPoint>::Ptr in, pcl::PointCloud<VPoint>::Ptr out);
  void remove_close_far_pt(const pcl::PointCloud<VPoint>::Ptr in, pcl::PointCloud<VPoint>::Ptr out);
  void estimate_plane_();
  void extract_initial_seeds_(const pcl::PointCloud<VPoint> &p_sorted);
  void post_process(const pcl::PointCloud<VPoint>::Ptr in, const pcl::PointCloud<VPoint>::Ptr out);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_point_cloud_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_no_ground_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_all_points_;

  int sensor_model_;
  double sensor_height_, clip_height_, min_distance_, max_distance_;
  int num_seg_ = 1;
  int num_iter_, num_lpr_;
  double th_seeds_, th_dist_;
  float d_, th_dist_d_;
  MatrixXf normal_;

  pcl::PointCloud<VPoint>::Ptr g_seeds_pc;
  pcl::PointCloud<VPoint>::Ptr g_ground_pc;
  pcl::PointCloud<VPoint>::Ptr g_not_ground_pc;
  pcl::PointCloud<SLRPointXYZIRL>::Ptr g_all_pc;
};
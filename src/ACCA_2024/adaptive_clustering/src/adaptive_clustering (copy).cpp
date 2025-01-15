// Copyright (C) 2018  Zhi Yan and Li Sun

// This program is free software: you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by the Free
// Software Foundation, either version 3 of the License, or (at your option)
// any later version.

// This program is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
// FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
// more details.

// You should have received a copy of the GNU General Public License along
// with this program.  If not, see <http://www.gnu.org/licenses/>.

// ROS 2
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include "adaptive_clustering_msgs/msg/cluster_array.hpp"


// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

//#define LOG

const int region_max_ = 10; // Change this value to match how far you want to detect.


class AdaptiveClustering : public rclcpp::Node {
public:
  AdaptiveClustering()
    : Node("adaptive_clustering"),
      print_fps_(false),
      leaf_(1),
      z_axis_min_(-1.2),
      z_axis_max_(1.0),
      cluster_size_min_(20),
      cluster_size_max_(100),
      x_threshold_(0.5),
      y_threshold_(0.5),
      cone_position_z_(-0.6),
      frames_(0),
      reset_(true) {
    
    this->declare_parameter<std::string>("sensor_model", "VLP-32");
    this->declare_parameter<bool>("print_fps", false);
    this->declare_parameter<int>("leaf", 1);
    this->declare_parameter<float>("z_axis_min", -0.8);
    this->declare_parameter<float>("z_axis_max", 2.0);
    this->declare_parameter<float>("x_threshold", 0.5);
    this->declare_parameter<float>("y_threshold", 0.5);
    this->declare_parameter<float>("cone_position_z", -0.6);
    this->declare_parameter<int>("cluster_size_min", 3);
    this->declare_parameter<int>("cluster_size_max", 2200000);
    
    this->get_parameter("sensor_model", sensor_model_);
    this->get_parameter("print_fps", print_fps_);
    this->get_parameter("leaf", leaf_);
    this->get_parameter("z_axis_min", z_axis_min_);
    this->get_parameter("z_axis_max", z_axis_max_);
    this->get_parameter("x_threshold", x_threshold_);
    this->get_parameter("y_threshold", y_threshold_);
    this->get_parameter("cone_position_z", cone_position_z_);
    this->get_parameter("cluster_size_min", cluster_size_min_);
    this->get_parameter("cluster_size_max", cluster_size_max_);
    
    cluster_array_pub_ = this->create_publisher<adaptive_clustering_msgs::msg::ClusterArray>("clusters", 100);
    cloud_filtered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("cloud_filtered", 100);
    pose_array_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>("cone_poses", 100);
    marker_array_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("markers", 100);
    
    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_no_ground", 1, std::bind(&AdaptiveClustering::pointCloudCallback, this, std::placeholders::_1));
    
    if (sensor_model_ == "VLP-16") {
      regions_ = {2, 3, 3, 3, 3, 3, 3, 2, 3, 3, 3, 3, 3, 3};
    } 
    else if (sensor_model_ == "HDL-32E") {
      regions_ = {4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5};
    } 
    else if (sensor_model_ == "HDL-64E") {
      regions_ = {14, 14, 14, 15, 14};
    } 
    else if (sensor_model_ == "VLP-32") {
      regions_ = {4, 5, 4, 5, 4, 5, 5, 4, 5, 4, 5, 5, 4, 5};
    }
    else {
      RCLCPP_FATAL(this->get_logger(), "Unknown sensor model!");
      rclcpp::shutdown();
    }
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstPtr& ros_pc2_in) {
    if (print_fps_) {
      if (reset_) {
        frames_ = 0;
        start_time_ = clock();
        reset_ = false;
      }
    }
    
    /*** Convert ROS message to PCL ***/
    pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_in(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*ros_pc2_in, *pcl_pc_in);
    
    /*** Downsampling + ground & ceiling removal ***/
    pcl::IndicesPtr pc_indices(new std::vector<int>);
    for (size_t i = 0; i < pcl_pc_in->size(); ++i) {
      if (i % leaf_ == 0) {
        if (pcl_pc_in->points[i].z >= z_axis_min_ && pcl_pc_in->points[i].z <= z_axis_max_) {
          pc_indices->push_back(i);
        }
      }
    }
    
    /*** Divide the point cloud into nested circular regions ***/
    std::array<std::vector<int>, region_max_> indices_array;
    for (size_t i = 0; i < pc_indices->size(); i++) {
      float range = 0.0;
      for (int j = 0; j < region_max_; j++) {
        float d2 = pcl_pc_in->points[(*pc_indices)[i]].x * pcl_pc_in->points[(*pc_indices)[i]].x +
                   pcl_pc_in->points[(*pc_indices)[i]].y * pcl_pc_in->points[(*pc_indices)[i]].y +
                   pcl_pc_in->points[(*pc_indices)[i]].z * pcl_pc_in->points[(*pc_indices)[i]].z;
        if (d2 > range * range && d2 <= (range + regions_[j]) * (range + regions_[j])) {
          indices_array[j].push_back((*pc_indices)[i]);
          break;
        }
        range += regions_[j];
      }
    }
    
    /*** Euclidean clustering ***/
    float tolerance = 0.0;
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<pcl::PointXYZI>::Ptr>> clusters;
    
    for (int i = 0; i < region_max_; i++) {
      tolerance += 0.1;
      if (indices_array[i].size() > cluster_size_min_) {
        std::shared_ptr<const std::vector<int>> indices_array_ptr = std::make_shared<const std::vector<int>>(indices_array[i]);
        pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
        tree->setInputCloud(pcl_pc_in, indices_array_ptr);
        
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
        ec.setClusterTolerance(tolerance);
        ec.setMinClusterSize(cluster_size_min_);
        ec.setMaxClusterSize(cluster_size_max_);
        ec.setSearchMethod(tree);
        ec.setInputCloud(pcl_pc_in);
        ec.setIndices(indices_array_ptr);
        ec.extract(cluster_indices);
        
        for (const auto& it : cluster_indices) {
          pcl::PointCloud<pcl::PointXYZI>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZI>);
          for (const auto& pit : it.indices) {
            cluster->points.push_back(pcl_pc_in->points[pit]);
          }
          cluster->width = cluster->size();
          cluster->height = 1;
          cluster->is_dense = true;
          clusters.push_back(cluster);
        }
      }
    }
    
    /*** Output ***/
    if (cloud_filtered_pub_->get_subscription_count() > 0) {
      pcl::PointCloud<pcl::PointXYZI>::Ptr pcl_pc_out(new pcl::PointCloud<pcl::PointXYZI>);
      sensor_msgs::msg::PointCloud2 ros_pc2_out;
      pcl::copyPointCloud(*pcl_pc_in, *pc_indices, *pcl_pc_out);
      pcl::toROSMsg(*pcl_pc_out, ros_pc2_out);
      cloud_filtered_pub_->publish(ros_pc2_out);
    }
    
    adaptive_clustering_msgs::msg::ClusterArray cluster_array;
    geometry_msgs::msg::PoseArray pose_array;
    visualization_msgs::msg::MarkerArray marker_array;
    
    for (size_t i = 0; i < clusters.size(); i++) {
      if (cluster_array_pub_->get_subscription_count() > 0) {
        sensor_msgs::msg::PointCloud2 ros_pc2_out;
        pcl::toROSMsg(*clusters[i], ros_pc2_out);
        cluster_array.clusters.push_back(ros_pc2_out);
      }
      
      if (pose_array_pub_->get_subscription_count() > 0) {
        std::vector<Eigen::Vector4f> centroids;
        for (const auto& cluster : clusters) {
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cluster, centroid);
            centroids.push_back(centroid);
        }

        std::vector<bool> used(clusters.size(), false);

        for (size_t i = 0; i < clusters.size(); ++i) {
            if (used[i]) continue;
            used[i] = true;

            Eigen::Vector4f avg_centroid = centroids[i];
            int count = 1;

            for (size_t j = i + 1; j < clusters.size(); ++j) {
                if (used[j]) continue;
                if (std::abs(centroids[i][0] - centroids[j][0]) <= x_threshold_ &&
                    std::abs(centroids[i][1] - centroids[j][1]) <= y_threshold_) {
                    avg_centroid += centroids[j];
                    count++;
                    used[j] = true;
                }
            }

            avg_centroid /= count;

            geometry_msgs::msg::Pose pose;
            pose.position.x = avg_centroid[0];
            pose.position.y = avg_centroid[1];
            pose.position.z = cone_position_z_;
            pose.orientation.w = 1;
            pose_array.poses.push_back(pose);
        }
        
#ifdef LOG
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*clusters[i], min, max);
        std::cerr << ros_pc2_in->header().frame_id << " "
                  << ros_pc2_in->header().stamp.sec << " "
                  << min[0] << " "
                  << min[1] << " "
                  << min[2] << " "
                  << max[0] << " "
                  << max[1] << " "
                  << max[2] << " "
                  << std::endl;
#endif
      }
      
      if (marker_array_pub_->get_subscription_count() > 0) {
        Eigen::Vector4f min, max;
        pcl::getMinMax3D(*clusters[i], min, max);
        
        visualization_msgs::msg::Marker marker;
        marker.header = ros_pc2_in->header;
        marker.ns = "adaptive_clustering";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_LIST;
        
        geometry_msgs::msg::Point p[24];
        p[0].x = max[0];  p[0].y = max[1];  p[0].z = max[2];
        p[1].x = min[0];  p[1].y = max[1];  p[1].z = max[2];
        p[2].x = max[0];  p[2].y = max[1];  p[2].z = max[2];
        p[3].x = max[0];  p[3].y = min[1];  p[3].z = max[2];
        p[4].x = max[0];  p[4].y = max[1];  p[4].z = max[2];
        p[5].x = max[0];  p[5].y = max[1];  p[5].z = min[2];
        p[6].x = min[0];  p[6].y = min[1];  p[6].z = min[2];
        p[7].x = max[0];  p[7].y = min[1];  p[7].z = min[2];
        p[8].x = min[0];  p[8].y = min[1];  p[8].z = min[2];
        p[9].x = min[0];  p[9].y = max[1];  p[9].z = min[2];
        p[10].x = min[0]; p[10].y = min[1]; p[10].z = min[2];
        p[11].x = min[0]; p[11].y = min[1]; p[11].z = max[2];
        p[12].x = min[0]; p[12].y = max[1]; p[12].z = max[2];
        p[13].x = min[0]; p[13].y = max[1]; p[13].z = min[2];
        p[14].x = min[0]; p[14].y = max[1]; p[14].z = max[2];
        p[15].x = min[0]; p[15].y = min[1]; p[15].z = max[2];
        p[16].x = max[0]; p[16].y = min[1]; p[16].z = max[2];
        p[17].x = max[0]; p[17].y = min[1]; p[17].z = min[2];
        p[18].x = max[0]; p[18].y = min[1]; p[18].z = max[2];
        p[19].x = min[0]; p[19].y = min[1]; p[19].z = max[2];
        p[20].x = max[0]; p[20].y = max[1]; p[20].z = min[2];
        p[21].x = min[0]; p[21].y = max[1]; p[21].z = min[2];
        p[22].x = max[0]; p[22].y = max[1]; p[22].z = min[2];
        p[23].x = max[0]; p[23].y = min[1]; p[23].z = min[2];
        for (int i = 0; i < 24; i++) {
          marker.points.push_back(p[i]);
        }
        
        marker.scale.x = 0.02;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.5;
        marker.lifetime = rclcpp::Duration(0, 300000000);
        marker_array.markers.push_back(marker);
      }
    }
    
    if (!cluster_array.clusters.empty()) {
      cluster_array.header = ros_pc2_in->header;
      cluster_array_pub_->publish(cluster_array);
    }
    
    if (!pose_array.poses.empty()) {
      pose_array.header = ros_pc2_in->header;
      pose_array_pub_->publish(pose_array);
    }
    
    if (!marker_array.markers.empty()) {
      marker_array_pub_->publish(marker_array);
    }
    
    if (print_fps_) {
      if (++frames_ > 10) {
        RCLCPP_INFO(this->get_logger(), "[adaptive_clustering] fps = %f, timestamp = %ld",
                    float(frames_) / (float(clock() - start_time_) / CLOCKS_PER_SEC), clock() / CLOCKS_PER_SEC);
        reset_ = true;
      }
    }
  }
  
  rclcpp::Publisher<adaptive_clustering_msgs::msg::ClusterArray>::SharedPtr cluster_array_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_filtered_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_array_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_pub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  
  bool print_fps_;
  int leaf_;
  float z_axis_min_;
  float z_axis_max_;
  float x_threshold_;
  float y_threshold_;
  float cone_position_z_;
  int cluster_size_min_;
  int cluster_size_max_;
  
  std::vector<int> regions_;
  
  int frames_;
  clock_t start_time_;
  bool reset_;
  std::string sensor_model_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AdaptiveClustering>());
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>   // ★ 추가: 다운샘플링용

class PointCloudVisualizer : public rclcpp::Node
{
public:
  explicit PointCloudVisualizer(const std::string &pcd_file_path,
                                float voxel_size = 1.0f)          // ★ leaf size 기본값 10 cm
  : Node("pointcloud_visualizer")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                   "pointcloud", rclcpp::QoS{10}.transient_local());

    /* 1. PCD 읽기 --------------------------------------------------------- */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_file_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s",
                   pcd_file_path.c_str());
      return;
    }

    /* 2. VoxelGrid 다운샘플링 -------------------------------------------- */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_ds(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> voxel;
    voxel.setInputCloud(cloud);
    voxel.setLeafSize(voxel_size, voxel_size, voxel_size);  // (X,Y,Z) leaf 크기
    voxel.filter(*cloud_ds);

    /* 3. ROS 메시지 변환 및 발행 ------------------------------------------ */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_ds, output);
    output.header.frame_id = "map";
    publisher_->publish(output);

    RCLCPP_INFO(this->get_logger(),
                "Published down-sampled PointCloud2 (%zu → %zu points, leaf=%.2f m) from: %s",
                cloud->size(), cloud_ds->size(), voxel_size, pcd_file_path.c_str());
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <path_to_pcd_file> [voxel_size_m]" << std::endl;
    return 1;
  }

  std::string pcd_file_path = argv[1];
  float voxel_size = (argc >= 3) ? std::stof(argv[2]) : 0.5f;  // 옵션: leaf 사이즈 지정

  rclcpp::spin(std::make_shared<PointCloudVisualizer>(pcd_file_path, voxel_size));
  rclcpp::shutdown();
  return 0;
}

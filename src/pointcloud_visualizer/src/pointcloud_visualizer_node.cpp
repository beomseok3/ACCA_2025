#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>   // ★ 추가: 다운샘플링용

class PointCloudVisualizer : public rclcpp::Node
{
public:
<<<<<<< HEAD
  PointCloudVisualizer(const std::string &pcd_path)
=======
  explicit PointCloudVisualizer(const std::string &pcd_file_path,
                                float voxel_size = 1.0f)          // ★ leaf size 기본값 10 cm
>>>>>>> 7cb2f3f7415bf105aa3505aa0c79254747affe06
  : Node("pointcloud_visualizer")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
                   "pointcloud", rclcpp::QoS{10}.transient_local());

    /* 1. PCD 읽기 --------------------------------------------------------- */
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
<<<<<<< HEAD
    if (pcl::io::loadPCDFile<pcl::PointXYZ>(pcd_path, *cloud) == -1) {
      RCLCPP_ERROR(this->get_logger(), "Couldn't read PCD file: %s", pcd_path.c_str());
      return;
    }

    // Voxel Grid Downsampling
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(0.5f, 0.5f, 0.5f); // leaf size 설정 (0.1m x 0.1m x 0.1m)
    sor.filter(*cloud_filtered);
=======
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
>>>>>>> 7cb2f3f7415bf105aa3505aa0c79254747affe06

    /* 3. ROS 메시지 변환 및 발행 ------------------------------------------ */
    sensor_msgs::msg::PointCloud2 output;
    pcl::toROSMsg(*cloud_ds, output);
    output.header.frame_id = "map";
    publisher_->publish(output);

<<<<<<< HEAD
    RCLCPP_INFO(this->get_logger(), "Published downsampled PointCloud2 from: %s", pcd_path.c_str());
=======
    RCLCPP_INFO(this->get_logger(),
                "Published down-sampled PointCloud2 (%zu → %zu points, leaf=%.2f m) from: %s",
                cloud->size(), cloud_ds->size(), voxel_size, pcd_file_path.c_str());
>>>>>>> 7cb2f3f7415bf105aa3505aa0c79254747affe06
  }

private:
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  if (argc < 2) {
<<<<<<< HEAD
    std::cerr << "Usage: ros2 run <package> <executable> <pcd_file_path>\n";
    return 1;
  }

  std::string pcd_path = argv[1];

  rclcpp::spin(std::make_shared<PointCloudVisualizer>(pcd_path));
=======
    std::cerr << "Usage: " << argv[0] << " <path_to_pcd_file> [voxel_size_m]" << std::endl;
    return 1;
  }

  std::string pcd_file_path = argv[1];
  float voxel_size = (argc >= 3) ? std::stof(argv[2]) : 0.5f;  // 옵션: leaf 사이즈 지정

  rclcpp::spin(std::make_shared<PointCloudVisualizer>(pcd_file_path, voxel_size));
>>>>>>> 7cb2f3f7415bf105aa3505aa0c79254747affe06
  rclcpp::shutdown();
  return 0;
}

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  char path[512];
  std::string package_share_directory = ament_index_cpp::get_package_share_directory("moveit_tutorials");
  std::string RESOURCE_DIR = package_share_directory + "/data";
  printf("package_share_directory: %s\n", package_share_directory.c_str());
  snprintf(path, sizeof(path), "%s/random_pick.pcd", RESOURCE_DIR.c_str());
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  if (pcl::io::loadPCDFile<pcl::PointXYZRGBA>(path, cloud)) {
    return -1;
  }
  sensor_msgs::msg::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.frame_id = "camera_rgb_optical_frame";
  auto pcd_node = rclcpp::Node::make_shared("PCDPublisher");
  auto pcd_pub = pcd_node->create_publisher<sensor_msgs::msg::PointCloud2>(
    "/camera/depth_registered/points", 10);
  rclcpp::Rate loop_rate(30);
  while (rclcpp::ok()) {
    msg.header.stamp = pcd_node->now();
    pcd_pub->publish(msg);
    loop_rate.sleep();
  }

  rclcpp::shutdown();
  return 0;
}
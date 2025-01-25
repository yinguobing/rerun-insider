#include <memory>
#include <rerun.hpp>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class Insider : public rclcpp::Node {
public:
  Insider(std::string node_name);
  ~Insider();

private:
  // Callback for lidar points
  void callback_lidar(
      const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) const;

  // Callback for camera images
  void
  callback_camera(const sensor_msgs::msg::Image::ConstSharedPtr &msg) const;

  // Callback for odometery data
  void callback_odom(const nav_msgs::msg::Odometry::ConstSharedPtr &msg) const;

  // Subcription of lidar point cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;

  // Subcription of camera images
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr sub_cam;

  // Subcription of odometery data
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom;

  // Rerun recording stream
  rerun::RecordingStream *rec;
};

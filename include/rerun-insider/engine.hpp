#include <memory>
#include <rerun.hpp>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
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
  void callback_lidar(const sensor_msgs::msg::PointCloud2 &msg) const;

  // Subcription of lidar point cloud
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_lidar;

  // Rerun recording stream
  rerun::RecordingStream *rec;
};

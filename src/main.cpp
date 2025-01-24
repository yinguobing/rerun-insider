#include <memory>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"

using std::placeholders::_1;

class RosInsider : public rclcpp::Node {
public:
  RosInsider() : Node("ros_insider") {
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud_hasco_4lidars", rclcpp::SensorDataQoS(),
        std::bind(&RosInsider::lidar_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Listening...");
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2 &msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%u x %u'", msg.height,
                msg.width);
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosInsider>());
  rclcpp::shutdown();
  return 0;
}
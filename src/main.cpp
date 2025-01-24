#include <memory>
#include <rerun.hpp>

#include "rclcpp/qos.hpp"
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

class RosInsider : public rclcpp::Node {
public:
  RosInsider() : Node("ros_insider") {
    // Init rerun client
    rec = new rerun::RecordingStream("rerun-insider");
    // Try to spawn a new viewer instance.
    rec->spawn().exit_on_failure();

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/point_cloud_hasco_4lidars", rclcpp::SensorDataQoS(),
        std::bind(&RosInsider::lidar_callback, this, _1));
    RCLCPP_INFO(this->get_logger(), "Initialized");
  }

private:
  void lidar_callback(const sensor_msgs::msg::PointCloud2 &msg) const {
    std::vector<rerun::Position3D> points;
    std::vector<rerun::Color> colors;

    sensor_msgs::PointCloud2ConstIterator<int16_t> iter_x(msg, "x");
    const size_t number_of_points = msg.height * msg.width;
    for (size_t i = 0; i < number_of_points; ++i, ++iter_x) {
      float x = iter_x[0] / 100.0;
      float y = iter_x[1] / 100.0;
      float z = iter_x[2] / 100.0;
      uint8_t intensity = (uint8_t)iter_x[3];
      points.push_back(rerun::Position3D{x, y, z});
      colors.push_back(rerun::Color{255, 255, 255});
    }

    // Log the "my_points" entity with our data, using the `Points3D`
    rec->log_static(
        "my_points",
        rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));
  }
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
  rerun::RecordingStream *rec;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RosInsider>());
  rclcpp::shutdown();
  return 0;
}
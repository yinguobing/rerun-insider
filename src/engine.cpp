#include "rerun-insider/engine.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
#include <chrono>

using namespace std::chrono_literals;

Insider::Insider(const std::string &node_name,
                 const rclcpp::NodeOptions &options)
    : rclcpp::Node(node_name, options) {
  auto logger = this->get_logger();

  // Rerun viewer address
  this->declare_parameter("rerun_viewer_addr", "");

  // Subscription topic lidar
  this->declare_parameter("subscription_topic_lidar", "");

  // Subscription topic camera
  this->declare_parameter("subscription_topic_camera", "");

  // Subscription topic odometry
  this->declare_parameter("subscription_topic_odometry", "");

  // Rerun viewer address
  auto rerun_viewer_addr = this->get_parameter("rerun_viewer_addr")
                               .get_parameter_value()
                               .get<std::string>();
  RCLCPP_INFO(logger, "Rerun viewer address: %s ", rerun_viewer_addr.c_str());

  // Create callback groups
  auto callback_group_cam =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);
  auto callback_group_lidar =
      create_callback_group(rclcpp::CallbackGroupType::Reentrant);

  // Subscription topic lidar
  auto subscription_topic_lidar =
      this->get_parameter("subscription_topic_lidar")
          .get_parameter_value()
          .get<std::string>();
  RCLCPP_INFO(logger, "Lidar topic: %s ", subscription_topic_lidar.c_str());

  // Subscription topic camera
  auto subscription_topic_camera =
      this->get_parameter("subscription_topic_camera")
          .get_parameter_value()
          .get<std::string>();
  RCLCPP_INFO(logger, "Camera topic: %s ", subscription_topic_camera.c_str());

  // Subscription topic odometry
  auto subscription_topic_odometry =
      this->get_parameter("subscription_topic_odometry")
          .get_parameter_value()
          .get<std::string>();
  RCLCPP_INFO(logger, "Odometry topic: %s ",
              subscription_topic_odometry.c_str());

  // Init rerun client, try to connect to a viewer instance.
  this->rec = new rerun::RecordingStream("rerun-insider");
  this->rec->connect_tcp(rerun_viewer_addr, 0.1).exit_on_failure();

  // Init subscriptions
  rclcpp::SubscriptionOptions options_sub_lidar;
  options_sub_lidar.callback_group = callback_group_lidar;
  sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      subscription_topic_lidar, rclcpp::SensorDataQoS(),
      std::bind(&Insider::callback_lidar, this, _1), options_sub_lidar);

  rclcpp::SubscriptionOptions options_sub_cam;
  options_sub_cam.callback_group = callback_group_cam;
  sub_cam = this->create_subscription<sensor_msgs::msg::Image>(
      subscription_topic_camera, rclcpp::SensorDataQoS(),
      std::bind(&Insider::callback_camera, this, _1), options_sub_cam);

  sub_odom = this->create_subscription<nav_msgs::msg::Odometry>(
      subscription_topic_odometry, rclcpp::SensorDataQoS(),
      std::bind(&Insider::callback_odom, this, _1));

  // Callback of timer
  this->timer =
      this->create_wall_timer(50ms, std::bind(&Insider::callback_timer, this));

  RCLCPP_INFO(this->get_logger(), "Initialized");
}

Insider::~Insider() {}

void Insider::callback_lidar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) const {
  std::vector<rerun::Position3D> points;
  std::vector<rerun::Color> colors;

  int num_points = msg->width * msg->height;
  points.reserve(num_points);
  colors.reserve(num_points);

  // Populate the point cloud data.
  sensor_msgs::PointCloud2ConstIterator<int16_t> iter(*msg, "x");
  const size_t number_of_points = msg->height * msg->width;
  for (size_t i = 0; i < number_of_points; ++i, ++iter) {
    float x = iter[0] / 100.0;
    float y = iter[1] / 100.0;
    float z = iter[2] / 100.0;
    points.push_back(rerun::Position3D{x, y, z});
    colors.push_back(rerun::Color{255, 255, 255});
  }

  // Log the entity with our data, using the `Points3D`
  rec->set_time_seconds("main", msg->header.stamp.sec +
                                    msg->header.stamp.nanosec * 1e-9);
  rec->log("point-cloud",
           rerun::Points3D(points).with_colors(colors).with_radii({0.01f}));
}

void Insider::callback_camera(
    const sensor_msgs::msg::Image::ConstSharedPtr &msg) const {
  std::vector<uint8_t> data;
  if (msg->encoding == "nv12") {
    cv::Mat bgr;
    cv::Mat yuv((int)(msg->height), (int)(msg->width), CV_8UC1,
                (uint8_t *)(msg->data.data()));
    cv::cvtColor(yuv, bgr, cv::COLOR_YUV2BGR_NV12);
    cv::imencode(".jpg", bgr, data);
  } else {
    auto img = cv_bridge::toCvShare(msg);
    auto compressed_msg = img->toCompressedImageMsg(cv_bridge::Format::JPEG);
    data.swap(compressed_msg->data);
  }
  rec->set_time_seconds("main", msg->header.stamp.sec +
                                    msg->header.stamp.nanosec * 1e-9);
  rec->log("image", rerun::EncodedImage::from_bytes(data));
}

void Insider::callback_odom(
    const nav_msgs::msg::Odometry::ConstSharedPtr &msg) {
  this->odom_x = msg->pose.pose.position.x;
}
void Insider::callback_timer() {
  rec->log("/odometry/pose/position/x", rerun::Scalar(odom_x));
}
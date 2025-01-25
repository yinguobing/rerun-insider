#include "rerun-insider/engine.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/opencv.hpp"
Insider::Insider(std::string node_name) : Node(node_name) {
  auto logger = this->get_logger();

  // Subscription topic lidar
  this->declare_parameter("subscription_topic_lidar", "");

  // Subscription topic camera
  this->declare_parameter("subscription_topic_camera", "");

  // Rerun viewer address
  this->declare_parameter("rerun_viewer_addr", "");

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
  RCLCPP_INFO(logger, "Lidar topic: %s ", subscription_topic_camera.c_str());

  // Rerun viewer address
  auto rerun_viewer_addr = this->get_parameter("rerun_viewer_addr")
                               .get_parameter_value()
                               .get<std::string>();
  RCLCPP_INFO(logger, "Rerun viewer address: %s ", rerun_viewer_addr.c_str());

  // Init rerun client, try to connect to a viewer instance.
  this->rec = new rerun::RecordingStream("rerun-insider");
  this->rec->connect_tcp(rerun_viewer_addr).exit_on_failure();

  // Init subscriptions
  sub_lidar = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      subscription_topic_lidar, rclcpp::SensorDataQoS(),
      std::bind(&Insider::callback_lidar, this, _1));

  sub_cam = this->create_subscription<sensor_msgs::msg::Image>(
      subscription_topic_camera, rclcpp::SensorDataQoS(),
      std::bind(&Insider::callback_camera, this, _1));

  RCLCPP_INFO(this->get_logger(), "Initialized");
}

Insider::~Insider() {}

void Insider::callback_lidar(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) const {
  std::vector<rerun::Position3D> points;
  std::vector<rerun::Color> colors;

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
  rec->log_static(
      "point-cloud",
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
  rec->log_static("image", rerun::EncodedImage::from_bytes(data));
}
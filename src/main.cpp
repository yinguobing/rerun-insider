#include "rerun-insider/engine.hpp"
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Insider>("rerun_insider"));
  rclcpp::shutdown();
  return 0;
}
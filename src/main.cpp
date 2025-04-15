#include "rerun-insider/engine.hpp"
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  std::string node_name = "rerun_insider";
  rclcpp::NodeOptions options;
  std::shared_ptr<Insider> node = std::make_shared<Insider>(node_name, options);
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node->get_node_base_interface());
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
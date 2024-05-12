#include "monitor/demo.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto monitor = std::make_shared<MonitorDemo2>();
  executor.add_node(monitor);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

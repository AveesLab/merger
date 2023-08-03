#include "monitor/sub.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto monitor = std::make_shared<MonitorDemo>();
  executor.add_node(monitor);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

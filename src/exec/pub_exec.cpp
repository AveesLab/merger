#include "monitor/pub.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto camera = std::make_shared<CameraDemo>();
  executor.add_node(camera);
  executor.spin();

  rclcpp::shutdown();

  return 0;
}

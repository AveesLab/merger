#include "rclcpp/rclcpp.hpp"
#include "monitor/demo_camera.hpp"
#include "monitor/demo_detection.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto monitor_camera = std::make_shared<MonitorDemoCamera>();
  auto monitor_detections = std::make_shared<MonitorDemoDetections>();

  executor.add_node(monitor_camera);
  executor.add_node(monitor_detections);

  executor.spin();
  rclcpp::shutdown();

  return 0;
}


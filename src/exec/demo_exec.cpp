#include "merger/demo.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MergerDemo>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

#include "merger/merger.hpp"

#include <memory>


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Merger>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}

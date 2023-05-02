#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <fstream>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "rtx_msg_interface/msg/bounding_box.hpp"
#include "rtx_msg_interface/msg/bounding_boxes.hpp"


class Merger : public rclcpp::Node
{
public:
  explicit Merger(std::string str);
  ~Merger();

private:
  void callback(const rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg);

  rclcpp::Subscription<rtx_msg_interface::msg::BoundingBoxes>::SharedPtr result_subscriber_;

  std::fstream file_;
  std::vector<std::pair<double, int > > tmp_list_;
};


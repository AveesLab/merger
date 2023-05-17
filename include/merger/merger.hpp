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

  double initialized_time_ = 0.0;

  std::fstream file_;
  std::vector<int> tmp_list_stamp;
  std::vector<double> tmp_list_all_node_time;
  std::vector<double> tmp_list_inference_execution_time;
  std::vector<int> tmp_list_node_index;
};


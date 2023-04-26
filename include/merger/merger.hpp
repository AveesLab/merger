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


class Merger : public rclcpp::Node
{
public:
  explicit Merger(std::string str);
  ~Merger();

private:
  void callback(const std_msgs::msg::Header::SharedPtr msg);

  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr ownership_subscriber_;

  std::fstream file_;
  std::vector<std::pair<double, int > > tmp_list_;
};


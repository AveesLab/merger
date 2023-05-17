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
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

// benchmark
#include <string>
#include <ctime>
#include <fstream>


class MergerDemo : public rclcpp::Node
{
public:
  explicit MergerDemo();
  ~MergerDemo();

private:
  void result_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void compressed_result_callback(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr result_subscriber_;
  rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr compressed_result_subscriber_;

  // benchmark
  bool use_benchmark_;
  std::fstream file_;
  void benchmark();
  void finish_benchmark();
};


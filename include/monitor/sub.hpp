#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <cmath>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>


class MonitorDemo : public rclcpp::Node
{
public:
  explicit MonitorDemo();
  ~MonitorDemo();

private:
  void image_callback1(const sensor_msgs::msg::Image::SharedPtr image);
  void image_callback2(const sensor_msgs::msg::Image::SharedPtr image);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber1_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber2_;
};

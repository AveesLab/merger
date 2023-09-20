#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <fstream>
#include <pthread.h>
#include <mutex>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <cv_bridge/cv_bridge.h>

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// benchmark
#include <string>
#include <ctime>
#include <fstream>

// ROS
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/header.hpp>
#include <posenet_msgs/msg/keypoints.hpp>
#include <posenet_msgs/msg/links.hpp>
#include <posenet_msgs/msg/objectpose.hpp>
#include <posenet_msgs/msg/poses.hpp>


class MonitorDemo : public rclcpp::Node
{
public:
  explicit MonitorDemo();
  ~MonitorDemo();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void result_callback(const posenet_msgs::msg::Poses::SharedPtr poses);
  void show_overlay(cv::Mat& input, const posenet_msgs::msg::Poses::SharedPtr poses);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<posenet_msgs::msg::Poses>::SharedPtr detections_subscriber_;

  // Shared Resource
  std::queue<cv_bridge::CvImagePtr> image_queue_;

  // mutex
  pthread_mutex_t mutex_image;
};

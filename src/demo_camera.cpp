#include "monitor/demo_camera.hpp"
#include <iomanip>
#include <stdint.h>
#include "monitor/shared_val.hpp"
cv_bridge::CvImagePtr result_image_ = nullptr;

MonitorDemoCamera::MonitorDemoCamera()
: Node("MonitorDemoCamera")
{

  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using placeholders::_1;
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/raw_image", QOS_RKL10V, bind(&MonitorDemoCamera::image_callback, this, _1));
  
  // Information
  RCLCPP_INFO(this->get_logger(), "[DemoCamera] Finish initialization.");
}


MonitorDemoCamera::~MonitorDemoCamera()
{
  RCLCPP_INFO(this->get_logger(), "[DemoCamera] Terminate system.\n");
}

void MonitorDemoCamera::image_callback(const sensor_msgs::msg::Image::SharedPtr image)
{

  cv::Mat loaded_image = cv::imread("/home/avees/RTCSA_2024/src/merger/data/4078.jpg", cv::IMREAD_COLOR);
  
  if(loaded_image.empty()) {
  RCLCPP_ERROR(this->get_logger(), "Failed to load image.");
  return;
  }
  
  auto tmp_cv_image = std::make_shared<cv_bridge::CvImage>();
  tmp_cv_image->image = loaded_image;
  tmp_cv_image->encoding = "bgr8";
  /*cv_bridge::CvImagePtr tmp_cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (tmp_cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(tmp_cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(tmp_cv_image->image, rgb8_image);
    
    tmp_cv_image->encoding = "rgb8";
  }*/

  // Append a image on queue
  pthread_mutex_lock(&mutex_image);
  result_image_ = tmp_cv_image;
  pthread_mutex_unlock(&mutex_image);
}

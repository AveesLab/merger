#include "monitor/sub.hpp"


MonitorDemo::MonitorDemo()
: Node("MonitorDemo")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).durability_volatile();

  // Subscriber
  using std::placeholders::_1;
  this->image_subscriber1_ = this->create_subscription<sensor_msgs::msg::Image>("/toy_image1", QOS_RKL10V, std::bind(&MonitorDemo::image_callback1, this, _1));
  this->image_subscriber2_ = this->create_subscription<sensor_msgs::msg::Image>("/toy_image2", QOS_RKL10V, std::bind(&MonitorDemo::image_callback2, this, _1));

  // Information
  RCLCPP_INFO(this->get_logger(), "[Demo] Finish initialization.");
}

MonitorDemo::~MonitorDemo()
{
  RCLCPP_INFO(this->get_logger(), "[Demo] Terminate system.\n");
}

void MonitorDemo::image_callback1(const sensor_msgs::msg::Image::SharedPtr image)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(cv_image->image, rgb8_image);

    cv_image->encoding = "rgb8";
  }

  cv::imshow("Camera #1", cv_image->image);
  cv::waitKey(10);
}

void MonitorDemo::image_callback2(const sensor_msgs::msg::Image::SharedPtr image)
{
  cv_bridge::CvImagePtr cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(cv_image->image, rgb8_image);

    cv_image->encoding = "rgb8";
  }

  cv::imshow("Camera #2", cv_image->image);
  cv::waitKey(10);
}
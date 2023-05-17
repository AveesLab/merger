#include "merger/demo.hpp"


MergerDemo::MergerDemo()
: Node("MergerDemo")
{
  const auto QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using std::placeholders::_1;
  this->result_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/cluster/result/image", QOS_RKL10V, std::bind(&MergerDemo::result_callback, this, _1));
  this->compressed_result_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("/cluster/result/compressed_image", QOS_RKL10V, std::bind(&MergerDemo::compressed_result_callback, this, _1));

  this->benchmark();
}

MergerDemo::~MergerDemo()
{
  this->finish_benchmark();
}

void MergerDemo::result_callback(const sensor_msgs::msg::Image::SharedPtr image)
{
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(rclcpp::Time(image->header.stamp).seconds() * 1000000.0) << ",";
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  // Convert Ros2 image to OpenCV image
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvShare(image, image->encoding);
  if (cv_image->image.empty()) return;

  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  if ((std::string)image->encoding == "bayer_rggb8")
  {
    RCLCPP_INFO(this->get_logger(), "Encoding : bayer_rggb8");
    cv::Mat color_image;
    cv::cvtColor(cv_image->image, color_image, cv::COLOR_BayerRG2RGB);
    cv::imshow("Result image", color_image);
  }
  else
  {
    RCLCPP_INFO(this->get_logger(), "Encoding : %s", (std::string)image->encoding);
    cv::imshow("Result image", cv_image->image);
  }

  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  cv::waitKey(10);

  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << "\n";
  }
}

void MergerDemo::compressed_result_callback(const sensor_msgs::msg::CompressedImage::SharedPtr compressed_image)
{
  // Convert Ros2 image to OpenCV image
  cv_bridge::CvImageConstPtr cv_image = cv_bridge::toCvCopy(compressed_image, sensor_msgs::image_encodings::RGB8);
  if (cv_image->image.empty()) return;

  RCLCPP_INFO(this->get_logger(), " - image stamp : %10.5lf s.", rclcpp::Time(compressed_image->header.stamp).seconds());
  cv::imshow("Result image", cv_image->image);

  cv::waitKey(10);
}

void MergerDemo::benchmark()
{
  use_benchmark_ = this->declare_parameter("use_benchmark", true);

  if (use_benchmark_)
  {
    time_t raw_time;
    struct tm* pTime_info;

    raw_time = time(NULL);
    pTime_info = localtime(&raw_time);

    std::string simulation_time = std::to_string(pTime_info->tm_mon + 1) + "_" + std::to_string(pTime_info->tm_mday) + "_" + std::to_string(pTime_info->tm_hour) + "_" + std::to_string(pTime_info->tm_min) + "_" + std::to_string(pTime_info->tm_sec);
    std::string directory = "./data/cluster_inference/" + simulation_time + ".csv";

    this->file_.open(directory.c_str(), std::ios_base::out | std::ios_base::app);
  }
}

void MergerDemo::finish_benchmark()
{
  if (use_benchmark_)
  {
    this->file_.close();
    RCLCPP_INFO(this->get_logger(), "Saving benchmark result is successful.");
  }
}
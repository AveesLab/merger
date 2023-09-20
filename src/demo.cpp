#include "monitor/demo.hpp"


MonitorDemo::MonitorDemo()
: Node("MonitorDemo")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using std::placeholders::_1;
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/raw_image", QOS_RKL10V, std::bind(&MonitorDemo::image_callback, this, _1));
  this->detections_subscriber_ = this->create_subscription<posenet_msgs::msg::Poses>("/cluster/poses", QOS_RKL10V, std::bind(&MonitorDemo::result_callback, this, _1));

  // Information
  RCLCPP_INFO(this->get_logger(), "[Demo] Finish initialization.");
}

MonitorDemo::~MonitorDemo()
{
  RCLCPP_INFO(this->get_logger(), "[Demo] Terminate system.\n");
}

void MonitorDemo::image_callback(const sensor_msgs::msg::Image::SharedPtr image)
{
  cv_bridge::CvImagePtr tmp_cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (tmp_cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(tmp_cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(tmp_cv_image->image, rgb8_image);

    tmp_cv_image->encoding = "rgb8";
  }

  // Append a image on queue
  pthread_mutex_lock(&mutex_image);
  this->image_queue_.push(tmp_cv_image);
  pthread_mutex_unlock(&mutex_image);
}

void MonitorDemo::result_callback(const posenet_msgs::msg::Poses::SharedPtr poses)
{
  // Select image
  cv_bridge::CvImagePtr cv_image = nullptr;

  pthread_mutex_lock(&mutex_image);

  int detections_stamp = static_cast<int>(rclcpp::Time(poses->header.stamp).seconds() * 1000.0);
  while (this->image_queue_.size())
  {
    int queued_image_stamp = static_cast<int>(rclcpp::Time(this->image_queue_.front()->header.stamp).seconds() * 1000.0);

    if (queued_image_stamp != detections_stamp)
    {
      // Consume image
      if (queued_image_stamp < detections_stamp)
      {
        this->image_queue_.pop();
      }
      if (queued_image_stamp > detections_stamp)
      {
        break;
      }
    }
    else
    {
      // Save image
      cv_image = this->image_queue_.front();

      this->image_queue_.pop();
    }
  }

  pthread_mutex_unlock(&mutex_image);

  // Check image
  if (cv_image == nullptr)
  {
    return;
  }

  // Draw bounding boxes
  show_overlay(cv_image->image, poses);
}

void MonitorDemo::show_overlay(cv::Mat& input, const posenet_msgs::msg::Poses::SharedPtr poses)
{
	// get the image dimensions
	const float line_width = std::max(std::max(input.cols, input.rows) * 0.0013f, 1.5f);
  const float circle_radius = std::max(std::max(input.cols, input.rows) * 0.0052f, 4.0f);

	for (const auto& pose : poses->poses)
    {
      for (const auto& link : pose.links)
      {
        const int a = link.first;
        const int b = link.second;
        cv::line(input, cv::Point(pose.keypoints[a].x, pose.keypoints[a].y), cv::Point(pose.keypoints[b].x, pose.keypoints[b].y), cv::Scalar(0, 255, 0), line_width);
      }

      for (const auto& keypoint : pose.keypoints)
      {
        cv::circle(input, cv::Point(keypoint.x, keypoint.y), circle_radius, cv::Scalar(0, 255, 0), -1);
      }
    }

	cv::imshow("Overlay Image", input);
  cv::waitKey(1);
}
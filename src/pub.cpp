#include "monitor/pub.hpp"


CameraDemo::CameraDemo()
: Node("CameraDemo")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  this->period_ = this->declare_parameter("period", 33);
  this->video_path_ = this->declare_parameter("video_path", "/home/avees/ros2_ws/video/test.mp4");
  this->node_index_ = this->declare_parameter("node_index", 1);

  std::string topic_name = "/toy_image" + std::to_string(this->node_index_);

  // Publisher
  this->image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name , 1);

  // Timer
  this->timer_ = this->create_wall_timer(std::chrono::milliseconds(this->period_), std::bind(&CameraDemo::timer_callback, this));

  // Get images
  this->cap_ = cv::VideoCapture(this->video_path_);

  int total_frames = this->cap_.get(cv::CAP_PROP_FRAME_COUNT);
  cv::Mat frame;
  int current_frame = 0;

  while(cap_.read(frame)) {
    cv_bridge::CvImage cv_image;
    cv_image.encoding = "bgr8";
    cv_image.image = frame;
    frames_.push_back(cv_image);

    current_frame++;
    RCLCPP_INFO(this->get_logger(), "Loading video... %d%%", (int)(current_frame / (float)total_frames * 100));
  }

  // Information
  RCLCPP_INFO(this->get_logger(), "[Demo] Finish initialization.");
}

CameraDemo::~CameraDemo()
{
  RCLCPP_INFO(this->get_logger(), "[Demo] Terminate system.\n");
}

void CameraDemo::timer_callback()
{
  if (frame_id_ < frames_.size()) {
    frames_[frame_id_].header.stamp = now();
    frames_[frame_id_].header.frame_id = std::to_string(frame_id_++);
    auto msg = frames_[frame_id_].toImageMsg();
    image_publisher_->publish(*msg);
  }
}
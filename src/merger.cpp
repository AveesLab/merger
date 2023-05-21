#include "merger/merger.hpp"


Merger::Merger()
: Node("Merger")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using std::placeholders::_1;
  this->result_subscriber_ = this->create_subscription<rtx_msg_interface::msg::BoundingBoxes>("/cluster/result", QOS_RKL10V, std::bind(&Merger::callback, this, _1));
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("camera/raw_image", QOS_RKL10V, std::bind(&Merger::image_callback, this, _1));

  this->benchmark();

  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Finish.");
}

Merger::~Merger()
{
  this->finish_benchmark();
}

void Merger::image_callback(const sensor_msgs::msg::Image::SharedPtr image)
{
  cv_bridge::CvImagePtr tmp_cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (tmp_cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(tmp_cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(tmp_cv_image->image, rgb8_image);

    tmp_cv_image->encoding = "rgb8";
  }

  this->image_queue_.push(tmp_cv_image);
}

void Merger::callback(const rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg)
{
  rclcpp::Time received_result_image_stamp = rclcpp::Time(msg->image_header.stamp);

  while (this->image_queue_.size())
  {
    rclcpp::Time queued_image_stamp = rclcpp::Time(this->image_queue_.front()->header.stamp);

    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(queued_image_stamp.seconds() * 1000000.0) << ",";
    }

    if (queued_image_stamp < received_result_image_stamp)
    {
      // consume image
      this->image_queue_.pop();

      if (use_benchmark_) {
        this->file_ << 0 << ",";
      }

      // Information
      RCLCPP_INFO(this->get_logger(), "Comsume Image.");
    }
    else
    {
      if (queued_image_stamp == received_result_image_stamp)
      {
        draw_image(this->image_queue_.front(), msg);

        cv::imshow("Result image", this->image_queue_.front()->image);
        cv::waitKey(10);

        this->image_queue_.pop();

        if (use_benchmark_) {
          this->file_ << 1 << ",";
        }

        RCLCPP_INFO(this->get_logger(), "Show Detected Image.");
      }

      break;
    }
  }

  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << "\n";
  }
}

void Merger::draw_image(cv_bridge::CvImagePtr cv_image, rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg)
{
  for (size_t i = 0; i < msg->bounding_boxes.size(); i++) {
    // Get rectangle from 1 object
    cv::Rect r = cv::Rect(round(msg->bounding_boxes[i].left),
                          round(msg->bounding_boxes[i].top),
                          round(msg->bounding_boxes[i].right - msg->bounding_boxes[i].left),
                          round(msg->bounding_boxes[i].bot - msg->bounding_boxes[i].top));

    // draw_box
    cv::rectangle(cv_image->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

    // put id
    cv::putText(cv_image->image, std::to_string(static_cast<int>(msg->bounding_boxes[i].id)), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
}

// benchmark
void Merger::benchmark()
{
  use_benchmark_ = this->declare_parameter("use_benchmark", false);

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

void Merger::finish_benchmark()
{
  if (use_benchmark_)
  {
    this->file_.close();
    RCLCPP_INFO(this->get_logger(), "Saving benchmark result is successful.");
  }
}
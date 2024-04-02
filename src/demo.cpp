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
  this->detections_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("/detections", QOS_RKL10V, std::bind(&MonitorDemo::detections_receive, this, _1));

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
  this->result_image_ = tmp_cv_image;
  pthread_mutex_unlock(&mutex_image);
}

void MonitorDemo::detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
  // Select image
  cv_bridge::CvImagePtr cv_image = nullptr;

  pthread_mutex_lock(&mutex_image);
  
  RCLCPP_INFO(this->get_logger(), "node_index: %s, num_detection: %u, Computing_nodes_timestamp : %2f", detections->header.frame_id.c_str(), detections->detections.size(), rclcpp::Time(detections->header.stamp).seconds());
  
  // Convert frame_id to int and mark as received
  int frame_id = std::stoi(detections->header.frame_id);
  if (frame_id >= 1 && frame_id <= 9) {
    detections_received[frame_id - 1] = true; // Mark as received
  }

  // Check if all detections have been received
  bool all_received = std::all_of(detections_received.begin(), detections_received.end(), [](bool received) { return received; });
  if (all_received) {
    cerr << "All node are received!" << endl;
  }

  // Save image
  cv_image = this->result_image_;

  pthread_mutex_unlock(&mutex_image);

  // Check image
  if (cv_image == nullptr)
  {
    return;
  }

 /* // Draw bounding boxes
  draw_image(cv_image, detections);

  // Show image
  cv::imshow("Result image", cv_image->image);
  cv::waitKey(10);
  
  // Save the rejult image
  save_img(cv_image);
  */
  if (all_received) std::fill(detections_received.begin(), detections_received.end(), false);
}

void MonitorDemo::draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
  for (size_t i = 0; i < detections->detections.size(); i++) {
    // Get rectangle from 1 object
    cv::Rect r = cv::Rect(round(detections->detections[i].bbox.center.x - detections->detections[i].bbox.size_x),
                          round(detections->detections[i].bbox.center.y - detections->detections[i].bbox.size_y),
                          round(2 * detections->detections[i].bbox.size_x),
                          round(2 * detections->detections[i].bbox.size_y));

    // draw_box
    cv::rectangle(cv_image->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

    // put id
    cv::putText(cv_image->image, detections->detections[i].tracking_id, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
}

void MonitorDemo::save_img(cv_bridge::CvImagePtr cv_image) {
    std::string file_path = "./src/merger/result/";

    auto t = std::time(nullptr);
    auto tm = *std::localtime(&t);
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    std::string fileName = oss.str() + ".png";
    std::string fullPath = file_path + fileName;

    cv::imwrite(fullPath, cv_image->image);
}

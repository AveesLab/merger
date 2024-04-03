#include "monitor/demo.hpp"


MonitorDemo::MonitorDemo()
: Node("MonitorDemo")
{
  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  // Subscriber
  using placeholders::_1;
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/raw_image", QOS_RKL10V, bind(&MonitorDemo::image_callback, this, _1));
  this->detections_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>("/detections", QOS_RKL10V, bind(&MonitorDemo::detections_receive, this, _1));

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

  pthread_mutex_lock(&mutex_receive);
  
  RCLCPP_INFO(this->get_logger(), "node_index: %s, num_detection: %u, Computing_nodes_timestamp : %2f", detections->header.frame_id.c_str(), detections->detections.size(), rclcpp::Time(detections->header.stamp).seconds());
  
  detection_list.push_back(detections);
  //cerr << "detection list size:" << detection_list.size() << endl;
  
  // Convert frame_id to int and mark as received
  int frame_id = stoi(detections->header.frame_id);
  if (frame_id >= 1 && frame_id <= 9) {
    detections_received[frame_id - 1] = true; // Mark as received
  }

  // Check if all detections have been received
  bool all_received = all_of(detections_received.begin(), detections_received.end(), [](bool received) { return received; });
  if (all_received) {
    cerr << "All node are received!" << endl;
  }
  pthread_mutex_unlock(&mutex_receive);
  
  pthread_mutex_lock(&mutex_image);

  cv_image = this->result_image_;

  pthread_mutex_unlock(&mutex_image);

  // Check image
  if (cv_image == nullptr)
  {
    return;
  }

  
  pthread_mutex_lock(&mutex_receive_check);

  if (all_received) {
  
  // Draw bounding boxes
  draw_image(cv_image, detection_list[0]);
  draw_image(cv_image, detection_list[1]);
  draw_image(cv_image, detection_list[2]);
  draw_image(cv_image, detection_list[3]);
  draw_image(cv_image, detection_list[4]);
  draw_image(cv_image, detection_list[5]);
  draw_image(cv_image, detection_list[6]);
  draw_image(cv_image, detection_list[7]);
  draw_image(cv_image, detection_list[8]);

  // Show image
  cv::imshow("Result image", cv_image->image);
  cv::waitKey(10);
  
  // Save the result image
  save_img(cv_image);
  
  fill(detections_received.begin(), detections_received.end(), false);
  detection_list.clear();
  }
  pthread_mutex_unlock(&mutex_receive_check);

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
    string file_path = "./src/merger/result/";

    auto t = time(nullptr);
    auto tm = *localtime(&t);
    ostringstream oss;
    oss << put_time(&tm, "%Y-%m-%d_%H-%M-%S");
    string fileName = oss.str() + ".png";
    string fullPath = file_path + fileName;

    cv::imwrite(fullPath, cv_image->image);
}

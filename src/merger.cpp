#include "merger/merger.hpp"


Merger::Merger(std::string str)
: Node("Merger")
{
  // Set csv name
  time_t raw_time;
  struct tm* pTime_info;

  raw_time = time(NULL);
  pTime_info = localtime(&raw_time);

  std::string simulation_time = std::to_string(pTime_info->tm_mon + 1) + "_" + std::to_string(pTime_info->tm_mday) + "_" + std::to_string(pTime_info->tm_hour) + "_" + std::to_string(pTime_info->tm_min);
  std::string directory = "./src/merger/data/" + str + "_" + simulation_time + ".csv";

  this->file_.open(directory.c_str(), std::ios_base::out | std::ios_base::app);

  rclcpp::QoS system_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

  // Subscriber
  using std::placeholders::_1;
  this->result_subscriber_ = this->create_subscription<rtx_msg_interface::msg::BoundingBoxes>("/cluster/result", system_qos, std::bind(&Merger::callback, this, _1));
}

Merger::~Merger()
{
 // std::sort(this->tmp_list_.begin(), this->tmp_list_.end());

  int max_index = static_cast<int>(this->tmp_list_stamp.size());
  for (int index = 0; index < max_index; index++) {
    this->file_ << this->tmp_list_stamp[index] << ","
                << this->tmp_list_all_node_time[index] << ","
                << this->tmp_list_inference_execution_time[index] << ","
                << this->tmp_list_node_index[index] << "\n";
  }

  this->file_.close();
}

void Merger::callback(const rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg)
{
  std_msgs::msg::Header msg_header = msg->image_header;
  if (this->initialized_time_ == 0.0)
  {
    try {
      this->initialized_time_ = rclcpp::Time(msg->image_header.stamp).seconds() * 1000.0;
    }
    catch (std::string & s) {
      RCLCPP_ERROR(this->get_logger(), "No match time stamp");
      return;
    }
  }

  rclcpp::Time now_stamp = this->get_clock()->now();

  rclcpp::Time received_image_stamp;
  int received_image_time = -1;
  try {
    received_image_stamp = rclcpp::Time(msg_header.stamp);
    received_image_time = static_cast<int>(received_image_stamp.seconds() * 1000.0 - this->initialized_time_);
  }
  catch (std::string & s) {
    RCLCPP_ERROR(this->get_logger(), "No match time stamp");
    return;
  }

  rclcpp::Time inference_node_stamp;
  double inference_node_time = -1;
  try {
    inference_node_stamp = rclcpp::Time(msg->header.stamp);
    inference_node_time = inference_node_stamp.seconds() * 1000.0;
  }
  catch (std::string & s) {
    RCLCPP_ERROR(this->get_logger(), "No match time stamp");
    return;
  }
  
  int node_index = -1;
  try {
    node_index = std::stoi(msg_header.frame_id);
  }
  catch (std::string & s) {
    RCLCPP_ERROR(this->get_logger(), "No match frame_id");
    return;
  }

  double all_node_time = -1;
  try {
    if ((now_stamp - received_image_stamp).seconds() < 100.0)
    {
      all_node_time = (now_stamp - received_image_stamp).seconds() * 1000.0;
    }
  }
  catch (std::string & s) {
    RCLCPP_ERROR(this->get_logger(), "image's stamp is not server time");
    return;
  }

  RCLCPP_INFO(this->get_logger(), "[Merger] : Receive result. (%lf ms.)", rclcpp::Time(msg_header.stamp).seconds() * 1000.0);

  this->tmp_list_stamp.push_back(received_image_time);
  this->tmp_list_all_node_time.push_back(all_node_time);
  this->tmp_list_inference_execution_time.push_back(inference_node_time);
  this->tmp_list_node_index.push_back(node_index);
}

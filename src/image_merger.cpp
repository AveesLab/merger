#include "merger/merger.hpp"


MergerDemo::MergerDemo(std::string str)
: Node("MergerDemo")
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
  this->result_subscriber_ = this->create_subscription<rtx_msg_interface::msg::BoundingBoxes>("/cluster/result/image", system_qos, std::bind(&Merger::callback, this, _1));
}

Merger::~Merger()
{
 // std::sort(this->tmp_list_.begin(), this->tmp_list_.end());

  int max_index = static_cast<int>(this->tmp_list_.size());
  for (int index = 0; index < max_index; index++) {
    this->file_ << this->tmp_list_[index].first << "," << this->tmp_list_[index].second << "\n";
  }

  this->file_.close();
}

void Merger::callback(const rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg)
{
  std_msgs::msg::Header msg_header = msg->image_header;

  double received_time = -1.0;
  try {
    received_time = rclcpp::Time(msg_header.stamp).seconds();
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
  
  this->tmp_list_.push_back({received_time, node_index});
}

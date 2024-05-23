#include "monitor/demo_detection.hpp"
#include <iomanip>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <thread>
#include <fstream>
#include <vector>
#include <algorithm>
#include <map>

MonitorDemoDetections::MonitorDemoDetections()
: Node("MonitorDemoDetections"),
  detections_received_(TOTAL_NUM_OF_NODES, false),
  draw_num_(0),
  cluster_num_(0),
  all_received_(false)
{
  RCLCPP_INFO(this->get_logger(), "Initialization Start.");

  rclcpp::QoS QOS_RKL10V = rclcpp::QoS(rclcpp::KeepLast(10)).reliable().durability_volatile();

  detections_subscriber_ = this->create_subscription<vision_msgs::msg::Detection2DArray>(
    "/detections", QOS_RKL10V, std::bind(&MonitorDemoDetections::detections_receive, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "[Detections] Finish initialization.");
}

MonitorDemoDetections::~MonitorDemoDetections()
{
  RCLCPP_INFO(this->get_logger(), "[Detections] Terminate system.\n");
}

void MonitorDemoDetections::detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
  std::thread([this, detections] {
    //std::lock_guard<std::mutex> lock(mutex_receive_);
    pthread_mutex_lock(&mutex_d_receive);

    if (all_of(detections_received_.begin(), detections_received_.end(), [](bool received) { return !received; })) {
      start_waiting_all_received_.push_back(get_time_in_ms());
    }
    int frame_id = stoi(detections->header.frame_id);
    node_index_.push_back(frame_id);
    node_end_ethernet_.push_back(get_time_in_ms());
    
    RCLCPP_INFO(this->get_logger(), "node_index: %s, num_detection: %u, Computing_nodes_timestamp : %2f", detections->header.frame_id.c_str(), detections->detections.size(), rclcpp::Time(detections->header.stamp).seconds());
                
    std::cerr << frame_id << std::endl;
  
   
    detection_list[frame_id - 1] = detections;
    std::cerr << detections << std::endl;
    
    if (frame_id >= 1 && frame_id <= TOTAL_NUM_OF_NODES) {
      detections_received_[frame_id - 1] = true;
      status_.push_back(0);
      std::cerr << "success push back" << std::endl;
      all_received_ = all_of(detections_received_.begin(), detections_received_.end(), [](bool received) { return received; });

      if (all_received_) {
        end_ethernet_.push_back(node_end_ethernet_.back());
        status_.pop_back();
        status_.push_back(1);

        std::cerr << draw_num_ << "All node are received!" << std::endl;
        end_waiting_all_received_.push_back(get_time_in_ms());
      }
    }
    
    pthread_mutex_unlock(&mutex_d_receive);
    
    pthread_mutex_lock(&mutex_a_receive);    

    if (all_received_) {
      //std::lock_guard<std::mutex> image_lock(mutex_receive_);

      start_merge_.push_back(get_time_in_ms());
      
      pthread_mutex_lock(&mutex_image);
      
      cv_bridge::CvImagePtr cv_image = this->result_image_;
      if (cv_image == nullptr) {
        return;
      }
      pthread_mutex_unlock(&mutex_image);

      std::vector<BoundingBox> partial_car_bboxes;
      filterDetections(detection_list, partial_car_bboxes);
      merge_bbox_with_clustering(partial_car_bboxes);
      end_merge_.push_back(get_time_in_ms());

      start_draw_.push_back(get_time_in_ms());
      for (int node_id = 0; node_id < TOTAL_NUM_OF_NODES; node_id++) {
        draw_image(cv_image, detection_list[node_id]);
      }

      for (const auto& pair : clusterBoxes_) {
        cv::rectangle(cv_image->image, pair.second, cv::Scalar(0, 0, 255), 2);
      }

      end_draw_.push_back(get_time_in_ms());

      start_display_.push_back(get_time_in_ms());
      cv::imshow("After merge image", cv_image->image);
      cv::waitKey(1);
      end_display_.push_back(get_time_in_ms());

      std::fill(detections_received_.begin(), detections_received_.end(), false);
      labels_.clear();
      clusterBoxes_.clear();

      draw_num_++;
      if (draw_num_ == EXP_NUM) {
        for (int i = 0; i < EXP_NUM; i++) {
          e_waiting_all_received_.push_back(end_waiting_all_received_[i] - start_waiting_all_received_[i]);
          e_merge_.push_back(end_merge_[i] - start_merge_[i]);
          e_draw_.push_back(end_draw_[i] - start_draw_[i]);
          e_display_.push_back(end_display_[i] - start_display_[i]);
        }

        std::ofstream file1("master_node.csv");
        file1 << std::fixed << std::setprecision(6)
              << "start_waiting_all_received," << "e_waiting_all_received(us)," << "end_waiting_all_received,"
              << "end_ethernet,"
              << "start_merge," << "e_merge(us)," << "end_merge,"
              << "start_draw," << "e_draw(us)," << "end_draw,"
              << "start_display," << "e_display(us)," << "end_display\n";
        for (int i = 0; i < draw_num_; i++) {
          file1 << start_waiting_all_received_[i] << "," << e_waiting_all_received_[i] << "," << end_waiting_all_received_[i]
                << "," << end_ethernet_[i] << "," << start_merge_[i] << "," << e_merge_[i] << "," << end_merge_[i]
                << "," << start_draw_[i] << "," << e_draw_[i] << "," << end_draw_[i]
                << "," << start_display_[i] << "," << e_display_[i] << "," << end_display_[i] << "\n";
        }
        file1.close();

        std::ofstream file2("master_node_ethernet.csv");
        file2 << std::fixed << std::setprecision(6) << "node_index," << "end_ethernet," << "status\n";
        for (int i = 0; i < draw_num_ * TOTAL_NUM_OF_NODES; i++) {
          file2 << node_index_[i] << "," << node_end_ethernet_[i] << "," << status_[i] << "\n";
        }
        file2.close();

        std::cerr << "write result at " << "./master_node.csv" << std::endl;
        std::cerr << "write result at " << "./master_node_ethernet.csv" << std::endl;

        exit(0);
      }
    }
    pthread_mutex_unlock(&mutex_a_receive);
  }).detach();
}

uint64_t MonitorDemoDetections::get_time_in_ms()
{
  rclcpp::Time now = this->get_clock()->now();
  uint64_t nanosecond = now.nanoseconds();
  return nanosecond / 1000;
}

void MonitorDemoDetections::draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{
  for (size_t i = 0; i < detections->detections.size(); i++) {
    cv::Rect r = cv::Rect(round(detections->detections[i].bbox.center.x - detections->detections[i].bbox.size_x),
                          round(detections->detections[i].bbox.center.y - detections->detections[i].bbox.size_y),
                          round(2 * detections->detections[i].bbox.size_x),
                          round(2 * detections->detections[i].bbox.size_y));
    cv::rectangle(cv_image->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);
    cv::putText(cv_image->image, detections->detections[i].tracking_id, cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
}

void MonitorDemoDetections::filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list,
                                             std::vector<BoundingBox>& partial_car_bboxes)
{
  for (const auto& detections : detection_list) {
    for (const auto& detection : detections->detections) {
      if (detection.tracking_id == "1") {
        BoundingBox box;
        box.centerX = detection.bbox.center.x;
        box.centerY = detection.bbox.center.y;
        box.width = 2 * detection.bbox.size_x;
        box.height = 2 * detection.bbox.size_y;
        partial_car_bboxes.push_back(box);
      }
    }
  }
}

void MonitorDemoDetections::simpleDBSCAN(const std::vector<BoundingBox> partial_car_bboxes, double eps, int minPts)
{
  int clusterId = 0;
  labels_.resize(partial_car_bboxes.size(), -1);

  for (size_t i = 0; i < partial_car_bboxes.size(); ++i) {
    if (labels_[i] != -1) continue;

    std::vector<size_t> neighbors;

    for (size_t j = 0; j < partial_car_bboxes.size(); ++j) {
      double distance = std::sqrt(std::pow(partial_car_bboxes[i].centerX - partial_car_bboxes[j].centerX, 2) +
                                  std::pow(partial_car_bboxes[i].centerY - partial_car_bboxes[j].centerY, 2));
      if (distance <= eps) {
        neighbors.push_back(j);
      }
    }

    if (neighbors.size() < minPts) continue;
    for (auto idx : neighbors) {
      labels_[idx] = clusterId;
    }
    clusterId++;
  }
}

void MonitorDemoDetections::merge_bbox_with_clustering(const std::vector<BoundingBox> partial_car_bboxes)
{
  double eps = 300.0;
  int minPts = 2;

  simpleDBSCAN(partial_car_bboxes, eps, minPts);

  for (size_t i = 0; i < partial_car_bboxes.size(); ++i) {
    if (labels_[i] == -1) continue;

    cv::Point top_left(partial_car_bboxes[i].centerX - partial_car_bboxes[i].width / 2, partial_car_bboxes[i].centerY - partial_car_bboxes[i].height / 2);
    cv::Point bottom_right(partial_car_bboxes[i].centerX + partial_car_bboxes[i].width / 2, partial_car_bboxes[i].centerY + partial_car_bboxes[i].height / 2);

    cv::Rect currentBox(top_left, bottom_right);
    if (clusterBoxes_.find(labels_[i]) == clusterBoxes_.end()) {
      clusterBoxes_[labels_[i]] = currentBox;
    } else {
      clusterBoxes_[labels_[i]] |= currentBox;
    }
  }
}

void MonitorDemoDetections::save_img(cv_bridge::CvImagePtr cv_image)
{
  std::string file_path = "./src/merger/result/";

  auto t = std::time(nullptr);
  auto tm = *std::localtime(&t);
  std::ostringstream oss;
  oss << std::put_time(&tm, "%Y-%m-%d_%H-%M-%S");
  std::string fileName = oss.str() + ".png";
  std::string fullPath = file_path + fileName;

  cv::imwrite(fullPath, cv_image->image);
}


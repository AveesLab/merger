#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <queue>
#include <stdexcept>
#include <cmath>
#include <ctime>
#include <fstream>
#include <pthread.h>
#include <mutex>
#include <stdint.h>

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "vision_msgs/msg/detection2_d_array.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/float32_multi_array.hpp"

// OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/objdetect/objdetect.hpp>

// benchmark
#include <string>
#include <ctime>
#include <fstream>
#include <iostream>
using namespace std;

// ROS
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/header.hpp"

// DNN Inference
#include "/home/avees/RTCSA_2024/src/merger/include/objectdetection/objectdetection.hpp"

#define TOTAL_NUM_OF_NODES 9
#define EXP_NUM 15
bool all_received;

pthread_mutex_t mutex_receive_check;
pthread_mutex_t mutex_image;
pthread_mutex_t mutex_receive;
vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list(TOTAL_NUM_OF_NODES);
vector<std_msgs::msg::Float32MultiArray::SharedPtr> yolo_list(TOTAL_NUM_OF_NODES);
std_msgs::msg::Float32MultiArray yolo_data;

map<int, cv::Rect> clusterBoxes;
vector<int> labels;
  
struct BoundingBox {
      double centerX;
      double centerY;
      double width;
      double height;
  };

class MonitorDemoDetections : public rclcpp::Node
{
public:
  explicit MonitorDemoDetections();
  ~MonitorDemoDetections();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections);
  void yolo_receive(const std_msgs::msg::Float32MultiArray::SharedPtr yolo);
  void draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections);
  void merge_bbox_with_clustering(const vector<BoundingBox> partial_car);
  void simpleDBSCAN(const std::vector<BoundingBox> partial_car, double eps, int minPts);
  void save_img(cv_bridge::CvImagePtr cv_image);
  void filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list, std::vector<BoundingBox>& partial_car_bboxes);
  uint64_t get_time_in_ms();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr yolo_subscriber_;  

  // Shared Resource
  queue<cv_bridge::CvImagePtr> image_queue_;
  
  
  vector<bool> detections_received{vector<bool>(TOTAL_NUM_OF_NODES, false)};
  vector<bool> yolo_received{vector<bool>(TOTAL_NUM_OF_NODES, false)};
  
  // Object Detection
  std::shared_ptr<Darknet> inference_;
  std::string dnn_model_path_;
  std::string dnn_cfg_path_;
  std::string dnn_weight_path_;

};

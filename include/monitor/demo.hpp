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

#define TOTAL_NUM_OF_NODES 3
#define EXP_NUM 190
bool all_received;

pthread_mutex_t mutex_receive_check;
pthread_mutex_t mutex_init;
vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list(TOTAL_NUM_OF_NODES);

map<int, cv::Rect> clusterBoxes;
vector<int> labels;
  
struct BoundingBox {
      double centerX;
      double centerY;
      double width;
      double height;
  };

class MonitorDemo1 : public rclcpp::Node
{
public:
  explicit MonitorDemo1();
  ~MonitorDemo1();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections1);
  void draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections1);
  void merge_bbox_with_clustering(const vector<BoundingBox> partial_car);
  void simpleDBSCAN(const std::vector<BoundingBox> partial_car, double eps, int minPts);
  void save_img(cv_bridge::CvImagePtr cv_image);
  void filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list, std::vector<BoundingBox>& partial_car_bboxes);
  uint64_t get_time_in_ms();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections1_subscriber_;

  // Shared Resource
  queue<cv_bridge::CvImagePtr> image_queue_;
  cv_bridge::CvImagePtr result_image_;
  
  vector<bool> detections_received{vector<bool>(TOTAL_NUM_OF_NODES, false)};
  // mutex
  pthread_mutex_t mutex_image;
  pthread_mutex_t mutex_receive;
  pthread_mutex_t mutex_init;
   

};


class MonitorDemo2 : public rclcpp::Node
{
public:
  explicit MonitorDemo2();
  ~MonitorDemo2();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections2);
  void draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections2);
  void merge_bbox_with_clustering(const vector<BoundingBox> partial_car);
  void simpleDBSCAN(const std::vector<BoundingBox> partial_car, double eps, int minPts);
  void save_img(cv_bridge::CvImagePtr cv_image);
  void filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list, std::vector<BoundingBox>& partial_car_bboxes);
  uint64_t get_time_in_ms();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections2_subscriber_;

  // Shared Resource
  queue<cv_bridge::CvImagePtr> image_queue_;
  cv_bridge::CvImagePtr result_image_;
  
  vector<bool> detections_received{vector<bool>(TOTAL_NUM_OF_NODES, false)};
  // mutex
  pthread_mutex_t mutex_image;
  pthread_mutex_t mutex_receive;
};

class MonitorDemo3 : public rclcpp::Node
{
public:
  explicit MonitorDemo3();
  ~MonitorDemo3();

private:
  void image_callback(const sensor_msgs::msg::Image::SharedPtr image);
  void detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections3);
  void draw_image(cv_bridge::CvImagePtr cv_image, const vision_msgs::msg::Detection2DArray::SharedPtr detections3);
  void merge_bbox_with_clustering(const vector<BoundingBox> partial_car);
  void simpleDBSCAN(const std::vector<BoundingBox> partial_car, double eps, int minPts);
  void save_img(cv_bridge::CvImagePtr cv_image);
  void filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list, std::vector<BoundingBox>& partial_car_bboxes);
  uint64_t get_time_in_ms();
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscriber_;
  rclcpp::Subscription<vision_msgs::msg::Detection2DArray>::SharedPtr detections3_subscriber_;

  // Shared Resource
  queue<cv_bridge::CvImagePtr> image_queue_;
  cv_bridge::CvImagePtr result_image_;
  
  vector<bool> detections_received{vector<bool>(TOTAL_NUM_OF_NODES, false)};
  // mutex
  pthread_mutex_t mutex_image;
  pthread_mutex_t mutex_receive;
};

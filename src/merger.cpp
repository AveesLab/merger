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
  this->image_subscriber_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/raw_image", QOS_RKL10V, std::bind(&Merger::image_callback, this, _1));

  this->benchmark();

  if (use_can_)
  {
    this->can_receiver_ = std::make_shared<ObjectDetectionsReceiver>();

    this->detections_per_node_.resize(this->number_of_nodes_);

    // pthread
    pthread_mutex_init(&mutex, NULL);
    pthread_mutex_init(&mutex_image, NULL);
    pthread_cond_init(&cond, NULL);

    pthread_create(&thread_receive, NULL, receive_thread, this);
    pthread_create(&thread_show, NULL, show_thread, this);
  }

  // Information
  RCLCPP_INFO(this->get_logger(), "Initialization Finish.");
}

Merger::~Merger()
{
  std::cerr << "Destroy Start.\n";

  this->finish_benchmark();

  std::cerr << "Benchmark fin.\n";

  if (use_can_)
  {
    run_flag_ = false;

    pthread_mutex_destroy(&mutex);
    pthread_cond_destroy(&cond);

    pthread_join(thread_receive, NULL);
    pthread_join(thread_show, NULL);
  }
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

  if (use_can_)
  {
    pthread_mutex_lock(&mutex_image);

    this->image_queue_.push(tmp_cv_image);

    pthread_mutex_unlock(&mutex_image);
  }
  else
  {
    this->image_queue_.push(tmp_cv_image);
  }
}

void Merger::callback(const rtx_msg_interface::msg::BoundingBoxes::SharedPtr msg)
{
  // benchmark
  if (use_benchmark_) {
    // beforegetdetections / aftergetdetections
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  rclcpp::Time received_result_image_stamp = rclcpp::Time(msg->image_header.stamp);

  // benchmark
  if (use_benchmark_) {
    // beforeselectimage
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  while (this->image_queue_.size())
  {
    rclcpp::Time queued_image_stamp = rclcpp::Time(this->image_queue_.front()->header.stamp);

    if (queued_image_stamp < received_result_image_stamp)
    {
      // consume image
      this->image_queue_.pop();

      // Information
      RCLCPP_INFO(this->get_logger(), "Comsume Image.");
    }
    else
    {
      // benchmark
      if (use_benchmark_) {
        // afterselectimage
        this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
      }

      if (queued_image_stamp == received_result_image_stamp)
      {
        // benchmark
        if (use_benchmark_) {
          // isshowimage
          this->file_ << static_cast<long long int>(1) << ",";
        }

        draw_image(this->image_queue_.front(), *msg);

        // benchmark
        if (use_benchmark_) {
          // afterdrawimage
          this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
        }

        cv::imshow("Result image", this->image_queue_.front()->image);
        cv::waitKey(10);

        this->image_queue_.pop();

        RCLCPP_INFO(this->get_logger(), "Show Detected Image.");

        // benchmark
        if (use_benchmark_) {
          // endpoint
          this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
          // timestamp
          this->file_ << static_cast<long long int>(queued_image_stamp.seconds() * 1000000.0) << "\n";
        }
      }
      else
      {
        // benchmark
        if (use_benchmark_) {
          // isshowimage
          this->file_ << static_cast<long long int>(0) << "\n";
        }
      }

      break;
    }
  }
}

void Merger::can_receive()
{
  ObjectDetection detection;
  int node_index;

  node_index = this->can_receiver_->GetMessage(detection);

  if (detection.id == -1)
  {
    pthread_mutex_lock(&mutex);

    if (this->detections_per_node_[node_index].size())
    {
      std::vector<ObjectDetection>().swap(this->detections_);
      this->detections_per_node_[node_index].push_back(detection);
      this->detections_.swap(this->detections_per_node_[node_index]);

      pthread_cond_signal(&cond);
    }

    pthread_mutex_unlock(&mutex);
  }
  else
  {
    pthread_mutex_lock(&mutex);

    this->detections_per_node_[node_index].push_back(detection);

    pthread_mutex_unlock(&mutex);
  }

  usleep(300);
}

void Merger::can_show()
{
  // // Result
  pthread_mutex_lock(&mutex);

  pthread_cond_wait(&cond, &mutex);

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  std::vector<ObjectDetection> detections;
  detections.swap(this->detections_);

  pthread_mutex_unlock(&mutex);

  rtx_msg_interface::msg::BoundingBoxes msg;

  for (int object_index = 0; object_index < static_cast<int>(detections.size()) - 1; object_index++)
  {
    rtx_msg_interface::msg::BoundingBox msg_part;
    
    msg_part.id = detections[object_index].id;
    msg_part.left = detections[object_index].center_x;
    msg_part.right = detections[object_index].center_y;
    msg_part.top = detections[object_index].width_half;
    msg_part.bot = detections[object_index].height_half;

    msg.bounding_boxes.push_back(msg_part);
  }

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  // Image
  cv_bridge::CvImagePtr cv_image = nullptr;

  pthread_mutex_lock(&mutex_image);

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  while (this->image_queue_.size())
  {
    int queued_image_stamp = static_cast<int>(static_cast<long long int>(rclcpp::Time(this->image_queue_.front()->header.stamp).seconds() * 1000.0) % 60000ll);

    if (queued_image_stamp != detections.back().time)
    {
      // consume image
      if (queued_image_stamp > detections.back().time + 30000)
      {
        this->image_queue_.pop();
      }
      if (queued_image_stamp < detections.back().time)
      {
        this->image_queue_.pop();
      }
      if ((queued_image_stamp > detections.back().time) && (queued_image_stamp < detections.back().time + 30000))
      {
        break;
      }
    }
    else
    {
      cv_image = this->image_queue_.front();

      this->image_queue_.pop();
    }
  }

  pthread_mutex_unlock(&mutex_image);

  // Merge
  if (cv_image == nullptr)
  {
    // benchmark
    if (use_benchmark_) {
      this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
      this->file_ << static_cast<long long int>(0) << "\n";
    }

    return;
  }

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    this->file_ << static_cast<long long int>(1) << ",";
  }

  draw_image(cv_image, msg);

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
  }

  cv::imshow("Result image", cv_image->image);
  cv::waitKey(10);

  // benchmark
  if (use_benchmark_) {
    this->file_ << static_cast<long long int>(this->get_clock()->now().seconds() * 1000000.0) << ",";
    this->file_ << static_cast<long long int>(rclcpp::Time(cv_image->header.stamp).seconds() * 1000000.0) << "\n";
  }
}

void* Merger::receive_thread(void* arg)
{
  while (run_flag_)
  {
    static_cast<Merger*>(arg)->can_receive();
  }
  return nullptr;
}

void* Merger::show_thread(void* arg)
{
  while (run_flag_)
  {
    static_cast<Merger*>(arg)->can_show();
  }
  return nullptr;
}

void Merger::draw_image(cv_bridge::CvImagePtr cv_image, rtx_msg_interface::msg::BoundingBoxes& msg)
{
  for (size_t i = 0; i < msg.bounding_boxes.size(); i++) {
    // Get rectangle from 1 object
    cv::Rect r = cv::Rect(round(msg.bounding_boxes[i].left - msg.bounding_boxes[i].top),
                          round(msg.bounding_boxes[i].right - msg.bounding_boxes[i].bot),
                          round(2 * msg.bounding_boxes[i].top),
                          round(2 * msg.bounding_boxes[i].bot));

    // draw_box
    cv::rectangle(cv_image->image, r, cv::Scalar(0x27, 0xC1, 0x36), 2);

    // put id
    cv::putText(cv_image->image, std::to_string(static_cast<int>(msg.bounding_boxes[i].id)), cv::Point(r.x, r.y - 1), cv::FONT_HERSHEY_PLAIN, 1.2, cv::Scalar(0xFF, 0xFF, 0xFF), 2);
  }
}

// benchmark
void Merger::benchmark()
{
  use_benchmark_ = this->declare_parameter("use_benchmark", true);

  if (use_benchmark_)
  {
    time_t raw_time;
    struct tm* pTime_info;

    raw_time = time(NULL);
    pTime_info = localtime(&raw_time);

    std::string simulation_time = std::to_string(pTime_info->tm_mon + 1) + "_" + std::to_string(pTime_info->tm_mday) + "_" + std::to_string(pTime_info->tm_hour) + "_" + std::to_string(pTime_info->tm_min) + "_" + std::to_string(pTime_info->tm_sec);
    std::string directory = "./data/merge/" + simulation_time + ".csv";

    this->file_.open(directory.c_str(), std::ios_base::out | std::ios_base::app);

    this->file_ << "beforegetdetections,aftergetdetections,beforeselectimage,afterselectimage,isshowimage,afterdrawimage,endpoint,timestamp\n";
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

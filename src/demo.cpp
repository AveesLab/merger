#include "monitor/demo.hpp"
#include <iomanip>
#include <stdint.h>

int cluster_num=0;
int draw_num=0;
vector<uint64_t> end_ethernet;
vector<uint64_t> e_merge;
vector<uint64_t> start_merge;
vector<uint64_t> end_merge;
vector<uint64_t> e_draw;
vector<uint64_t> start_draw;
vector<uint64_t> end_draw;
vector<uint64_t> e_display;
vector<uint64_t> start_display;
vector<uint64_t> end_display;
vector<uint64_t> node_index;
vector<uint64_t> node_end_ethernet;
vector<uint64_t> status;
vector<uint64_t> e_waiting_all_received;
vector<uint64_t> start_waiting_all_received;
vector<uint64_t> end_waiting_all_received;



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


  cv::Mat loaded_image = cv::imread("/home/avees/RTCSA_2024/src/merger/data/0316.jpg", cv::IMREAD_COLOR);
  
  if(loaded_image.empty()) {
  RCLCPP_ERROR(this->get_logger(), "Failed to load image.");
  return;
  }
  
  auto tmp_cv_image = std::make_shared<cv_bridge::CvImage>();
  tmp_cv_image->image = loaded_image;
  tmp_cv_image->encoding = "bgr8";
  /*cv_bridge::CvImagePtr tmp_cv_image = cv_bridge::toCvCopy(*image, image->encoding);

  if (tmp_cv_image->encoding == "bayer_rggb8")
  {
    cv::Mat rgb8_image;
    cv::cvtColor(tmp_cv_image->image, rgb8_image, cv::COLOR_BayerRG2RGB);
    cv::swap(tmp_cv_image->image, rgb8_image);
    
    tmp_cv_image->encoding = "rgb8";
  }*/

  // Append a image on queue
  pthread_mutex_lock(&mutex_image);
  this->result_image_ = tmp_cv_image;
  pthread_mutex_unlock(&mutex_image);
}

uint64_t MonitorDemo::get_time_in_ms() {
  rclcpp::Time now = this->get_clock()->now();
  uint64_t nanosecond = now.nanoseconds();
  return nanosecond/1000;
}

void MonitorDemo::detections_receive(const vision_msgs::msg::Detection2DArray::SharedPtr detections)
{

  pthread_mutex_lock(&mutex_receive);
  // waiting_all_received
  if (all_of(detections_received.begin(), detections_received.end(), [](bool received) { return !received; })) {
  	start_waiting_all_received.push_back(get_time_in_ms());
  }
  int frame_id = stoi(detections->header.frame_id);
  node_index.push_back(frame_id);
  node_end_ethernet.push_back(get_time_in_ms());
  

  RCLCPP_INFO(this->get_logger(), "node_index: %s, num_detection: %u, Computing_nodes_timestamp : %2f", detections->header.frame_id.c_str(), detections->detections.size(), rclcpp::Time(detections->header.stamp).seconds());
  
  detection_list[frame_id - 1] = detections;
  
  if (frame_id >= 1 && frame_id <= TOTAL_NUM_OF_NODES) {
	detections_received[frame_id - 1] = true; // Mark as received
	status.push_back(0);
	
	all_received = all_of(detections_received.begin(), detections_received.end(), [](bool received) { return received; });
	
	if (all_received) {
		end_ethernet.push_back(node_end_ethernet.back());
		status.pop_back();
		status.push_back(1);

		cerr << draw_num << "All node are received!" << endl;
		end_waiting_all_received.push_back(get_time_in_ms());
	}
  }
  
  
  pthread_mutex_unlock(&mutex_receive);
  

  // All nodes is received
  pthread_mutex_lock(&mutex_receive_check);

  if (all_received) {
	  
	// merge with clustering 
	start_merge.push_back(get_time_in_ms());

	pthread_mutex_lock(&mutex_image);

	cv_bridge::CvImagePtr cv_image = nullptr;	
	cv_image = this->result_image_;
	if (cv_image == nullptr)
	{
	return;
	}
	pthread_mutex_unlock(&mutex_image);


	vector<BoundingBox> partial_car_bboxes;
	filterDetections(detection_list, partial_car_bboxes);
	merge_bbox_with_clustering(partial_car_bboxes);

	end_merge.push_back(get_time_in_ms());


	// Draw bounding boxes
	start_draw.push_back(get_time_in_ms());
	
	for (int node_id = 0; node_id < TOTAL_NUM_OF_NODES; node_id++){
		draw_image(cv_image, detection_list[node_id]);
	}

	// draw_map(cv_image, clusterBoxes);
	for (const auto& pair : clusterBoxes) {
	cv::rectangle(cv_image->image, pair.second, cv::Scalar(0, 0, 255), 2);
	}

	end_draw.push_back(get_time_in_ms());


	// Show image
	start_display.push_back(get_time_in_ms());

	cv::imshow("After merge image", cv_image->image);
	cv::waitKey(1);

	// Save the result image
	//save_img(cv_image);
	
	end_display.push_back(get_time_in_ms());

	// Initialize all values
	fill(detections_received.begin(), detections_received.end(), false);
	// detection_list.clear();
	labels.clear();
	clusterBoxes.clear();


	draw_num++;
	
	if(draw_num == EXP_NUM) {
	     	for(int i=0 ; i<EXP_NUM; i++){		
			e_waiting_all_received.push_back(end_waiting_all_received[i] - start_waiting_all_received[i]);
			e_merge.push_back(end_merge[i] - start_merge[i]);
			e_draw.push_back(end_draw[i] - start_draw[i]);
			e_display.push_back(end_display[i]-start_display[i]);
		}
		
		std::ofstream file1("master_node.csv");

		file1 << std::fixed << std::setprecision(6) 
			<< "start_waiting_all_received," << "e_waiting_all_received(us)," << "end_waiting_all_received,"
			<< "end_ethernet," 
			<< "start_merge," << "e_merge(us)," << "end_merge,"
			<< "start_draw," << "e_draw(us)," << "end_draw,"
			<< "start_display," << "e_display(us)," << "end_display\n" ;

		for (int i=0;i<draw_num;i++){
			file1 << start_waiting_all_received[i]<<"," << e_waiting_all_received[i] << "," << end_waiting_all_received[i]
				<< "," << end_ethernet[i] << "," << start_merge[i] << "," << e_merge[i] << "," << end_merge[i] 
				<< "," << start_draw[i] << "," << e_draw[i] << "," << end_draw[i] 
				<< "," << start_display[i] << "," << e_display[i] << "," << end_display[i] << "\n";
		}
		
		file1.close();

		std::ofstream file2("master_node_ethernet.csv");
		
		file2 << std::fixed << std::setprecision(6) << "node_index," << "end_ethernet,"<<"status\n" ;
		
		for (int i=0;i<draw_num*TOTAL_NUM_OF_NODES;i++){
			file2 << node_index[i] << "," << node_end_ethernet[i] <<","<< status[i] << "\n";
		}
		
		file2.close();
		
		cerr << "write result at " << "./master_node.csv" << endl;
		cerr << "write result at " << "./master_node_ethernet.csv" << endl;
		
		exit(0);
	}


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

void MonitorDemo::filterDetections(const std::vector<vision_msgs::msg::Detection2DArray::SharedPtr> detection_list,
                      std::vector<BoundingBox>& partial_car_bboxes) {
    for (const auto& detections : detection_list) { // detection_list 순회
        for (const auto& detection : detections->detections) { // 각 Detection2DArray 내의 Detection2D 순회
            if (detection.tracking_id == "1") { // tracking_id가 "1"인 경우
                // Detection2D의 바운딩 박스 정보를 BoundingBox 구조체로 변환                                                                                                         
                BoundingBox box;
                box.centerX = detection.bbox.center.x;
                box.centerY = detection.bbox.center.y;
                box.width = 2 * detection.bbox.size_x; // width는 size_x의 2배
                box.height = 2 * detection.bbox.size_y; // height는 size_y의 2배

                // 변환된 BoundingBox 구조체를 partial_car_bboxes 벡터에 추가
                partial_car_bboxes.push_back(box);
            }
        }
    }
}


void MonitorDemo::simpleDBSCAN(const std::vector<BoundingBox> partial_car_bboxes, double eps, int minPts) {
    
    int clusterId = 0;
    labels.resize(partial_car_bboxes.size(), -1);

    for (size_t i = 0; i < partial_car_bboxes.size(); ++i) {
        if (labels[i] != -1) continue;

        std::vector<size_t> neighbors;

        for (size_t j = 0; j < partial_car_bboxes.size(); ++j) {
            double distance = std::sqrt(std::pow(partial_car_bboxes[i].centerX - partial_car_bboxes[j].centerX, 2) + std::pow(partial_car_bboxes[i].centerY - partial_car_bboxes[j].centerY, 2));
            if (distance <= eps) {
                neighbors.push_back(j);
            }
        }

        if (neighbors.size() < minPts) continue;
        for (auto idx : neighbors) {
            labels[idx] = clusterId;
        }
        clusterId++;
    }

}

void MonitorDemo::merge_bbox_with_clustering(const vector<BoundingBox> partial_car_bboxes)
{
    
    double eps = 300.0;
    int minPts = 2; 
    
    simpleDBSCAN(partial_car_bboxes, eps, minPts);

 
    
    for (size_t i = 0; i < partial_car_bboxes.size(); ++i) {
        if (labels[i] == -1) continue;




        cv::Point top_left(partial_car_bboxes[i].centerX - partial_car_bboxes[i].width / 2, partial_car_bboxes[i].centerY - partial_car_bboxes[i].height / 2);
        cv::Point bottom_right(partial_car_bboxes[i].centerX + partial_car_bboxes[i].width / 2, partial_car_bboxes[i].centerY + partial_car_bboxes[i].height / 2);
        
        cv::Rect currentBox(top_left, bottom_right);
        if (clusterBoxes.find(labels[i]) == clusterBoxes.end()) {
            clusterBoxes[labels[i]] = currentBox;
        } else {
            clusterBoxes[labels[i]] |= currentBox;
        }
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

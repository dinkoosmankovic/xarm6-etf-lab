#include "aruco_calibration.h"

#include <cmath>
#include <chrono>
#include <thread>
#include <opencv2/calib3d.hpp>
#include <iostream>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

using std::placeholders::_1;

ArucoCalibration::ArucoCalibration() : rclcpp::Node(kArucoCalibrationNodeName) 
{   
  
  tf_broadcaster_ =
      std::make_unique<tf2_ros::TransformBroadcaster>(*this);
	       
  subscription_image_raw_ = this->create_subscription<sensor_msgs::msg::Image>(
    kImageRawTopic, 10, std::bind(&ArucoCalibration::HandleImage, this, _1)
  );
  subscription_camera_info_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    kCameraInfoTopic, 10, std::bind(&ArucoCalibration::UpdateCameraInfo, this, _1)
  );  
}

void ArucoCalibration::HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg) 
{
  	RCLCPP_INFO(this->get_logger(), "HandleImage got message");
  	cv::Mat cv_image = cv_bridge::toCvCopy(msg)->image;
  	
  	try 
  	{
  		cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);
  		cv::aruco::detectMarkers(cv_image, dictionary, markerCorners, markerIds, parameters, rejectedCandidates);
  		RCLCPP_INFO(this->get_logger(), "HandleImage found %ld markers", markerIds.size());
  		
		std::vector<cv::Vec3d> rvecs, tvecs;
  		
  		//std::cout << "Matrix K: " << camera_matrix << std::endl;
  		//std::cout << "Vector d: " << dist_coeffs << std::endl;
  		
  		cv::aruco::estimatePoseSingleMarkers(markerCorners, 0.15, camera_matrix, dist_coeffs, rvecs, tvecs);
  		
  		//std::cout << "tvecs: " << tvecs[0] << std::endl;
  		cv::Mat R(3, 3, CV_64FC1);
		cv::Rodrigues(rvecs[0], R); 
		
		if (std::isnan(R.at<double>(0, 0)))
			return;
	
		cv::Mat camR = R.t();  
		
		cv::Mat tvec_cam;
    	tvec_cam = - camR * tvecs[0]; // translation of inverse 
    	
    	//std::cout << "Matrix R: " << R << std::endl;
    	tf2::Matrix3x3 tf2_mat(R.at<double>(0, 0), R.at<double>(0, 1), R.at<double>(0, 2),
                       	   	R.at<double>(1, 0), R.at<double>(1, 1), R.at<double>(1, 2),
                           	R.at<double>(2, 0), R.at<double>(2, 1), R.at<double>(2, 2));
                           	
    	
    	geometry_msgs::msg::TransformStamped t;
    	t.header.stamp = this->get_clock()->now();
    	t.header.frame_id = kCameraFrame;
    	t.child_frame_id = "aruco_marker";
    	
    	double roll, pitch, yaw;
    	tf2_mat.getRPY(roll, pitch, yaw);
    	
    	
    	tf2::Quaternion q;
    	q.setRPY(roll, pitch, yaw);
    	t.transform.rotation.x = q.x();
    	t.transform.rotation.y = q.y();
    	t.transform.rotation.z = q.z();
    	t.transform.rotation.w = q.w();
    	
    	t.transform.translation.x = tvecs[0][0];
    	t.transform.translation.y = tvecs[0][1];
    	t.transform.translation.z = tvecs[0][2];
    	
    	RCLCPP_INFO(this->get_logger(),
                  "[Tf2ListenerExample]: 'child_frame' -> 'parent_frame':\n\t translation: [%.2f, %.2f, %.2f]\n\t rotation: [%.2f, %.2f, %.2f, %.2f]",
                  t.transform.translation.x, t.transform.translation.y, t.transform.translation.z,
                  t.transform.rotation.w, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z);
	
    	// Send the transformation
    	tf_broadcaster_->sendTransform(t);
    	//RCLCPP_INFO(this->get_logger(), "Transform sent!");
    }
    catch (cv::Exception& e) 
    {
    	RCLCPP_INFO(this->get_logger(), "Exception: %s", e.err.c_str());    	
    }
}

void ArucoCalibration::UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg) {
	//RCLCPP_INFO(this->get_logger(), "UpdateCameraInfo got message");
  	camera_matrix = cv::Mat(3, 3, CV_64FC1, (void *) msg->k.data());
  	dist_coeffs = cv::Mat(1, 5, CV_64FC1, (void *) msg->d.data());
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ArucoCalibration>());
  rclcpp::shutdown();
  return 0;
}
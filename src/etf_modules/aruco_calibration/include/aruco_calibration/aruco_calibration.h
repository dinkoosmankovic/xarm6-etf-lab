#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <tf2_ros/transform_broadcaster.h>


//this node deal with two topics:
// /camera/color/camera_info [sensor_msgs/msg/CameraInfo]
// /camera/color/image_raw [sensor_msgs/msg/Image]

//const static std::string kImageRawTopic = "/camera_left/color/image_raw";
//const static std::string kCameraInfoTopic = "/camera_left/color/camera_info";
const static std::string kArucoCalibrationNodeName = "aruco_localization";
//const static std::string kCameraFrame = "camera_left_color_optical_frame";

class ArucoCalibration : public rclcpp::Node
{
public:
  ArucoCalibration();

private:
  void HandleImage(sensor_msgs::msg::Image::ConstSharedPtr msg);
  void UpdateCameraInfo(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg);

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_image_raw_;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscription_camera_info_;

  std::vector<int> markerIds;
  std::vector<std::vector<cv::Point2f>> markerCorners, rejectedCandidates;
  cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(cv::aruco::DICT_6X6_250);

  cv::Mat camera_matrix, dist_coeffs;
  
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  
  double marker_size;
  std::string camera_name;
  std::string kImageRawTopic;
  std::string kCameraInfoTopic;
  std::string kCameraFrame;

};

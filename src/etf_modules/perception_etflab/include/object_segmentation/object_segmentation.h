#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "pcl/common/transforms.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"


using namespace std::chrono_literals;

class ObjectSegmentation : public rclcpp::Node
{
	public:
		ObjectSegmentation();
	
	private:
		void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
		void publish_objects_point_cloud(pcl::PCLPointCloud2::Ptr output_cloud);	
		void publish_objects();
		
		rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
		rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher;
		
		std::string input_cloud;
		std::string objects_cloud;

};

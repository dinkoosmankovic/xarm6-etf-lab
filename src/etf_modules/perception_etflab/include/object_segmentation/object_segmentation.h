#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "pcl_conversions/pcl_conversions.h"
#include "geometry_msgs/msg/transform_stamped.h"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "pcl/common/transforms.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_ros/transform_listener.h"
#include "rclcpp/rclcpp.hpp"

#include <Scenario.h>


using namespace std::chrono_literals;

class ObjectSegmentation : public rclcpp::Node
{
public:
	ObjectSegmentation();

private:
	std::string scenario_file_path = "/sim_bringup/data/scenario_etf_lab.yaml";	
	std::shared_ptr<scenario::Scenario> scenario;
	std::shared_ptr<robots::AbstractRobot> robot;
	std::shared_ptr<Eigen::MatrixXf> skeleton;
	std::shared_ptr<base::State> joint_states;	
	std::string input_cloud;
	std::string objects_cloud;

	rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
	rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr object_point_cloud_publisher;

	void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void joint_states_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
	void publish_objects_point_cloud(pcl::PCLPointCloud2::Ptr output_cloud);	
	void publish_objects();
	void removePointsOccupiedByRobot(pcl::PointCloud<pcl::PointXYZRGB> &pcl);
};

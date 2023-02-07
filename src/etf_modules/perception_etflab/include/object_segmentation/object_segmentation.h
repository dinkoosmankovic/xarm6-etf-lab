#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/transform_stamped.h>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <rclcpp/rclcpp.hpp>

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
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_publisher;
	rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_polygons_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_free_cells_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_occupied_cells_publisher;

	void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
	void publishObjectsPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
	void publishBoundingBoxes(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clusters);
	void publishConvexHulls(std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> &pcl_clusters);
	void removePointsOccupiedByRobot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, int tolerance_factor = 2);
	void visualizeOutputPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl);
	void visualizeBoundingBoxes(pcl::PointCloud<pcl::PointXYZ>::Ptr bounding_boxes);
	void visualizeConvexHulls(pcl::PointCloud<pcl::PointXYZRGB>::Ptr convex_hulls_points);
	void visualizeRobotCapsules();
    void visualizeRobotSkeleton();
};

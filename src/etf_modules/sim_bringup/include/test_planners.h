#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <octomap_msgs/srv/get_octomap.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <fcl/fcl.h>

#include <AbstractPlanner.h>
#include <AbstractRobot.h>
#include <RBTConnect.h>
#include <RGBTConnect.h>
#include <xArm6.h>
#include <Scenario.h>

using namespace std::chrono_literals;

class TestPlannersNode : public rclcpp::Node
{
public:
    TestPlannersNode();

private:
    std::string scenario_file_path = "/data/scenario_etf_lab.yaml";	
    std::string project_path;

    std::shared_ptr<scenario::Scenario> scenario;
    std::shared_ptr<robots::AbstractRobot> robot;
	std::vector<std::shared_ptr<base::State>> path;
	trajectory_msgs::msg::JointTrajectory trajectory;
    octomap::OcTree* octomap_octree;
	std::shared_ptr<fcl::OcTreef> octree;
    std::vector<fcl::Vector3f> bounding_boxes;
    std::vector<std::vector<fcl::Vector3f>> convex_hulls;
    std::vector<std::vector<int>> convex_hulls_polygons;

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr convex_hulls_polygons_subscription;

    void testPlannersCallback();
    void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convexHullsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convexHullsPolygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
	void readOctree();
    void updateEnvironment();
	void planPath();
	void publishTrajectory(int init_time, int delta_time);
    void visualizeOctreeBoxes();
    
};
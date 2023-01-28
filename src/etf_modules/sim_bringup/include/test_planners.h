#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "control_msgs/msg/joint_trajectory_controller_state.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "fcl/fcl.h"

#include <AbstractPlanner.h>
#include <AbstractRobot.h>
#include <RBTConnect.h>
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

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_array_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;

	void readOctree();
    void updateEnvironment();
	void planPath();
	void publishTrajectory(int init_time, int delta_time);
    void visualizeOctreeBoxes();
    void joint_states_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void test_planners_callback();
    
};
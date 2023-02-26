//
// Created by nermin on 25.02.23.
//

#include <rclcpp/rclcpp.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory_point.hpp>
#include <control_msgs/msg/joint_trajectory_controller_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <fcl/fcl.h>

#include <xArm6.h>
#include <Scenario.h>
#include <RealTimeDRGBTConnect.h>

using namespace std::chrono_literals;

class TestRealTimeDRGBTConnectNode : public rclcpp::Node
{
public:
    TestRealTimeDRGBTConnectNode();

private:
    std::string scenario_file_path = "/data/scenario_etf_lab.yaml";	
    std::string project_path;

    std::unique_ptr<planning::drbt::RealTimeDRGBTConnect> planner;
    std::shared_ptr<scenario::Scenario> scenario;
    std::shared_ptr<robots::AbstractRobot> robot;
	std::vector<std::shared_ptr<base::State>> path;
	trajectory_msgs::msg::JointTrajectory trajectory;
    std::vector<fcl::Vector3f> bounding_boxes;

    rclcpp::TimerBase::SharedPtr timer_computing_next_state;
    rclcpp::TimerBase::SharedPtr timer_replanning;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;
    rclcpp::Subscription<control_msgs::msg::JointTrajectoryControllerState>::SharedPtr joint_states_subscription;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr bounding_boxes_subscription;

    void computingNextState();
    void replanning();
    void jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg);
    void boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void updateEnvironment();
	void publishTrajectory(int init_time, int delta_time);
    
};

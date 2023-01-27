#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

using namespace std::chrono_literals;

class TestPlannersNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_publisher;

public:
    TestPlannersNode() : Node("trajectory_publisher_node")
    {
        trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
        timer = this->create_wall_timer(5s, std::bind(&TestPlannersNode::test_planners_callback, this));
    }

private:
    void test_planners_callback()
    {
        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
        
        trajectory_msgs::msg::JointTrajectoryPoint point1;
        point1.positions = {0, 0, 0, 0, 0, 0};
        point1.time_from_start.sec = 1;

        trajectory_msgs::msg::JointTrajectoryPoint point2;
        point2.positions = {1.56, 0, 0, 0, 0, 0};
        point2.time_from_start.sec = 2;

        trajectory.points.emplace_back(point1);
        trajectory.points.emplace_back(point2);

        std::cout << "Publishing trajectory ...\n";
        trajectory_publisher->publish(trajectory);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPlannersNode>());
    rclcpp::shutdown();
    return 0;
}
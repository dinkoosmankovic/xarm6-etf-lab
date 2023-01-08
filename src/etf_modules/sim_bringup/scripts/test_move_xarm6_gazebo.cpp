#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"

#include <AbstractPlanner.h>
#include <RBTConnect.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

using namespace std::chrono_literals;

class TestJointTrajectoryNode : public rclcpp::Node
{
private:
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher_;

public:
    TestJointTrajectoryNode() : Node("trajectory_publisher_node")
    {
        publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
        timer_ = this->create_wall_timer(30s, std::bind(&TestJointTrajectoryNode::timer_callback, this));
    }

private:
    void timer_callback()
    {
        std::srand((unsigned int) time(0));
        FLAGS_logtostderr = true;
        LOG(INFO) << "GLOG successfully initialized!";

        std::string root_path = "/home/nermin/RPMPLv2";
        std::string scenario_file_path = root_path + "/data/xarm6/scenario_test/scenario_test1.yaml";
        // std::string scenario_file_path = root_path + "/data/xarm6/scenario1/scenario1.yaml";
        // std::string scenario_file_path = root_path + "/data/xarm6/scenario2/scenario2.yaml";

        ConfigurationReader::initConfiguration(root_path);
        scenario::Scenario scenario(scenario_file_path, root_path);
        std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
        std::vector<std::shared_ptr<base::State>> path;
            
        LOG(INFO) << "Using scenario: " << scenario_file_path;
        LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
        LOG(INFO) << "Dimensions: " << ss->getDimensions();
        LOG(INFO) << "State space type: " << ss->getStateSpaceType();
        LOG(INFO) << "Start: " << scenario.getStart();
        LOG(INFO) << "Goal: " << scenario.getGoal();

        try
        {
            std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, scenario.getStart(), scenario.getGoal());
            bool res = planner->solve();
            LOG(INFO) << "RBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
            LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
            LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
                
            if (res)
            {
                path = planner->getPath();
                for (int i = 0; i < path.size(); i++)
                    std::cout << path.at(i)->getCoord().transpose() << std::endl;
            }
            planner->outputPlannerData(scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log");

        }
        catch (std::domain_error &e)
        {
            LOG(ERROR) << e.what();
        }

        trajectory_msgs::msg::JointTrajectory trajectory;
        trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

        int time = 10;
        for (std::shared_ptr<base::State> q : path)
        {
            trajectory_msgs::msg::JointTrajectoryPoint point;
            for (int i = 0; i < q->getDimensions(); i++)
                point.positions.emplace_back(q->getCoord(i));
            point.time_from_start.sec = time;
            time += 1;
            trajectory.points.emplace_back(point);
        }
        
        // trajectory_msgs::msg::JointTrajectoryPoint point1;
        // point1.positions = {0, 0, 0, 0, 0, 0};
        // point1.time_from_start.sec = 1;

        // trajectory_msgs::msg::JointTrajectoryPoint point2;
        // point2.positions = {1.56, 0, 0, 0, 0, 0};
        // point2.time_from_start.sec = 2;

        // trajectory.points.emplace_back(point1);
        // trajectory.points.emplace_back(point2);

        LOG(INFO) << "Publishing trajectory ...";
        publisher_->publish(trajectory);
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestJointTrajectoryNode>());
    rclcpp::shutdown();
    return 0;
}
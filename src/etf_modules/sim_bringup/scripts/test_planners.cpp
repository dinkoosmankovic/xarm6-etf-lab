#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory_point.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "octomap_msgs/conversions.h"
#include "octomap/octomap.h"
#include "fcl/fcl.h"

#include <AbstractPlanner.h>
#include <RBTConnect.h>
#include <Scenario.h>
#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

using namespace std::chrono_literals;

class TestPlannersNode : public rclcpp::Node
{
private:
    std::string root_path = "/home/nermin/RPMPLv2";
    std::string scenario_file_path = root_path + "/data/xarm6/scenario_test/scenario_test1.yaml";
    // std::string scenario_file_path = root_path + "/data/xarm6/scenario1/scenario1.yaml";
    // std::string scenario_file_path = root_path + "/data/xarm6/scenario2/scenario2.yaml";
    scenario::Scenario scenario = scenario::Scenario(scenario_file_path, root_path);

    rclcpp::TimerBase::SharedPtr timer;
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr publisher;
	std::shared_ptr<fcl::OcTreef> octree;
	std::vector<std::shared_ptr<base::State>> path;
	trajectory_msgs::msg::JointTrajectory trajectory;

public:
    TestPlannersNode() : Node("test_planners_node")
    {
        publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
        timer = this->create_wall_timer(30s, std::bind(&TestPlannersNode::timer_callback, this));

        ConfigurationReader::initConfiguration(root_path);
		trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
    }

private:
	void readOctree()
	{
		std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_read_octree_node");
        rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client = 
            node->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");        
        auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
        
        while (!client->wait_for_service(1s)) 
        {
            if (!rclcpp::ok()) 
            {
                RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
        }

        // Wait for the result
        auto result = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
        {
            octomap_msgs::msg::Octomap octomap_msg = result.get()->map;
            octomap::AbstractOcTree* octomap_abstract_octree = octomap_msgs::msgToMap(octomap_msg);
            octomap::OcTree* octomap_octree = dynamic_cast<octomap::OcTree*>(octomap_abstract_octree);

            fcl::OcTreef Octree(std::make_shared<const octomap::OcTree>(*octomap_octree));
			octree = std::make_shared<fcl::OcTreef>(Octree);            
            
            RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octree read successfully. ");
        }
        else
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read octree!");
	}

	void planPath(const std::vector<std::shared_ptr<fcl::CollisionObjectf>> &parts)
	{
        std::shared_ptr<base::StateSpace> ss = scenario.getStateSpace();
        std::shared_ptr<base::State> start = scenario.getStart();
        std::shared_ptr<base::State> goal = scenario.getGoal();
        scenario.setEnvironment(parts);
        
        LOG(INFO) << "Using scenario: " << scenario_file_path;
        LOG(INFO) << "Environment parts: " << scenario.getEnvironment()->getParts().size();
        LOG(INFO) << "Dimensions: " << ss->getDimensions();
        LOG(INFO) << "State space type: " << ss->getStateSpaceType();
        LOG(INFO) << "Start: " << start;
        LOG(INFO) << "Goal: " << goal;

        try
        {
            std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, start, goal);
            bool res = planner->solve();
            LOG(INFO) << "RBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
            LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
            LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";
                
            if (res)
            {
                path = planner->getPath();
                for (int i = 0; i < path.size(); i++)
                    std::cout << path.at(i)->getCoord().transpose() << std::endl;

				// Swap start and goal
                // scenario.setStart(goal);
                // scenario.setGoal(start);
            }
            planner->outputPlannerData(scenario_file_path.substr(0, scenario_file_path.size()-5) + "_planner_data.log");

        }
        catch (std::domain_error &e)
        {
            LOG(ERROR) << e.what();
        }
	}

	void publishTrajectory(int init_time, int delta_time)
	{
		trajectory.points.clear();

        int time = init_time;
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (std::shared_ptr<base::State> q : path)
        {
            point.positions.clear();
            for (int i = 0; i < q->getDimensions(); i++)
                point.positions.emplace_back(q->getCoord(i));

            point.time_from_start.sec = time;
            trajectory.points.emplace_back(point);
            time += delta_time;
        }

        // Go back to home
        point.positions = {0, 0, 0, 0, 0, 0};
        point.time_from_start.sec = time;
        trajectory.points.emplace_back(point);

        LOG(INFO) << "Publishing trajectory ...\n";
        publisher->publish(trajectory);
	}

    void timer_callback()
    {
		readOctree();

		std::shared_ptr<fcl::CollisionGeometryf> octree_ = std::make_shared<fcl::OcTreef>(*octree);
        std::vector<std::shared_ptr<fcl::CollisionObjectf>> parts = {std::make_shared<fcl::CollisionObjectf>(octree_)};

		planPath(parts);        

        publishTrajectory(0, 1);

        LOG(INFO) << "-----------------------------------------------------------------------\n";
    }
};

int main(int argc, char * argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestPlannersNode>());
    rclcpp::shutdown();
    return 0;
}
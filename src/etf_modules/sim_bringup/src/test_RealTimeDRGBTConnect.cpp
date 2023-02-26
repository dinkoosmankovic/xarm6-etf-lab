//
// Created by nermin on 25.02.23.
//

#include "test_RealTimeDRGBTConnect.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <fcl/geometry/shape/convex.h>

#include <ConfigurationReader.h>
#include <CommandLine.h>
#include <glog/logging.h>

TestRealTimeDRGBTConnectNode::TestRealTimeDRGBTConnectNode() : Node("test_RealTimeDRGBTConnect_node")
{
    timer_computing_next_state = this->create_wall_timer(1s, std::bind(&TestRealTimeDRGBTConnectNode::computingNextState, this));
    timer_replanning = this->create_wall_timer(10s, std::bind(&TestRealTimeDRGBTConnectNode::replanning, this));
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&TestRealTimeDRGBTConnectNode::jointStatesCallback, this, std::placeholders::_1));
    bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&TestRealTimeDRGBTConnectNode::boundingBoxesCallback, this, std::placeholders::_1));
    
    std::string project_path_(__FILE__);
    for (int i = 0; i < 2; i++)
        project_path_ = project_path_.substr(0, project_path_.find_last_of("/\\"));
    project_path = project_path_;
    scenario_file_path = project_path + scenario_file_path;
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path);
    robot = scenario->getRobot();
    planner = std::make_unique<planning::drbt::RealTimeDRGBTConnect>(scenario->getStateSpace(), scenario->getStart(), scenario->getGoal());

    YAML::Node node = YAML::LoadFile(scenario_file_path);
    ConfigurationReader::initConfiguration(node["configurations"].as<std::string>());
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};

    auto time_alg_start = std::chrono::steady_clock::now();     // Start the algorithm clock
    auto time_iter_start = time_alg_start;                      // Start the iteration clock
    auto time_current = time_alg_start;

    // Obtaining the inital path using RGBTConnect (any planner can be used)
    planner->replanning();
}

void TestRealTimeDRGBTConnectNode::computingNextState()
{
    
    RCLCPP_INFO(this->get_logger(), "--------------------------------computingNextState finished--------------------------------\n");
}

void TestRealTimeDRGBTConnectNode::replanning()
{

    RCLCPP_INFO(this->get_logger(), "---------------------------------replanning finished-------------------------------\n");
}

void TestRealTimeDRGBTConnectNode::jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
    Eigen::VectorXf q(6);
    q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    robot->setConfiguration(scenario->getStateSpace()->newState(q));
    // RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void TestRealTimeDRGBTConnectNode::boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    bounding_boxes.clear();
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);
    for (int i = 0; i < pcl->size(); i += 2)
    {
        pcl::PointXYZ dim = pcl->points[i];
        pcl::PointXYZ trans = pcl->points[i+1];
        // RCLCPP_INFO(this->get_logger(), "Bounding-box %d: dim = (%f, %f, %f), trans = (%f, %f, %f)",
        //     i/2, dim.x, dim.y, dim.z, trans.x, trans.y, trans.z);
        bounding_boxes.emplace_back(fcl::Vector3f(dim.x, dim.y, dim.z));
        bounding_boxes.emplace_back(fcl::Vector3f(trans.x, trans.y, trans.z));
    }
}

void TestRealTimeDRGBTConnectNode::updateEnvironment()
{
    std::vector<std::shared_ptr<fcl::CollisionObjectf>> col_obj;

    // Table
    fcl::Vector3f tr(0, 0, -0.05);
    Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
    std::shared_ptr<fcl::CollisionGeometryf> table = std::make_shared<fcl::Cylinderf>(0.75, 0.1);
    col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(table, rot, tr));

    // Bounding-boxes
    for (int i = 0; i < bounding_boxes.size(); i += 2)
    {
        std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(bounding_boxes[i]);
        col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(box, rot, bounding_boxes[i+1]));
    }

    scenario->setEnvironment(col_obj);
}

void TestRealTimeDRGBTConnectNode::publishTrajectory(int init_time, int delta_time)
{
    trajectory.points.clear();

    int time = init_time;
    for (std::shared_ptr<base::State> q : path)
    {
        trajectory_msgs::msg::JointTrajectoryPoint point;
        for (int i = 0; i < q->getDimensions(); i++)
            point.positions.emplace_back(q->getCoord(i));

        point.time_from_start.sec = time;
        trajectory.points.emplace_back(point);
        time += delta_time;
    }

    if (path.size() > 0)
    {
        RCLCPP_INFO(this->get_logger(), "Publishing trajectory ...\n");
        trajectory_publisher->publish(trajectory);
    }
    else
        RCLCPP_INFO(this->get_logger(), "There is no trajectory to publish!\n");
}

int main(int argc, char *argv[])
{
	google::InitGoogleLogging(argv[0]);
	std::srand((unsigned int) time(0));
	FLAGS_logtostderr = true;
	LOG(INFO) << "GLOG successfully initialized!";

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestRealTimeDRGBTConnectNode>());
    rclcpp::shutdown();
    return 0;
}
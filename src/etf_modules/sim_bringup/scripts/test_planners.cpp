#include "test_planners.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include <ConfigurationReader.h>
#include <RealVectorSpace.h>
#include <CommandLine.h>
#include <glog/logging.h>


TestPlannersNode::TestPlannersNode() : Node("test_planners_node")
{
    timer = this->create_wall_timer(15s, std::bind(&TestPlannersNode::test_planners_callback, this));
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);
    
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&TestPlannersNode::joint_states_callback, this, std::placeholders::_1));

    std::string project_path_(__FILE__);
    for (int i = 0; i < 2; i++)
        project_path_ = project_path_.substr(0, project_path_.find_last_of("/\\"));
    project_path = project_path_;
    scenario_file_path = project_path + scenario_file_path;
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path);
    robot = scenario->getRobot();
    skeleton = robot->computeSkeleton(scenario->getStart());

    YAML::Node node = YAML::LoadFile(scenario_file_path);
    ConfigurationReader::initConfiguration(node["configurations"].as<std::string>());
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

void TestPlannersNode::readOctree()
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
        octomap::AbstractOcTree* octomap_abstract_octree = octomap_msgs::binaryMsgToMap(octomap_msg);
        octomap_octree = dynamic_cast<octomap::OcTree*>(octomap_abstract_octree);

        removeNodesOccupiedByRobot();

        fcl::OcTreef Octree(std::make_shared<const octomap::OcTree>(*octomap_octree));
        octree = std::make_shared<fcl::OcTreef>(Octree);            
        
        std::vector<std::array<float, 6>> boxes = octree->toBoxes();
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octree read successfully with %d occupied boxes.", boxes.size());

        for (int i = 0; i < boxes.size(); i++)
            LOG(INFO) << "Box "<< i << ": (" << boxes[i][0] << ", " << boxes[i][1] << ", " << boxes[i][2] << ")";
        // publishMarkerArray(boxes);
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read octree!");
}

void TestPlannersNode::removeNodesOccupiedByRobot()
{
    int tolerance_factor = 1.5, i = 0;
    for (octomap::OcTree::leaf_iterator node = octomap_octree->begin_leafs(), end = octomap_octree->end_leafs(); node != end; node++)
    {
        if (octomap_octree->isNodeOccupied(*node))
        {
            // bool occupied = false;
            // for (std::shared_ptr<base::State> q : path)
            // {
                skeleton = robot->computeSkeleton(joint_states);
                octomap::point3d node_pos_ = node.getCoordinate();
                Eigen::Vector3f node_pos(node_pos_.x(), node_pos_.y(), node_pos_.z());
                LOG(INFO) << "Box " << i++ << ": " << node_pos.transpose();
                for (int k = 0; k < robot->getParts().size(); k++)
                {
                    float d_c = std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(skeleton->col(k), skeleton->col(k+1), node_pos));
                    if (d_c < robot->getRadius(k) * tolerance_factor)
                    // if (abs(node_pos(0)) < 0.5 && abs(node_pos(1)) < 0.5)
                    {
                        octomap_octree->updateNode(node.getIndexKey(), false);
                        octomap_octree->setNodeValue(node.getIndexKey(), -10);    // It corresponds to the lowest probability
                        LOG(INFO) << "Removing this node!";
                        // occupied = true;
                        break;
                    }
                }
            //     if (occupied)
            //         break;
            // }
        }
    }
}

void TestPlannersNode::publishMarkerArray(std::vector<std::array<float, 6>> &boxes)
{
    visualization_msgs::msg::MarkerArray marker_array_msg;
    for (int i = 0; i < boxes.size(); i++) 
    {
        visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.header.frame_id = "world";
        marker.header.stamp = now();
        marker.pose.position.x = boxes[i][0];
        marker.pose.position.y = boxes[i][1];
        marker.pose.position.z = boxes[i][2];
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;
        marker_array_msg.markers.push_back(marker);
    }
    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing marker_array...");
}

void TestPlannersNode::updateEnvironment()
{
    // Table
    fcl::Vector3f tr(0, 0, -0.05);
    fcl::Quaternionf rot(0, 0, 0, 0);
    std::shared_ptr<fcl::CollisionGeometryf> table_ = std::make_shared<fcl::Cylinderf>(0.75, 0.1);

    // Octree contains all other objects (without table and robot)
    readOctree();
    std::shared_ptr<fcl::CollisionGeometryf> octree_ = std::make_shared<fcl::OcTreef>(*octree);

    std::vector<std::shared_ptr<fcl::CollisionObjectf>> col_obj = {
        std::make_shared<fcl::CollisionObjectf>(table_, rot.matrix(), tr),
        std::make_shared<fcl::CollisionObjectf>(octree_)
    };
    scenario->setEnvironment(col_obj);
}

void TestPlannersNode::planPath()
{
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    std::shared_ptr<base::State> start = scenario->getStart();
    std::shared_ptr<base::State> goal = scenario->getGoal();
    
    LOG(INFO) << "Environment col_obj: " << scenario->getEnvironment()->getParts().size();
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

        path = {};
        if (res)
        {
            path = planner->getPath();
            LOG(INFO) << "Found path is: ";
            for (std::shared_ptr<base::State> q : path)
                std::cout << q->getCoord().transpose() << std::endl;

            // Swap start and goal
            scenario->setStart(goal);
            scenario->setGoal(start);
        }
        planner->outputPlannerData(project_path + "/data/planner_data.log");

    }
    catch (std::domain_error &e)
    {
        LOG(ERROR) << e.what();
    }
}

void TestPlannersNode::publishTrajectory(int init_time, int delta_time)
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
    
void TestPlannersNode::joint_states_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
    Eigen::VectorXf q(6);
    q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    joint_states = scenario->getStateSpace()->newState(q);
    // RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void TestPlannersNode::test_planners_callback()
{
    updateEnvironment();

    planPath();

    publishTrajectory(0, 1);

    delete(octomap_octree);

    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------\n");
}

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
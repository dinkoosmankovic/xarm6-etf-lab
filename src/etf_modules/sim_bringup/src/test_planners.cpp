#include "test_planners.h"
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


TestPlannersNode::TestPlannersNode() : Node("test_planners_node")
{
    timer = this->create_wall_timer(20s, std::bind(&TestPlannersNode::testPlannersCallback, this));
    trajectory_publisher = this->create_publisher<trajectory_msgs::msg::JointTrajectory>("/xarm6_traj_controller/joint_trajectory", 10);
    marker_array_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);
    
    joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
        ("/xarm6_traj_controller/state", 10, std::bind(&TestPlannersNode::jointStatesCallback, this, std::placeholders::_1));
    bounding_boxes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/bounding_boxes", 10, std::bind(&TestPlannersNode::boundingBoxesCallback, this, std::placeholders::_1));
    convex_hulls_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/convex_hulls", 10, std::bind(&TestPlannersNode::convexHullsCallback, this, std::placeholders::_1));
    convex_hulls_polygons_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>
        ("/convex_hulls_polygons", 10, std::bind(&TestPlannersNode::convexHullsPolygonsCallback, this, std::placeholders::_1));

    std::string project_path_(__FILE__);
    for (int i = 0; i < 2; i++)
        project_path_ = project_path_.substr(0, project_path_.find_last_of("/\\"));
    project_path = project_path_;
    scenario_file_path = project_path + scenario_file_path;
    scenario = std::make_shared<scenario::Scenario>(scenario_file_path);
    robot = scenario->getRobot();

    YAML::Node node = YAML::LoadFile(scenario_file_path);
    ConfigurationReader::initConfiguration(node["configurations"].as<std::string>());
    trajectory.joint_names = {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

void TestPlannersNode::testPlannersCallback()
{
    updateEnvironment();

    planPath();

    publishTrajectory(0, 1);

    RCLCPP_INFO(this->get_logger(), "----------------------------------------------------------------\n");
}

void TestPlannersNode::jointStatesCallback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
    std::vector<double> positions = msg->actual.positions;
    Eigen::VectorXf q(6);
    q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
    robot->setConfiguration(scenario->getStateSpace()->newState(q));
    // RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void TestPlannersNode::boundingBoxesCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
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

void TestPlannersNode::convexHullsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    convex_hulls = {std::vector<fcl::Vector3f>()};
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZRGB>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    int j = 0;
    for (int i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZRGB P = pcl->points[i];
        if (P.x == 0.0 && P.y == 0.0 && P.z == 0.0)     // This point is just delimiter to distinguish clusters
        {
            convex_hulls.emplace_back(std::vector<fcl::Vector3f>());
            j++;
        }
        else
        {
            convex_hulls[j].emplace_back(fcl::Vector3f(P.x, P.y, P.z));
            // RCLCPP_INFO(this->get_logger(), "Convex-hull %d.\t Point: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
}

void TestPlannersNode::convexHullsPolygonsCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    convex_hulls_polygons = {std::vector<int>()};
	pcl::PointCloud<pcl::PointXYZ>::Ptr pcl(new pcl::PointCloud<pcl::PointXYZ>);	
	pcl::moveFromROSMsg(*msg, *pcl);

    int j = 0;
    for (int i = 0; i < pcl->size(); i++)
    {
        pcl::PointXYZ P = pcl->points[i];
        if (P.x == -1)     // This point is just delimiter to distinguish clusters
        {
            convex_hulls_polygons.emplace_back(std::vector<int>());
            j++;
        }
        else
        {
            convex_hulls_polygons[j].emplace_back(P.x);
            convex_hulls_polygons[j].emplace_back(P.y);
            convex_hulls_polygons[j].emplace_back(P.z);
            // RCLCPP_INFO(this->get_logger(), "Convex-hull %d.\t Polygon indices: (%f, %f, %f)", j, P.x, P.y, P.z);
        }
    }    
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

        fcl::OcTreef Octree(std::make_shared<const octomap::OcTree>(*octomap_octree));
        octree = std::make_shared<fcl::OcTreef>(Octree);            
        
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Octree read successfully!");
        visualizeOctreeBoxes();
    }
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to read octree!");
}

void TestPlannersNode::visualizeOctreeBoxes()
{
    std::vector<std::array<float, 6>> boxes = octree->toBoxes();
    visualization_msgs::msg::MarkerArray marker_array_msg;
    for (int i = 0; i < boxes.size(); i++) 
    {
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Box %d: (%f, %f, %f)", i, boxes[i][0], boxes[i][1], boxes[i][2]);
        visualization_msgs::msg::Marker marker;
        marker.type = visualization_msgs::msg::Marker::CUBE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "octree_boxes";
        marker.id = i;
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
        marker.color.r = 0.0;
        marker.color.g = 0.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker_array_msg.markers.emplace_back(marker);
    }
    marker_array_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing octree boxes...");
}

void TestPlannersNode::updateEnvironment()
{
    std::vector<std::shared_ptr<fcl::CollisionObjectf>> col_obj;

    // Table
    fcl::Vector3f tr(0, 0, -0.05);
    Eigen::Matrix3f rot = fcl::Quaternionf(0, 0, 0, 0).matrix();
    std::shared_ptr<fcl::CollisionGeometryf> table = std::make_shared<fcl::Cylinderf>(0.75, 0.1);
    col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(table, rot, tr));

    // Octree contains all other objects (without table and robot)
    // readOctree();
    // std::shared_ptr<fcl::CollisionGeometryf> octree_ = std::make_shared<fcl::OcTreef>(*octree);
    // col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(octree_));

    // Bounding-boxes
    for (int i = 0; i < bounding_boxes.size(); i += 2)
    {
        std::shared_ptr<fcl::CollisionGeometryf> box = std::make_shared<fcl::Boxf>(bounding_boxes[i]);
        col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(box, rot, bounding_boxes[i+1]));
    }

    // Convex-hulls
    // for (int i = 0; i < convex_hulls.size(); i++)
    // {
    //     std::shared_ptr<std::vector<fcl::Vector3f>> vertices = std::make_shared<std::vector<fcl::Vector3f>>(convex_hulls[i]);
    //     std::shared_ptr<std::vector<int>> faces = std::make_shared<std::vector<int>>(convex_hulls_polygons[i]);
    //     for (int j = 0; j < vertices->size(); j++)
    //         LOG(INFO) << "Vertex " << j << ": " << vertices->at(j).transpose();
    //     for (int j = 0; j < faces->size(); j+=3)
    //         LOG(INFO) << "Face " << j << ": " << faces->at(j) << ", " << faces->at(j+1) << ", " << faces->at(j+2);

    //     std::shared_ptr<fcl::CollisionGeometryf> hull = std::make_shared<fcl::Convexf>(vertices, faces->size() / 3, faces, true); // This object cannot be created for some reason!
    //     col_obj.emplace_back(std::make_shared<fcl::CollisionObjectf>(hull));
    // }

    scenario->setEnvironment(col_obj);
}

void TestPlannersNode::planPath()
{
    std::shared_ptr<base::StateSpace> ss = scenario->getStateSpace();
    std::shared_ptr<base::State> start = scenario->getStart();
    std::shared_ptr<base::State> goal = scenario->getGoal();
    
    LOG(INFO) << "Number of collision objects: " << scenario->getEnvironment()->getParts().size();
    LOG(INFO) << "Dimensions: " << ss->getDimensions();
    LOG(INFO) << "State space type: " << ss->getStateSpaceType();
    LOG(INFO) << "Start: " << start;
    LOG(INFO) << "Goal: " << goal;

    try
    {
        // std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RBTConnect>(ss, start, goal);
        std::unique_ptr<planning::AbstractPlanner> planner = std::make_unique<planning::rbt::RGBTConnect>(ss, start, goal);
        bool res = planner->solve();
        LOG(INFO) << "RGBTConnect planning finished with " << (res ? "SUCCESS!" : "FAILURE!");
        LOG(INFO) << "Number of states: " << planner->getPlannerInfo()->getNumStates();
        LOG(INFO) << "Planning time: " << planner->getPlannerInfo()->getPlanningTime() << " [ms]";

        path = planner->getPath();
        if (res)
        {
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

int main(int argc, char *argv[])
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
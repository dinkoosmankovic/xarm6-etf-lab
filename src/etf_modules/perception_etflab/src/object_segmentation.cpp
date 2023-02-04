#include "object_segmentation/object_segmentation.h"
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/search.h>
#include <pcl/search/kdtree.h>
#include <pcl/features/normal_3d.h>
#include <pcl/common/common.h>

#include <RealVectorSpace.h>


ObjectSegmentation::ObjectSegmentation() : Node("segmentation_node")
{
	this->declare_parameter<std::string>("input_cloud", "pointcloud_combined");
	this->declare_parameter<std::string>("objects_cloud", "objects_cloud");
		
	this->get_parameter("input_cloud", input_cloud);
	this->get_parameter("objects_cloud", objects_cloud);
	
	RCLCPP_INFO(this->get_logger(), "Starting up the segmentation node with input topic %s and output topic %s", input_cloud.c_str(), objects_cloud.c_str());
	
	pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(input_cloud, 
		rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
		std::bind(&ObjectSegmentation::point_cloud_callback, this, std::placeholders::_1));
				
	joint_states_subscription = this->create_subscription<control_msgs::msg::JointTrajectoryControllerState>
		("/xarm6_traj_controller/state", 10, std::bind(&ObjectSegmentation::joint_states_callback, this, std::placeholders::_1));

	object_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(objects_cloud, 1);
	marker_array_free_cells_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/free_cells_vis_array", 10);
	marker_array_occupied_cells_publisher = this->create_publisher<visualization_msgs::msg::MarkerArray>("/occupied_cells_vis_array", 10);

	std::string file_path(__FILE__);
	for (int i = 0; i < 3; i++)
		file_path = file_path.substr(0, file_path.find_last_of("/\\"));
	scenario_file_path = file_path + scenario_file_path;
	scenario = std::make_shared<scenario::Scenario>(scenario_file_path);
	robot = scenario->getRobot();
	skeleton = robot->computeSkeleton(scenario->getStart());
}

void ObjectSegmentation::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	pcl::PCLPointCloud2::Ptr input_pcl_cloud(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2());
	
	pcl::moveFromROSMsg(*msg, *cloud_xyzrgb);
	pcl::toPCLPointCloud2(*cloud_xyzrgb, *input_pcl_cloud);	
	
  	// Create the filtering object: downsample the dataset using a leaf size of 1cm
  	pcl::VoxelGrid<pcl::PCLPointCloud2> donwnsampler;
  	donwnsampler.setInputCloud(input_pcl_cloud);
  	donwnsampler.setLeafSize(0.01f, 0.01f, 0.01f);
  	donwnsampler.filter(*output_cloud);
  	// RCLCPP_INFO(this->get_logger(), "Downsampled.");
  	
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr 	output_cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>), 
										  	output_cloud_xyzrgb1(new pcl::PointCloud<pcl::PointXYZRGB>),
  										  	output_cloud_xyzrgb2(new pcl::PointCloud<pcl::PointXYZRGB>);
  										   
  	pcl::fromPCLPointCloud2(*output_cloud, *output_cloud_xyzrgb);
  	
  	pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
  	pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr table_green_cond(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, 60));
 	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond(new pcl::ConditionAnd<pcl::PointXYZRGB>());
 	color_cond->addComparison(table_green_cond);
	
 	// Build the filter
 	color_filter.setInputCloud(output_cloud_xyzrgb);
 	color_filter.setCondition(color_cond);
 	color_filter.filter(*output_cloud_xyzrgb);
  	
  	pcl::PassThrough<pcl::PointXYZRGB> passThroughZAxis;
  	passThroughZAxis.setFilterFieldName ("z");
    passThroughZAxis.setFilterLimits(-0.05, 1.5);
  	
  	passThroughZAxis.setInputCloud(output_cloud_xyzrgb);
  	passThroughZAxis.filter(*output_cloud_xyzrgb1);
  	// RCLCPP_INFO(this->get_logger(), "After downsampling cloud size is %d.", output_cloud_xyzrgb->size());
  	
  	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliers(new pcl::PointIndices ());
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  	// Optional
  	seg.setOptimizeCoefficients(true);
  	// Mandatory
  	seg.setModelType(pcl::SACMODEL_PLANE);
  	seg.setMethodType(pcl::SAC_RANSAC);
  	seg.setMaxIterations(1000);
  	seg.setDistanceThreshold(0.02);
  	seg.setInputCloud(output_cloud_xyzrgb1);  	
  	// RCLCPP_INFO(this->get_logger(), "Plane segmented.");
  	
  	// Extract the inliers
   	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   	extract.setInputCloud(output_cloud_xyzrgb1);
   	extract.setIndices(inliers);
   	extract.setNegative(true);
   	extract.filter(*output_cloud_xyzrgb2);
   	// RCLCPP_INFO(this->get_logger(), "Cloud size is %d.", output_cloud_xyzrgb2->size());
  	
	removePointsOccupiedByRobot(output_cloud_xyzrgb2);
    // visualizeRobotCapsules();
    // visualizeRobotSkeleton();
	// visualizeOutputPCL(output_cloud_xyzrgb2);

    // Set up a KD-Tree for searching
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr kdtree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    kdtree->setInputCloud(output_cloud_xyzrgb2);

    // Perform Euclidean clustering
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    ec.setClusterTolerance(0.02);
    ec.setMinClusterSize(10);
    ec.setMaxClusterSize(1000);
    ec.setSearchMethod(kdtree);
    ec.setInputCloud(output_cloud_xyzrgb2);
    ec.extract(cluster_indices);
    RCLCPP_INFO(this->get_logger(), "Point cloud is segmented into %d clusters.", cluster_indices.size());

    // Create separate point clouds for each cluster
    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> pcl_clusters;
    std::vector<pcl::PointXYZ> min_points, max_points;
    int j = 0;
    for (pcl::PointIndices cluster_index : cluster_indices)
    {
        RCLCPP_INFO(this->get_logger(), "Cluster %d.", j++);
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (int ind : cluster_index.indices)
        {
            pcl_cluster->points.emplace_back(output_cloud_xyzrgb2->points[ind]);
            // pcl::PointXYZRGB point = output_cloud_xyzrgb2->points[ind];
            // RCLCPP_INFO(this->get_logger(), "(%f, %f, %f)", point.x, point.y, point.z);
        }        
        pcl_cluster->width = pcl_cluster->points.size();
        pcl_cluster->height = 1;
        pcl_cluster->is_dense = true;
        pcl_clusters.emplace_back(pcl_cluster);

        // Compute the bounding box for the cluster
        Eigen::Vector4f min_point, max_point;
        pcl::getMinMax3D(*pcl_cluster, min_point, max_point);
        RCLCPP_INFO(this->get_logger(), "Bounding-box points: min = (%f, %f, %f), max = (%f, %f, %f)",
            min_point(0), min_point(1), min_point(2), max_point(0), max_point(1), max_point(2));
    }

  	publish_objects_point_cloud(output_cloud_xyzrgb2);
 
}

void ObjectSegmentation::joint_states_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
	std::vector<double> positions = msg->actual.positions;
	Eigen::VectorXf q(6);
	q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
	joint_states = scenario->getStateSpace()->newState(q);
	// RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void ObjectSegmentation::publish_objects_point_cloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
	sensor_msgs::msg::PointCloud2 output_cloud_ros;	
	pcl::toROSMsg(*pcl, output_cloud_ros);
	output_cloud_ros.header.stamp = now();
	object_point_cloud_publisher->publish(output_cloud_ros);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Publishing output point cloud of size %d...", pcl->size());
}

// Remove all PCL points occupied by the robot's capsules.
// 'tolerance_factor' (default: 2) determines the factor of capsule enlargement, which is always needed due to measurements uncertainty.
void ObjectSegmentation::removePointsOccupiedByRobot(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl, int tolerance_factor)
{
	// Compute skeleton only if the robot changed its configuration
	if ((joint_states->getCoord() - robot->getConfiguration()->getCoord()).norm() > 1e-3)
	{
		// RCLCPP_INFO(this->get_logger(), "Robot is moving. Computing new skeleton for (%f, %f, %f, %f, %f, %f).", 
		// 	joint_states->getCoord(0), joint_states->getCoord(1), joint_states->getCoord(2),
		// 	joint_states->getCoord(3), joint_states->getCoord(4), joint_states->getCoord(5));
		skeleton = robot->computeSkeleton(joint_states);
	}

	// Remove points occupied by the robot
	int pcl_init_size = pcl->size(), i = 0;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = pcl->end()-1; pcl_point >= pcl->begin(); pcl_point--)
	{
		Eigen::Vector3f point(pcl_point->x, pcl_point->y, pcl_point->z);
		// RCLCPP_INFO(this->get_logger(), "Point %d: (%f, %f, %f)", pcl_init_size - (++i), point(0), point(1), point(2));
		for (int k = robot->getParts().size()-1; k >= 0; k--)
		{
			float d_c = std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(skeleton->col(k), skeleton->col(k+1), point));
			if (d_c < robot->getRadius(k) * tolerance_factor)
			{
				// RCLCPP_INFO(this->get_logger(), "d_c from %d-th segment is %f", k, d_c);
				// RCLCPP_INFO(this->get_logger(), "Removing point: (%f, %f, %f)", point(0), point(1), point(2));
				pcl->erase(pcl_point);
				break;
			}
		}
	}
   	// RCLCPP_INFO(this->get_logger(), "Removed %d points occupied by the robot.", pcl_init_size - pcl->size());   	
}

void ObjectSegmentation::visualizeOutputPCL(pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl)
{
	visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;    
    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "output_point_cloud";
    marker.id = 0;
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;

    geometry_msgs::msg::Point point;
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = pcl->begin(); pcl_point < pcl->end(); pcl_point++)
	{
        point.x = pcl_point->x; 
		point.y = pcl_point->y; 
		point.z = pcl_point->z;
        marker.points.emplace_back(point);
	}	
    marker_array_msg.markers.emplace_back(marker);
    marker_array_occupied_cells_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing output point cloud...");
}

void ObjectSegmentation::visualizeRobotCapsules()
{
    std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(joint_states);
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.ns = "robot_capsules";
        marker.header.frame_id = "world";
        marker.header.stamp = now();
    Eigen::Vector3f A, B, C;

    for (int i = 0; i < skeleton->cols() - 1; i++) 
    {
        A = skeleton->col(i);
        B = skeleton->col(i+1);
        // LOG(INFO) << "Skeleton: " << A.transpose() << " --- " << B.transpose();

        marker.type = visualization_msgs::msg::Marker::CYLINDER;
        marker.id = i;
        marker.pose.position.x = (A(0) + B(0)) / 2;
        marker.pose.position.y = (A(1) + B(1)) / 2;
        marker.pose.position.z = (A(2) + B(2)) / 2;
        marker.pose.orientation.x = B(0) - A(0);
        marker.pose.orientation.y = B(1) - A(1);
        marker.pose.orientation.z = B(2) - A(2);
        marker.pose.orientation.w = 0.0;
        marker.scale.x = 2 * robot->getRadius(i);
        marker.scale.y = 2 * robot->getRadius(i);
        marker.scale.z = (B - A).norm();
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 0.5;
        marker_array_msg.markers.emplace_back(marker);

        for (int j = 0; j < 2; j++)
        {
            C = skeleton->col(i+j);
            marker.type = visualization_msgs::msg::Marker::SPHERE;
            marker.id = 6 * (j+1) + i;
            marker.pose.position.x = C(0);
            marker.pose.position.y = C(1);
            marker.pose.position.z = C(2);
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 0.0;
            marker.scale.x = 2 * robot->getRadius(i);
            marker.scale.y = 2 * robot->getRadius(i);
            marker.scale.z = 2 * robot->getRadius(i);
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            marker.color.a = 0.5;
            marker_array_msg.markers.emplace_back(marker);
        }
    }

    marker_array_free_cells_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing robot capsules...");
}

void ObjectSegmentation::visualizeRobotSkeleton()
{
    std::shared_ptr<Eigen::MatrixXf> skeleton = robot->computeSkeleton(joint_states);
    visualization_msgs::msg::MarkerArray marker_array_msg;
    visualization_msgs::msg::Marker marker;
    Eigen::Vector3f P;
    
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.ns = "robot_skeleton";
    marker.id = 0;
    marker.header.frame_id = "world";
    marker.header.stamp = now();
    geometry_msgs::msg::Point point;
    for (int i = 0; i < skeleton->cols(); i++) 
    {
        P = skeleton->col(i);
        point.x = P(0); 
		point.y = P(1); 
		point.z = P(2);
        marker.points.emplace_back(point);
    }
    marker.pose.position.x = 0.0;
    marker.pose.position.y = 0.0;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 0.01;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.r = 0.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.color.a = 1.0;
    marker_array_msg.markers.emplace_back(marker);

    marker.type = visualization_msgs::msg::Marker::POINTS;
    marker.id = 1;
    marker.scale.x = 0.02;
    marker.scale.y = 0.02;
    marker.scale.z = 0.02;
    marker_array_msg.markers.emplace_back(marker);

    marker_array_free_cells_publisher->publish(marker_array_msg);
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Visualizing robot skeleton...");
}

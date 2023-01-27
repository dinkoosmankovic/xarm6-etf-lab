#include "object_segmentation/object_segmentation.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h> //and the other usuals

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
   	RCLCPP_INFO(this->get_logger(), "Output cloud size is %d.", output_cloud_xyzrgb2->size());
  	
	// removePointsOccupiedByRobot(*output_cloud_xyzrgb2); 	// It does not work!

  	pcl::toPCLPointCloud2(*output_cloud_xyzrgb2, *output_cloud);  	
  	publish_objects_point_cloud(output_cloud);
 
}

void ObjectSegmentation::joint_states_callback(const control_msgs::msg::JointTrajectoryControllerState::SharedPtr msg)
{
	std::vector<double> positions = msg->actual.positions;
	Eigen::VectorXf q(6);
	q << positions[0], positions[1], positions[2], positions[3], positions[4], positions[5];
	joint_states = scenario->getStateSpace()->newState(q);
	// RCLCPP_INFO(this->get_logger(), "Robot joint states: (%f, %f, %f, %f, %f, %f).", q(0), q(1), q(2), q(3), q(4), q(5));
}

void ObjectSegmentation::publish_objects_point_cloud(pcl::PCLPointCloud2::Ptr output_cloud)
{
	sensor_msgs::msg::PointCloud2 output_cloud_ros;
	
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::fromPCLPointCloud2(*output_cloud, *pcl_cloud);
	    
	pcl::toROSMsg(*pcl_cloud, output_cloud_ros);

	output_cloud_ros.header.stamp = now();
	object_point_cloud_publisher->publish(output_cloud_ros);
}

void ObjectSegmentation::publish_objects()
{
}

void ObjectSegmentation::removePointsOccupiedByRobot(pcl::PointCloud<pcl::PointXYZRGB> &pcl)
{
	// Compute skeleton only if the robot changed its configuration
	if ((joint_states->getCoord() - robot->getConfiguration()->getCoord()).norm() > 1e-4)
	{
		RCLCPP_INFO(this->get_logger(), "Robot is moving. Computing new skeleton for (%f, %f, %f, %f, %f, %f).", 
			joint_states->getCoord(0), joint_states->getCoord(1), joint_states->getCoord(2),
			joint_states->getCoord(3), joint_states->getCoord(4), joint_states->getCoord(5));
		skeleton = robot->computeSkeleton(joint_states);
	}

	// Remove points occupied by the robot
	int tolerance_factor = 1.5, i = 0;
	int pcl_init_size = pcl.size();
	for (pcl::PointCloud<pcl::PointXYZRGB>::iterator pcl_point = pcl.begin(); pcl_point < pcl.end(); pcl_point++)
	{
		Eigen::Vector3f point(pcl_point->x, pcl_point->y, pcl_point->z);
		RCLCPP_INFO(this->get_logger(), "Point %d: (%f, %f, %f)", i++, point(0), point(1), point(2));
		for (int k = 0; k < robot->getParts().size(); k++)
		{
			float d_c = std::get<0>(base::RealVectorSpace::distanceLineSegToPoint(skeleton->col(k), skeleton->col(k+1), point));
			if (d_c 
				< robot->getRadius(k) * tolerance_factor)
			{
				// RCLCPP_INFO(this->get_logger(), "d_c from %d-th segment is %f", k, d_c);
				RCLCPP_INFO(this->get_logger(), "Removing point: (%f, %f, %f)", point(0), point(1), point(2));
				pcl.erase(pcl_point);
				break;
			}
		}
	}
   	RCLCPP_INFO(this->get_logger(), "Removed %d points occupied by the robot.", pcl_init_size - pcl.size());   	
}
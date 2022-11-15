#include "object_segmentation/object_segmentation.h"
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/conditional_removal.h> //and the other usuals


ObjectSegmentation::ObjectSegmentation() : Node("segmentation_node")
{
	this->declare_parameter<std::string>("input_cloud", "pointcloud_combined");
	this->declare_parameter<std::string>("objects_cloud", "objects_cloud");
		
	this->get_parameter("input_cloud", input_cloud);
	this->get_parameter("objects_cloud", objects_cloud);
	
	RCLCPP_INFO(this->get_logger(), "Starting up the segmentation node with input topic %s and output topic %s", input_cloud.c_str(), objects_cloud.c_str());
	
	pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
				input_cloud, rclcpp::QoS(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data)), 
				std::bind(&ObjectSegmentation::point_cloud_callback, this, std::placeholders::_1));
				
	object_point_cloud_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>(objects_cloud, 1);
	
}

void ObjectSegmentation::point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
	pcl::PCLPointCloud2::Ptr input_pcl_cloud(new pcl::PCLPointCloud2());
	pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xyzrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::PCLPointCloud2::Ptr output_cloud(new pcl::PCLPointCloud2());
	
	pcl::moveFromROSMsg(*msg, *cloud_xyzrgb);
	pcl::toPCLPointCloud2 (*cloud_xyzrgb, *input_pcl_cloud);	
	
  	// Create the filtering object: downsample the dataset using a leaf size of 1cm
  	pcl::VoxelGrid<pcl::PCLPointCloud2> donwnsampler;
  	donwnsampler.setInputCloud(input_pcl_cloud);
  	donwnsampler.setLeafSize (0.01f, 0.01f, 0.01f);
  	donwnsampler.filter(*output_cloud);
  	
  	RCLCPP_INFO(this->get_logger(), "Downsampled.");
  	
  	pcl::PointCloud<pcl::PointXYZRGB>::Ptr output_cloud_xyzrgb (new pcl::PointCloud<pcl::PointXYZRGB>), 
  										   output_cloud_xyzrgb1(new pcl::PointCloud<pcl::PointXYZRGB>),
  										   output_cloud_xyzrgb2(new pcl::PointCloud<pcl::PointXYZRGB>);
  										   
  	pcl::fromPCLPointCloud2 (*output_cloud, *output_cloud_xyzrgb);
  	
  	pcl::ConditionalRemoval<pcl::PointXYZRGB> color_filter;
  	pcl::PackedRGBComparison<pcl::PointXYZRGB>::Ptr table_green_cond(new pcl::PackedRGBComparison<pcl::PointXYZRGB>("g", pcl::ComparisonOps::LT, 60));
 	pcl::ConditionAnd<pcl::PointXYZRGB>::Ptr color_cond (new pcl::ConditionAnd<pcl::PointXYZRGB> ());
 	color_cond->addComparison(table_green_cond);
	
 	// Build the filter
 	color_filter.setInputCloud(output_cloud_xyzrgb);
 	color_filter.setCondition (color_cond);
 	color_filter.filter(*output_cloud_xyzrgb);
  	
  	pcl::PassThrough<pcl::PointXYZRGB> passThroughZAxis;
  	passThroughZAxis.setFilterFieldName ("z");
    passThroughZAxis.setFilterLimits (-0.05, 1.5);
  	
  	passThroughZAxis.setInputCloud (output_cloud_xyzrgb);
  	passThroughZAxis.filter (*output_cloud_xyzrgb1);
  	
  	
  	RCLCPP_INFO(this->get_logger(), "After downsampling cloud size is %d.", output_cloud_xyzrgb->size());
  	
  	
  	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
  	// Create the segmentation object
  	pcl::SACSegmentation<pcl::PointXYZRGB> seg;
  	// Optional
  	seg.setOptimizeCoefficients (true);
  	// Mandatory
  	seg.setModelType (pcl::SACMODEL_PLANE);
  	seg.setMethodType (pcl::SAC_RANSAC);
  	seg.setMaxIterations (1000);
  	seg.setDistanceThreshold (0.02);
  	seg.setInputCloud (output_cloud_xyzrgb1);
  	
  	RCLCPP_INFO(this->get_logger(), "Plane segmented.");
  	
  	// Extract the inliers
   	pcl::ExtractIndices<pcl::PointXYZRGB> extract;
   	extract.setInputCloud (output_cloud_xyzrgb1);
   	extract.setIndices (inliers);
   	extract.setNegative (true);
   	extract.filter (*output_cloud_xyzrgb2);
   	
   	RCLCPP_INFO(this->get_logger(), "Output cloud size is %d.", output_cloud_xyzrgb2->size());
  	
  	pcl::toPCLPointCloud2 (*output_cloud_xyzrgb2, *output_cloud);
  	
  	publish_objects_point_cloud(output_cloud);
 
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

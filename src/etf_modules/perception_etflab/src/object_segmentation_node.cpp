#include "rclcpp/rclcpp.hpp"
#include "object_segmentation/object_segmentation.h"

int main(int argc, char *argv[])
{
	rclcpp::init(argc, argv);
	rclcpp::spin(std::make_shared<ObjectSegmentation>());
	rclcpp::shutdown();
	return 0;
}

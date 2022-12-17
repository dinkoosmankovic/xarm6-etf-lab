#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "octomap_msgs/srv/get_octomap.hpp"
#include "octomap_msgs/msg/octomap.hpp"
#include "fcl/fcl.h"

using namespace std::chrono_literals;


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("test_read_octree_node");
    rclcpp::Client<octomap_msgs::srv::GetOctomap>::SharedPtr client = 
        node->create_client<octomap_msgs::srv::GetOctomap>("/octomap_binary");
    
    auto request = std::make_shared<octomap_msgs::srv::GetOctomap::Request>();
    
    while (!client->wait_for_service(1s)) 
    {
        if (!rclcpp::ok()) 
        {
            RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
            return 0;
        }
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service not available, waiting again...");
    }

    // Wait for the result
    auto result = client->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node, result) == rclcpp::FutureReturnCode::SUCCESS)
    {
        octomap_msgs::msg::Octomap octomap = result.get()->map;
        fcl::OcTree<float> octree(std::shared_ptr<const octomap::OcTree>(octomap));
        RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Read successfully. ");
    } 
    else
        RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Failed to call service!");

    rclcpp::shutdown();
    return 0;
}
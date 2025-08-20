#include "nav2_map_server/costmap_filter_info_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

int main(int argc, char* argv[])
{
    auto logger = rclcpp::get_logger("costmap_filter_info_server");

    RCLCPP_INFO(logger, "This is costmap filter info publisher");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_map_server::CostmapFilterInfoServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();

    return 0;
}

#include "nav2_map_server/map_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char** argv)
{
    std::string node_name("map_server");

    rclcpp::init(argc, argv);
    auto node = std::make_shared<nav2_map_server::MapServer>();
    rclcpp::spin(node->get_node_base_interface());
    rclcpp::shutdown();
}

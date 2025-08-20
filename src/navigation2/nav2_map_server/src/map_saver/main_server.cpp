#include "nav2_map_server/map_saver.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>
#include <stdexcept>
#include <string>

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto logger       = rclcpp::get_logger("map_saver_server");
    auto service_node = std::make_shared<nav2_map_server::MapSaver>();
    rclcpp::spin(service_node->get_node_base_interface());
    rclcpp::shutdown();
    return 0;
}

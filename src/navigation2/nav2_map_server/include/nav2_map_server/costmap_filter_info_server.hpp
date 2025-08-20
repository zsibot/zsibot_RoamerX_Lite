#ifndef NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_
#define NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

#include "nav2_util/lifecycle_node.hpp"
#include "nav2_msgs/msg/costmap_filter_info.hpp"
#include "rclcpp/rclcpp.hpp"

#include <memory>

namespace nav2_map_server
{

    class CostmapFilterInfoServer : public nav2_util::LifecycleNode
    {
    public:
        /**
         * @brief Constructor for the nav2_map_server::CostmapFilterInfoServer
         * @param options Additional options to control creation of the node.
         */
        explicit CostmapFilterInfoServer(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

        /**
         * @brief Destructor for the nav2_map_server::CostmapFilterInfoServer
         */
        ~CostmapFilterInfoServer();

    protected:
        /**
         * @brief Creates CostmapFilterInfo publisher and forms published message from ROS parameters
         * @param state Lifecycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_configure(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Publishes a CostmapFilterInfo message
         * @param state Lifecycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_activate(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Deactivates publisher
         * @param state Lifecycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Resets publisher
         * @param state Lifecycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& state) override;
        /**
         * @brief Called when in Shutdown state
         * @param state Lifecycle Node's state
         * @return Success or Failure
         */
        nav2_util::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& state) override;

    private:
        rclcpp_lifecycle::LifecyclePublisher<nav2_msgs::msg::CostmapFilterInfo>::SharedPtr publisher_;

        nav2_msgs::msg::CostmapFilterInfo msg_;
    };  // CostmapFilterInfoServer

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__COSTMAP_FILTER_INFO_SERVER_HPP_

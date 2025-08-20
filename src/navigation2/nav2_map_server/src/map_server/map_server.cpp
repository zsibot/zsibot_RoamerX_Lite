#include "nav2_map_server/map_server.hpp"

#include "lifecycle_msgs/msg/state.hpp"
#include "nav2_map_server/map_io.hpp"
#include "nav2_util/execution_timer.hpp"
#include "yaml-cpp/yaml.h"

#include <fstream>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace nav2_map_server
{

    MapServer::MapServer(const rclcpp::NodeOptions& options)
        : nav2_util::LifecycleNode("map_server", "", options),
          map_available_(false)
    {
        RCLCPP_INFO(get_logger(), "Creating");

        // Declare the node parameters
        declare_parameter("yaml_filename", rclcpp::PARAMETER_STRING);
        declare_parameter("topic_name", "map");
        declare_parameter("frame_id", "map");
    }

    MapServer::~MapServer() {}

    nav2_util::CallbackReturn MapServer::on_configure(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Configuring");

        // Get the name of the YAML file to use (can be empty if no initial map should be used)
        std::string yaml_filename = get_parameter("yaml_filename").as_string();
        std::string topic_name    = get_parameter("topic_name").as_string();
        frame_id_                 = get_parameter("frame_id").as_string();

        // only try to load map if parameter was set
        if (!yaml_filename.empty())
        {
            // Shared pointer to LoadMap::Response is also should be initialized
            // in order to avoid null-pointer dereference
            std::shared_ptr<nav2_msgs::srv::LoadMap::Response> rsp = std::make_shared<nav2_msgs::srv::LoadMap::Response>();

            if (!loadMapResponseFromYaml(yaml_filename, rsp))
            {
                throw std::runtime_error("Failed to load map yaml file: " + yaml_filename);
            }
        }
        else
        {
            RCLCPP_INFO(get_logger(), "yaml-filename parameter is empty, set map through '%s'-service", load_map_service_name_.c_str());
        }

        // Use callback group to handle services callback
        map_callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // Make name prefix for services
        const std::string service_prefix = get_name() + std::string("/");

        // Create a service that provides the occupancy grid
        occ_service_ = create_service<nav_msgs::srv::GetMap>(service_prefix + std::string(service_name_),
            std::bind(&MapServer::getMapCallback, this, _1, _2, _3), rmw_qos_profile_services_default, map_callback_group_);

        // Create a publisher using the QoS settings to emulate a ROS1 latched topic
        rclcpp::QoS qos_profile(10);  // Initial depth, but KEEP_ALL will override
        qos_profile.reliability(RMW_QOS_POLICY_RELIABILITY_RELIABLE);
        qos_profile.history(RMW_QOS_POLICY_HISTORY_KEEP_ALL);
        occ_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>(topic_name, qos_profile);

        // Create a service that loads the occupancy grid from a file
        load_map_service_ = create_service<nav2_msgs::srv::LoadMap>(service_prefix + std::string(load_map_service_name_),
            std::bind(&MapServer::loadMapCallback, this, _1, _2, _3), rmw_qos_profile_services_default, map_callback_group_);

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MapServer::on_activate(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Activating");

        // Publish the map using the latched topic
        occ_pub_->on_activate();
        if (map_available_)
        {
            auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
            occ_pub_->publish(std::move(occ_grid));
        }

        // create bond connection
        createBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MapServer::on_deactivate(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Deactivating");

        occ_pub_->on_deactivate();

        // destroy bond connection
        destroyBond();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MapServer::on_cleanup(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Cleaning up");

        occ_pub_.reset();
        occ_service_.reset();
        load_map_service_.reset();
        map_available_ = false;
        msg_           = nav_msgs::msg::OccupancyGrid();

        return nav2_util::CallbackReturn::SUCCESS;
    }

    nav2_util::CallbackReturn MapServer::on_shutdown(const rclcpp_lifecycle::State& /*state*/)
    {
        RCLCPP_INFO(get_logger(), "Shutting down");
        return nav2_util::CallbackReturn::SUCCESS;
    }

    void MapServer::getMapCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<nav_msgs::srv::GetMap::Request> /*request*/, std::shared_ptr<nav_msgs::srv::GetMap::Response> response)
    {
        // if not in ACTIVE state, ignore request
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_WARN(get_logger(), "Received GetMap request but not in ACTIVE state, ignoring!");
            return;
        }
        RCLCPP_INFO(get_logger(), "Handling GetMap request");
        response->map = msg_;
    }

    void MapServer::loadMapCallback(const std::shared_ptr<rmw_request_id_t> /*request_header*/,
        const std::shared_ptr<nav2_msgs::srv::LoadMap::Request> request, std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
    {
        nav2_util::ExecutionTimer TIMER;
        TIMER.start();

        // if not in ACTIVE state, ignore request
        if (get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE)
        {
            RCLCPP_WARN(get_logger(), "Received LoadMap request but not in ACTIVE state, ignoring!");
            response->result = response->RESULT_UNDEFINED_FAILURE;
            return;
        }
        RCLCPP_INFO(get_logger(), "Handling LoadMap request");
        // Load from file
        if (loadMapResponseFromYaml(request->map_url, response))
        {
            auto occ_grid = std::make_unique<nav_msgs::msg::OccupancyGrid>(msg_);
            occ_pub_->publish(std::move(occ_grid));  // publish new map
        }

        TIMER.end();
        RCLCPP_INFO(get_logger(), "Handling LoadMap request time: %.9f", TIMER.elapsed_time_in_seconds());
    }

    bool MapServer::loadMapResponseFromYaml(const std::string& yaml_file, std::shared_ptr<nav2_msgs::srv::LoadMap::Response> response)
    {
        if (yaml_file == yaml_file_ && map_available_)
        {
            RCLCPP_INFO(get_logger(), "The map %s has already been loaded", yaml_file_.c_str());
            // Correcting msg_ header when it belongs to specific node
            updateMsgHeader();

            response->map    = msg_;
            response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
            return true;
        }

        switch (loadMapFromYaml(yaml_file, msg_))
        {
            case MAP_DOES_NOT_EXIST:
                response->result = nav2_msgs::srv::LoadMap::Response::RESULT_MAP_DOES_NOT_EXIST;
                return false;
            case INVALID_MAP_METADATA:
                response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_METADATA;
                return false;
            case INVALID_MAP_DATA:
                response->result = nav2_msgs::srv::LoadMap::Response::RESULT_INVALID_MAP_DATA;
                return false;
            case LOAD_MAP_SUCCESS:
                // Correcting msg_ header when it belongs to specific node
                updateMsgHeader();

                map_available_   = true;
                response->map    = msg_;
                response->result = nav2_msgs::srv::LoadMap::Response::RESULT_SUCCESS;
        }

        yaml_file_ = yaml_file;

        return true;
    }

    void MapServer::updateMsgHeader()
    {
        msg_.info.map_load_time = now();
        msg_.header.frame_id    = frame_id_;
        msg_.header.stamp       = now();
    }

}  // namespace nav2_map_server

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(nav2_map_server::MapServer)

#ifndef NAV2_MAP_SERVER__MAP_IO_HPP_
#define NAV2_MAP_SERVER__MAP_IO_HPP_

#include "nav2_map_server/map_mode.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

#include <string>
#include <vector>

/* Map input part */

namespace nav2_map_server
{

    struct LoadParameters
    {
        std::string         image_file_name;
        double              resolution{ 0 };
        std::vector<double> origin{ 0, 0, 0 };
        double              free_thresh;
        double              occupied_thresh;
        MapMode             mode;
        bool                negate;
    };

    typedef enum
    {
        LOAD_MAP_SUCCESS,
        MAP_DOES_NOT_EXIST,
        INVALID_MAP_METADATA,
        INVALID_MAP_DATA
    } LOAD_MAP_STATUS;

    /**
     * @brief Load and parse the given YAML file
     * @param yaml_filename Name of the map file passed though parameter
     * @return Map loading parameters obtained from YAML file
     * @throw YAML::Exception
     */
    LoadParameters loadMapYaml(const std::string& yaml_filename);

    /**
     * @brief Load the image from map file and generate an OccupancyGrid
     * @param load_parameters Parameters of loading map
     * @param map Output loaded map
     * @throw std::exception
     */
    void loadMapFromFile(const LoadParameters& load_parameters, nav_msgs::msg::OccupancyGrid& map);

    /**
     * @brief Load the map YAML, image from map file and
     * generate an OccupancyGrid
     * @param yaml_file Name of input YAML file
     * @param map Output loaded map
     * @return status of map loaded
     */
    LOAD_MAP_STATUS loadMapFromYaml(const std::string& yaml_file, nav_msgs::msg::OccupancyGrid& map);


    /* Map output part */

    struct SaveParameters
    {
        std::string map_file_name{ "" };
        std::string image_format{ "" };
        double      free_thresh{ 0.0 };
        double      occupied_thresh{ 0.0 };
        MapMode     mode{ MapMode::Trinary };
    };

    /**
     * @brief Write OccupancyGrid map to file
     * @param map OccupancyGrid map data
     * @param save_parameters Map saving parameters.
     * @return true or false
     */
    bool saveMapToFile(const nav_msgs::msg::OccupancyGrid& map, const SaveParameters& save_parameters);

    /**
     * @brief Expand ~/ to home user dir.
     * @param yaml_filename Name of input YAML file.
     * @param home_dir Expanded `~/`home dir or empty string if HOME not set
     *
     * @return Expanded string or input string if `~/` not expanded
     */
    std::string expand_user_home_dir_if_needed(std::string yaml_filename, std::string home_dir);

}  // namespace nav2_map_server

#endif  // NAV2_MAP_SERVER__MAP_IO_HPP_

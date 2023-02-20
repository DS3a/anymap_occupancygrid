#ifndef ANYMAP_OBSERVATION_BUFFER_H_
#define ANYMAP_OBSERVATION_BUFFER_H_

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_pcl/grid_map_pcl.hpp"


#include "nav_msgs/msg/occupancy_grid.hpp"

namespace anymap {
    grid_map::GridMap init_anymap() {
        // initialize with empty layers
        grid_map::GridMap anymap = grid_map::GridMap();

        anymap.setGeometry(grid_map::Length(6, 6), 0.01875);

        return anymap;
    }

    bool as_occupancy_grid(std::shared_ptr<grid_map::GridMap> anymap, std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg) {

        if (anymap->exists("aggregate")) {
            grid_map::GridMapRosConverter conv;
            conv.toOccupancyGrid(*anymap.get(), "aggregate", 0, 1, *grid_msg.get());
        }
        return true;
    }


// tests
    void add_test_layer(std::shared_ptr<grid_map::GridMap> anymap) {
        anymap->add("test");
    }

    void test_as_occupancy_grid(std::shared_ptr<grid_map::GridMap> anymap, std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg) {
        grid_map::GridMapRosConverter conv;

        conv.toOccupancyGrid(*anymap.get(), "test", 0, 1, *grid_msg.get());
    }
}

#endif // ANYMAP_OBSERVATION_BUFFER_H_

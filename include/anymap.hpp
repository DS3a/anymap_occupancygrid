#ifndef ANYMAP_H_
#define ANYMAP_H_

#include <iostream>

#include "grid_map_core/grid_map_core.hpp"
#include "grid_map_ros/grid_map_ros.hpp"
#include "grid_map_pcl/grid_map_pcl.hpp"

#include "observation_source.hpp"

#include "nav_msgs/msg/occupancy_grid.hpp"

#include "pcl/point_cloud.h"


namespace anymap {
	// Layers can be added to this with ease with the help of observation_sources.hpp, and all can be concatenated too

    // This function can be used to get a preliminary anymap instance
    grid_map::GridMap init_anymap() {
        // initialize with empty layers
        grid_map::GridMap anymap = grid_map::GridMap();

        anymap.setGeometry(grid_map::Length(8, 8), 0.025);
        // it's a 320x320 grid

        return anymap;
    }

    static std::vector<std::shared_ptr<observation_source::ObservationSource>> observation_sources;

    bool update_anymap() {
        // TODO update all layers and add them into the aggregate layer
    }

    // This function converts the anymap instance to an occupancy grid
    bool as_occupancy_grid(std::shared_ptr<grid_map::GridMap> anymap, std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg) {

        if (anymap->exists("aggregate")) {
            grid_map::GridMapRosConverter conv;
            conv.toOccupancyGrid(*anymap.get(), "aggregate", 0, 1, *grid_msg.get());
            return true;
        }
	return false;
    }


// tests
    void add_test_layer(std::shared_ptr<grid_map::GridMap> anymap) {
        anymap->add("test", 0);
    }

    void test_as_occupancy_grid(std::shared_ptr<grid_map::GridMap> anymap, std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg) {
        grid_map::GridMapRosConverter conv;

        conv.toOccupancyGrid(*anymap.get(), "test", 0, 1, *grid_msg.get());
    }

    void add_pcl_to_occupancy_grid();

    std::shared_ptr<int> test_int(new int(3));

    void update_test_int(std::shared_ptr<int> test_int_) {
        test_int = test_int_;
    }

    void print_test_int() {
        std::cout << *test_int.get() << std::endl;
    }

}

#endif // ANYMAP_OBSERVATION_BUFFER_H_

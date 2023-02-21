#ifndef OBSERVATION_SOURCE_H
#define OBSERVATION_SOURCE_H

#include "rclcpp/rclcpp.hpp"
#include "grid_map_pcl/grid_map_pcl.hpp"
#include "pcl/point_cloud.h"

namespace observation_source {
     class ObservationSource {
      public:
        ObservationSource(const rclcpp::Logger & node_logger) {
            this->cloud = std::make_shared<pcl::PointCloud<POINT_TYPE>>();
            pcl_loader_ptr = std::shared_ptr<grid_map::GridMapPclLoader>(new grid_map::GridMapPclLoader(node_logger));
        }

        void set_input_cloud(std::shared_ptr<pcl::PointCloud<POINT_TYPE>> input_cloud) {
            this->cloud = input_cloud;
            this->pcl_loader_ptr->setInputCloud(this->cloud);
        }

        void add_parameters(grid_map::grid_map_pcl::PclLoaderParameters::Parameters parameters) {
            this->pcl_loader_ptr->setParameters(parameters);
        }

        void add_default_parameters() {
            grid_map::grid_map_pcl::PclLoaderParameters::Parameters params;
            params.numThreads_ = 8;
            params.gridMap_.resolution_ = 0.01875;
            this->add_parameters(params);
            std::cout << this->pcl_loader_ptr->params_->parameters_.gridMap_.resolution_ << std::endl;
        }
      private:
        pcl::PointCloud<POINT_TYPE>::Ptr cloud;
        std::shared_ptr<grid_map::GridMapPclLoader> pcl_loader_ptr;
    };


}

#endif

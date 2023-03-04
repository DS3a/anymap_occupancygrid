#ifndef OBSERVATION_SOURCE_H
#define OBSERVATION_SOURCE_H
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "grid_map_pcl/grid_map_pcl.hpp"
#include "pcl/point_cloud.h"


// This needs work, the converts the pointcloud data into a grid which can be appended to the anymap instance
namespace observation_source {
    class ObservationSource {
    public:
        ObservationSource(std::string layer_, std::shared_ptr<grid_map::GridMap> _anymap_ptr, double resolution_=0.01875) {
            this->cloud = std::make_shared<pcl::PointCloud<POINT_TYPE>>();
            this->layer = layer_;
            this->anymap_ptr_ = _anymap_ptr;
            this->anymap_ptr_->add(this->layer, 0.0);
            this->resolution = resolution_;
        }

        void set_input_cloud(std::shared_ptr<pcl::PointCloud<POINT_TYPE>> input_cloud) {
            this->cloud = input_cloud;
        }

        void update_layer() {
            for (auto &point: *(this->cloud)) {
                int x_position = point.x / this->resolution;
                int y_position = point.y / this->resolution;
                std::cout << "(" << x_position << ", " << y_position << "), ";
                grid_map::Position pose(point.x, point.y);
                try {
                    this->anymap_ptr_->atPosition(this->layer, pose) += 1.0;
                } catch (std::out_of_range e) {
                    std::cout << "out of range, work on filtering please\n";
                    continue;
                }
            }
            std::cout << std::endl;
        }

        void clear_layer() {
            this->anymap_ptr_->get(this->layer).setConstant(0.0);
        }

    private:
        double resolution;
        pcl::PointCloud<POINT_TYPE>::Ptr cloud;
        std::string layer;
        std::shared_ptr<grid_map::GridMap> anymap_ptr_;
    };


}

#endif

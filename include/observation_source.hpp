#ifndef OBSERVATION_SOURCE_H
#define OBSERVATION_SOURCE_H
#include <opencv2/opencv.hpp>

#include "rclcpp/rclcpp.hpp"
#include "grid_map_pcl/grid_map_pcl.hpp"
#include "pcl/point_cloud.h"

#define POINT_TYPE pcl::PointXYZ

// This needs work, the converts the pointcloud data into a grid which can be appended to the anymap instance
namespace observation_source {
    class ObservationSource {
    public:
        ObservationSource(std::string layer_,
                          std::shared_ptr<grid_map::GridMap> _anymap_ptr,
                          double resolution_=0.01875) {
            this->layer = layer_;
            this->anymap_ptr_ = _anymap_ptr;
            this->anymap_ptr_->add(this->layer, 0.0);
            this->resolution = resolution_;
            this->point_weight = 0.8;
        }

        void set_input_cloud(boost::shared_ptr<pcl::PointCloud<POINT_TYPE>> input_cloud) {
            this->cloud = input_cloud;
        }

        void set_update_flag() {
            this->update_available = true;
        }

        void set_point_weight(float point_weight_) {
            this->point_weight = point_weight_;
        }

        void update_layer() {
            if (this->update_available) {
                for (auto &point: *(this->cloud)) {
                    int x_position = point.x / this->resolution;
                    int y_position = point.y / this->resolution;
                    // std::cout << "(" << x_position << ", " << y_position << "), ";
                    grid_map::Position pose(point.x, point.y);
                    try {
                        this->anymap_ptr_->atPosition(this->layer, pose) += this->point_weight;
                        // TODO optimization, avoid this try catch thingi
                    } catch (std::out_of_range e) {
                        // std::cout << "point out of range, useless fellow. work on filtering\n";
                        continue;
                    }
                    this->update_available = false;
                }
            } else {
                std::cout << "the pointcloud has not been updated";
            }
        }

        void clear_layer() {
            this->anymap_ptr_->get(this->layer).setConstant(0.0);
        }

    private:
        bool update_available;
        double resolution;
        float point_weight;
        pcl::PointCloud<POINT_TYPE>::Ptr cloud;
        std::string layer;
        std::shared_ptr<grid_map::GridMap> anymap_ptr_;
    };


}

#endif

#ifndef PRATHAM_OBSERVATION_SOURCES_H_
#define PRATHAM_OBSERVATION_SOURCES_H_

#include "pcl_preprocessor.hpp"
#include "observation_source.hpp"
// #include "layer_postprocessor.hpp"
#include "anymap.hpp"

#include "pcl/point_cloud.h"

class PrathamObservationSource() {
public:

    PrathamObservationSource(std::string layername);

    // DONE
    // To be called by anymap node
    // set the input cloud for the observation source
    void set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud);

    // DONE
    // set the preprocessor of the observation source
    void set_preprocessor(pcl_preprocessor::PclPreProcessor preprocessor_);


private:
    std::string layer_name;
    pcl::PointCloud<POINT_TYPE>::Ptr source_cloud;
    pcl_preprocessor::PclPreProcessor preprocessor;
};

PrathamObservationSource::PrathamObservationSource(std::string layername) {
    this->layer_name = layername
}

void PrathamObservationSource::set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud) {
    this->source_cloud = input_cloud;
}

void PrathamObservationSource::set_preprocessor(pcl_preprocessor::PclPreProcessor preprocessor_) {
    this->preprocessor = preprocessor_;
    this->preprocessor.set_input_cloud(this->source_cloud);
}

namespace pratham_observation_sources {
    /* TODO Add pcl sources
     * add pothole PCL
     * add left lane PCL
     * add right lane PCL
     * add lidar obstacle layer


     * TODO Preprocessing
     * add transformations for each
     * add boxfilter for each source


     * TODO Post Processing
     * add separate inflation for each layer
     */

    Eigen::Affine3f realsense_transformation = Eigen::Affine3f::Identity;
    bool rs_initialized = false;
    void initialize_rs_tf() {

        // TODO check if rotation or translation needs to be done first
        realsense_transformation.
            rotate(Eigen::AngleAxisf(
                       -3.141592653/2.0,
                       Eigen::Vector3f::UnitX()));
        realsense_transformation
            .rotate(Eigen::AngleAxisf(
                        3.141592653/2.0,
                        Eigen::Vector3f::UnitY()));
        rs_initialized = true;
    }
    
    PrathamObservationSource potholes_src;
    void init_potholes_src() {
        initialize_rs_tf();

        pcl_preprocessor::PclPreProcessor potholes_preprocessor;
        potholes_preprocessor.set_input_cloud(source_cloud);
        potholes_preprocessor.set_transform(realsense_transformation);
        potholes_preprocessor.set_boxfilter(0, -4, 0, 8, 4, 1.2);
        potholes_src.set_preprocessor(potholes_preprocessor);
    }

}

#endif // PRATHAM_OBSERVATION_SOURCES_H_

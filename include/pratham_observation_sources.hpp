#ifndef PRATHAM_OBSERVATION_SOURCES_H_
#define PRATHAM_OBSERVATION_SOURCES_H_

#include "pcl_preprocessor.hpp"
#include "observation_source.hpp"
#include "layer_postprocessor.hpp"
#include "anymap.hpp"

#include "pcl/point_cloud.h"

class PrathamObservationSource {
public:

    PrathamObservationSource(std::string layername);

    // DONE
    // To be called by anymap node
    // set the input cloud for the observation source
    void set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud);

    // DONE
    // set the preprocessor of the observation source
    void set_preprocessor(pcl_preprocessor::PclPreProcessor preprocessor_);

    // set the anymap pointer which has to be used by the postprocessor
    void set_anymap_ptr(std::shared_ptr<grid_map::GridMap> anymap_ptr);
    std::string layer_name;
    pcl::PointCloud<POINT_TYPE>::Ptr source_cloud;
    pcl_preprocessor::PclPreProcessor preprocessor;
    std::shared_ptr<grid_map::GridMap> anymap_ptr_;
    std::shared_ptr<observation_source::ObservationSource> source_ptr;
    layer_postprocessor::LayerPostProcessor postprocessor;
};

void PrathamObservationSource::set_anymap_ptr(std::shared_ptr<grid_map::GridMap> anymap_ptr) {
    this->anymap_ptr_ = anymap_ptr;
    this->postprocessor.set_input_grid(this->anymap_ptr_);
    this->source_ptr = std::shared_ptr<observation_source::ObservationSource>(
        new observation_source::ObservationSource(this->layer_name, this->anymap_ptr_));
}

PrathamObservationSource::PrathamObservationSource(std::string layername) {
    this->layer_name = layername;
    this->postprocessor.set_layer_name(this->layer_name);
    this->postprocessor.set_input_grid(this->anymap_ptr_);
}

void PrathamObservationSource::set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud) {
    this->source_cloud = input_cloud;
    this->preprocessor.set_input_cloud(this->source_cloud);
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
     *
     * TODO potholes_src
     * init_potholes_src()
     ** set the preprocessor DONE
     *** preprocessor transformation DONE
     *** preprocessor boxfilter DONE
     *** preprocessor leaf size DONE
     ** initialize
     ** set post processor params
     *** layername DONE
     *** gridmap_ptr DONE
     *
     * update_source_layer()
     */

    Eigen::Affine3f realsense_transformation = Eigen::Affine3f::Identity();
    bool rs_initialized = false;
    void initialize_rs_tf() {

        // TODO check if rotation or translation needs to be done first
        realsense_transformation
            .rotate(Eigen::AngleAxisf(
                       -3.141592653/2.0,
                       Eigen::Vector3f::UnitX()));
        realsense_transformation
            .rotate(Eigen::AngleAxisf(
                        3.141592653/2.0,
                        Eigen::Vector3f::UnitY()));
        rs_initialized = true;
    }
    
    static PrathamObservationSource potholes_src("potholes_layer");
    void init_potholes_src(pcl::PointCloud<POINT_TYPE>::Ptr source_cloud_,
                           std::shared_ptr<grid_map::GridMap> anymap_ptr_) {
        initialize_rs_tf();

        pcl_preprocessor::PclPreProcessor potholes_preprocessor;
        potholes_preprocessor.set_leaf_size(0.13);
        potholes_preprocessor.set_transform(realsense_transformation);
        potholes_preprocessor.set_boxfilter(0, -4, 0, 8, 4, 1.2);
        potholes_preprocessor.set_input_cloud(source_cloud_);

        potholes_src.set_preprocessor(potholes_preprocessor);
        potholes_src.set_input_cloud(source_cloud_);
        potholes_src.set_anymap_ptr(anymap_ptr_);

        potholes_src.source_ptr->set_point_weight(0.6);
        potholes_src.source_ptr->set_input_cloud(source_cloud_);

    }


    void update_potholes_layer() {

        std::cout << "beginning preprocessing \n";
        // potholes_src.preprocessor.set_input_cloud(potholes_src.source_cloud);
        potholes_src.preprocessor.preprocess_cloud();

        std::cout << "updating the layer\n";
        std::cout << "  clearing the layer\n";
        potholes_src.source_ptr->clear_layer();
        std::cout << "  setting the input cloud of the layer\n";
        potholes_src.source_ptr->set_input_cloud(potholes_src.source_cloud);
        std::cout << "  setting the update_flag of the layer\n";
        potholes_src.source_ptr->set_update_flag();
        std::cout << "  updating the layer\n";
        potholes_src.source_ptr->update_layer();

        std::cout << "postprocessing the layer\n";
        potholes_src.postprocessor.process_layer();
    }


    // TODO aggregate all layers and add them to the "aggregate" layer in pcl
    void update_pratham_anymap() {

    }
}

#endif // PRATHAM_OBSERVATION_SOURCES_H_

#ifndef LAYER_POSTPROCESSOR_H_
#define LAYER_POSTPROCESSOR_H_

#include "grid_map_cv/GridMapCvConverter.hpp"
#include <opencv2/opencv.hpp>

namespace layer_postprocessor {
    class LayerPostProcessor {
    public:
        LayerPostProcessor();
        void process_layer();

        void set_layer_name(std::string layer_name_);
        void set_input_grid(std::shared_ptr<grid_map::GridMap> gridmap_ptr_);
    private:
        grid_map::GridMapCvConverter cv_converter;
        float inflation;
        cv::Mat image;
        std::string layer_name;
        std::shared_ptr<grid_map::GridMap> anymap_ptr_;

        void get_image();

        void back_to_grid();
    };

    LayerPostProcessor::LayerPostProcessor() {
    }

    void LayerPostProcessor::set_layer_name(std::string layer_name_) {
        this->layer_name = layer_name_;
    }

    void LayerPostProcessor::set_input_grid(std::shared_ptr<grid_map::GridMap> gridmap_ptr_) {
        this->anymap_ptr_ = gridmap_ptr_;
    }

    void LayerPostProcessor::get_image() {
        // encoding is CV_32F => 32 bit float
        grid_map::GridMap *temp_grid = this->anymap_ptr_.get();

        const int encoding = 5;
        this->cv_converter.toImage<float, 1>((grid_map::GridMap&)*temp_grid, (const std::string&)this->layer_name,
                                   encoding, 0, 1, this->image);
    }

    void LayerPostProcessor::back_to_grid() {
        grid_map::GridMap *temp_grid = this->anymap_ptr_.get();
        this->cv_converter.addLayerFromImage<float, 1>(this->image,
                                             this->layer_name + "Processed",
                                             (grid_map::GridMap&)*temp_grid);
    }

    void LayerPostProcessor::process_layer() {
        this->get_image();

        // TODO process shit
        cv::Mat image_eroded_with_3x3_kernel;
/*
        cv::erode(this->image, image_eroded_with_3x3_kernel,
                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        this->image = image_eroded_with_3x3_kernel;
        cv::dilate(this->image, image_eroded_with_3x3_kernel,
                  cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        this->image = image_eroded_with_3x3_kernel;*/
        cv::dilate(this->image, image_eroded_with_3x3_kernel,
                  cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(15, 15)));
        this->image = image_eroded_with_3x3_kernel;


        this->back_to_grid();
    }
}
#endif // LAYER_POSTPROCESSOR_H_

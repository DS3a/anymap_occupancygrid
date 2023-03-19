#ifndef PCL_PREPROCESSOR_H_
#define PCL_PREPROCESSOR_H_

#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/filters/voxel_grid.h"

namespace pcl_preprocessor {
    class PclPreProcessor {
    public:
        // DONE
        PclPreProcessor();

        // DONE
        // preprocesses the input_cloud
        bool preprocess_cloud();


        // TODO
        void set_leaf_size(double leaf_size_);

        // DONE
        // set the transformation that the pointcloud has to go through
        // NOTE the transformation happens before the boxfilter if `transform_first` is set to true
        // the default value for `transform_first` is true
        void set_transform(Eigen::Affine3f transformation_);

        // DONE
        // set the parameters of the box. All parameters inside this box will be taken into consideration to add to the gridmap
        void set_boxfilter(double min_x_, double min_y_, double min_z_,
                           double max_x_, double max_y_, double max_z_);

        // DONE
        // set the input cloud pointer for preprocessing
        // a pointer is being accepted, and all modifications are made to it
        void set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud);

        // DONE
        // use this function to set `transform_first`
        void set_transform_first(bool transform_first_);

    private:
        pcl::PointCloud<POINT_TYPE>::Ptr cloud;

        bool cloud_given;

        bool transform_first;
        bool transform_cloud;
        Eigen::Affine3f transformation;

        // DONE
        // transform the cloud
        bool apply_transformation();

        bool downsample;
        double leaf_size;
        pcl::VoxelGrid<POINT_TYPE> downsampling_filter;
        bool apply_downsampling_cloud();


        // DONE
        // filter the cloud with boxes
        bool apply_box3d_filter();
        bool box3d_filter;
        double min_x;
        double min_y;
        double min_z;

        double max_x;
        double max_y;
        double max_z;
        pcl::ConditionalRemoval<POINT_TYPE> box3d_filter_handler;
        pcl::ConditionAnd<POINT_TYPE>::Ptr box3d_cond;

    };

    PclPreProcessor::PclPreProcessor() {
        this->downsample = false;
        this->transform_cloud = false;
        this->box3d_filter = false;
        this->transform_first = true;
        this->cloud_given = false;
    }

    bool PclPreProcessor::apply_transformation() {
        if (this->transform_cloud) {
            pcl::PointCloud<POINT_TYPE>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
            pcl::transformPointCloud(*(this->cloud), *transformed_cloud, this->transformation);
            this->cloud = transformed_cloud;
            return true;
        } else {
            std::cout << "transform_cloud is set to false, please ensure you have set the transfomration using set_transform()\n";
            return false;
        }
    }


    bool PclPreProcessor::apply_box3d_filter() {
        if (this->box3d_filter) {
            this->box3d_filter_handler.filter(*(this->cloud));
            return true;
        } else {
            std::cout << "box3d_filter is set ot false, please ensure that you have set the transformation using set_boxfilter()\n";
            return false;
        }
    }

    bool PclPreProcessor::apply_downsampling_cloud() {
        if (this->downsample) {
            pcl::PointCloud<POINT_TYPE>::Ptr downsampled_cloud (new pcl::PointCloud<POINT_TYPE>());
            this->downsampling_filter.setInputCloud(this->cloud);
            this->downsampling_filter.filter(*downsampled_cloud);
            this->cloud = downsampled_cloud;

            return true;
        }

        return false;
    }

    bool PclPreProcessor::preprocess_cloud() {
        if (this->cloud_given) {
            // applies downsampling if downsampling is set to true
            this->apply_downsampling_cloud();
            if (this->transform_first) {
                this->apply_transformation();
                this->apply_box3d_filter();
            } else {
                this->apply_box3d_filter();
                this->apply_transformation();
                return true;
            }
        } else {
            std::cout << "input cloud is not given, set the input cloud using set_input_cloud()\n";
            return false;
        }
    }

    void PclPreProcessor::set_input_cloud(pcl::PointCloud<POINT_TYPE>::Ptr input_cloud) {
        this->cloud = input_cloud;
    }

    void PclPreProcessor::set_transform_first(bool transform_first_) {
        this->transform_first = transform_first_;
    }

    void PclPreProcessor::set_leaf_size(double leaf_size_) {
        this->downsample = true;
        this->leaf_size = leaf_size_;
        this->downsampling_filter.setInputCloud(this->cloud);
        this->downsampling_filter.setLeafSize(this->leaf_size, this->leaf_size, this->leaf_size);
    }

    void PclPreProcessor::set_transform(Eigen::Affine3f transformation_) {
        this->transformation = transformation;
        this->transform_cloud = true;
    }

    void PclPreProcessor::set_boxfilter(double min_x_, double min_y_, double min_z_,
                       double max_x_, double max_y_, double max_z_) {
        this->min_x = min_x_;
        this->min_y = min_y_;
        this->min_z = min_z_;
        this->max_x = max_x_;
        this->max_y = max_y_;
        this->max_z = max_z_;

        pcl::ConditionAnd<POINT_TYPE>::Ptr temp_cond (new pcl::ConditionAnd<POINT_TYPE>());
        this->box3d_cond = temp_cond;

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "z", pcl::ComparisonOps::LT, this->max_z)));

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "z", pcl::ComparisonOps::GT, this->min_z)));

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "y", pcl::ComparisonOps::LT, this->max_y)));

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "y", pcl::ComparisonOps::GT, this->min_y)));

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "x", pcl::ComparisonOps::LT, this->max_x)));

        this->box3d_cond->addComparison(
            pcl::FieldComparison<POINT_TYPE>::Ptr (
                new pcl::FieldComparison<POINT_TYPE>(
                    "x", pcl::ComparisonOps::GT, this->min_x)));

        this->box3d_filter_handler.setInputCloud(this->cloud);
        this->box3d_filter_handler.setCondition(this->box3d_cond);
        this->box3d_filter = true;
    }
}


#endif // PCL_PREPROCESSOR_H_

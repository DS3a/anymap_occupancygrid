#include <cstdio>
#include <iostream>

#include "anymap.hpp"
#include "observation_source.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"


using namespace std::chrono_literals;

class AnyMapNode : public rclcpp::Node
{
public:
    AnyMapNode() : Node("anymap_node") {
        pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth/color/points", 15, std::bind(&AnyMapNode::pcl_callback, this, std::placeholders::_1)
        );

        pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
        pcl::ConditionAnd<POINT_TYPE>::Ptr range_cond (new pcl::ConditionAnd<POINT_TYPE>());
        z_obstacle_cond = range_cond;
        // just use the real realsense xD
        z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 3.27)));
        z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, 0.0)));

        spatial_obstacle_filter.setInputCloud(cloud);
        spatial_obstacle_filter.setCondition(z_obstacle_cond);


        this->grid_msg_ptr = std::shared_ptr<nav_msgs::msg::OccupancyGrid>(new nav_msgs::msg::OccupancyGrid);

        std::cout << "Initializing anymap\n";
        this->anymap_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
        *anymap_ptr.get() = anymap::init_anymap();

        this->test_source_ptr = std::shared_ptr<observation_source::ObservationSource>(new observation_source::ObservationSource(this->get_logger()));

        // testing

        anymap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("anymap", 10);

        std::cout << "initialized anymap, adding test layer\n";
        anymap::add_test_layer(this->anymap_ptr);
        std::cout << "added test layer, converting to rosmsg\n";
        anymap::test_as_occupancy_grid(this->anymap_ptr, this->grid_msg_ptr);

        std::cout << this->anymap_ptr->exists("test") << std::endl;
        std::cout << "The resolution is : " << this->grid_msg_ptr->info.resolution << std::endl;
        std::cout << "The height and width are : " << this->grid_msg_ptr->info.height << " " << this->grid_msg_ptr->info.width << std::endl;

        this->test_source_ptr->add_default_parameters();

        timer_ = this->create_wall_timer(
            500ms, std::bind(&AnyMapNode::timer_callback, this));

    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr anymap_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg_ptr;
    std::shared_ptr<grid_map::GridMap> anymap_ptr;

    std::shared_ptr<grid_map::GridMap> test_grid_map;

    std::shared_ptr<observation_source::ObservationSource> test_source_ptr;

    pcl::PointCloud<POINT_TYPE>::Ptr cloud = std::make_shared<pcl::PointCloud<POINT_TYPE>>();

    pcl::ConditionAnd<POINT_TYPE>::Ptr z_obstacle_cond;
    pcl::ConditionalRemoval<POINT_TYPE> spatial_obstacle_filter = pcl::ConditionalRemoval<POINT_TYPE>();


    int counter = 0;

    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
        sensor_msgs::msg::PointCloud2 obstacles_msg;
        pcl::fromROSMsg(*msg, *this->cloud);
        spatial_obstacle_filter.setInputCloud(this->cloud);
        spatial_obstacle_filter.filter(*this->cloud);
        Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();
        Eigen::Affine3f transform_x = Eigen::Affine3f::Identity();
        transform_y.rotate(Eigen::AngleAxisf(3.1415, Eigen::Vector3f::UnitY()));
        // transform_x.rotate(Eigen::AngleAxisf(1.57075, Eigen::Vector3f::UnitX()));
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
        pcl::transformPointCloud(*cloud, *transformed_cloud, transform_y);

        // pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud_x (new pcl::PointCloud<POINT_TYPE>());
        // pcl::transformPointCloud(*transformed_cloud, *transformed_cloud_x, transform_x);

        pcl::toROSMsg(*cloud, obstacles_msg);
        this->pcl_publisher->publish(obstacles_msg);

        cloud = transformed_cloud;

        this->counter++;
        if (counter == 15) {
            counter = 0;
            this->test_source_ptr->set_input_cloud(cloud);
            this->test_source_ptr->initialize_grid_map_geometry();
            this->test_source_ptr->add_layer_from_input_cloud("pcl");

            this->anymap_ptr = std::make_shared<grid_map::GridMap>(this->test_source_ptr->get_grid_map());
        }
    }

    void timer_callback() {
        grid_map::GridMapRosConverter conv;
        if (this->anymap_ptr->exists("pcl")) {
          std::cout << "found pcl layer publishing gridmap \n";
          conv.toOccupancyGrid(*this->anymap_ptr.get(), "pcl", 0, 1, *this->grid_msg_ptr.get());
          grid_msg_ptr->header.frame_id = "camera_link";
          this->anymap_publisher->publish(*this->grid_msg_ptr.get());
        }
    }
};


int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    /*
    std::shared_ptr<int> test_shared_ptr(new int);
    *test_shared_ptr.get() = 1;

    anymap::print_test_int();
    anymap::update_test_int(test_shared_ptr);
    anymap::print_test_int();

    *test_shared_ptr.get() = 23;
    anymap::print_test_int();

    NOTE shared_ptrs work the way Arc::Mutex works in rust
    */
    rclcpp::init(argc, argv);
    auto anymap_node = std::make_shared<AnyMapNode>();
    rclcpp::spin(anymap_node);


    return 0;
}

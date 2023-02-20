#include <cstdio>
#include <iostream>

#include "anymap.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"

#define POINT_TYPE pcl::PointXYZ

class AnyMapNode : public rclcpp::Node
{
public:
    AnyMapNode() : Node("anymap_node") {
        pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/depth_camera/points", 15, std::bind(&AnyMapNode::pcl_callback, this, std::placeholders::_1)
        );

        pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);
        pcl::ConditionAnd<POINT_TYPE>::Ptr range_cond (new pcl::ConditionAnd<POINT_TYPE>());
        z_obstacle_cond = range_cond;
        // just use the real realsense xD
        z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 20.27)));
        z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, 8.0)));

        spatial_obstacle_filter.setInputCloud(cloud);
        spatial_obstacle_filter.setCondition(z_obstacle_cond);
    }

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;

    pcl::PointCloud<POINT_TYPE>::Ptr cloud = std::make_shared<pcl::PointCloud<POINT_TYPE>>();

    pcl::ConditionAnd<POINT_TYPE>::Ptr z_obstacle_cond;
    pcl::ConditionalRemoval<POINT_TYPE> spatial_obstacle_filter = pcl::ConditionalRemoval<POINT_TYPE>();

    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

        sensor_msgs::msg::PointCloud2 obstacles_msg;
        pcl::fromROSMsg(*msg, *cloud);
        spatial_obstacle_filter.setInputCloud(cloud);
        spatial_obstacle_filter.filter(*cloud);

        pcl::toROSMsg(*cloud, obstacles_msg);
        this->pcl_publisher->publish(obstacles_msg);
    }
};


int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    rclcpp::init(argc, argv);
    auto anymap_node = std::make_shared<AnyMapNode>();
    rclcpp::spin(anymap_node);


    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg_ptr(new nav_msgs::msg::OccupancyGrid);

    std::cout << "Initializing anymap\n";
    std::shared_ptr<grid_map::GridMap> anymap_ptr(new grid_map::GridMap);
    *anymap_ptr.get() = anymap::init_anymap();
    std::cout << "initialized anymap, adding test layer\n";
    anymap::add_test_layer(anymap_ptr);
    std::cout << "added test layer, converting to rosmsg\n";
    // anymap::test_as_occupancy_grid(anymap_ptr, grid_msg_ptr);

    std::cout << anymap_ptr->exists("test") << std::endl;
    std::cout << "The resolution is : " << grid_msg_ptr->info.resolution << std::endl;
    std::cout << "The height and width are : " << grid_msg_ptr->info.height << " " << grid_msg_ptr->info.width << std::endl;

    return 0;
}

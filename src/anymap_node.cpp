/* TODO
 * - add all observation sources
 * - set the update flag when the pcl callback is called
 * - update the layer when the update flag is set to true
 * - post process the layer when all points are added to it
 * - aggregate all layers when the update costmap service is called
 * - sort the TF tree stuff, i.e. copy the current tf between base_link and odom,
 *        and publish that till the next time the map is updated
 */

#include <cstdio>
#include <iostream>

#include "anymap.hpp"
#include "observation_source.hpp"
#include "pcl_preprocessor.hpp"

#include "anymap_interfaces/srv/trigger_update.hpp"

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

#include "pcl_conversions/pcl_conversions.h"
#include "pcl/filters/conditional_removal.h"
#include "pcl/point_cloud.h"
#include "pcl/common/transforms.h"

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"

//testing
#include "layer_postprocessor.hpp"
#include <pcl/filters/voxel_grid.h>
#include "pratham_observation_sources.hpp"

using namespace std::chrono_literals;

static grid_map::GridMapRosConverter conv;

class AnyMapNode : public rclcpp::Node
{
public:
    AnyMapNode();
private:
    // pcl source 1
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_subscription;
    std::shared_ptr<observation_source::ObservationSource> test_source_ptr;
    layer_postprocessor::LayerPostProcessor test_postprocessor;
    pcl::PointCloud<POINT_TYPE>::Ptr cloud = boost::make_shared<pcl::PointCloud<POINT_TYPE>>();
    pcl::ConditionAnd<POINT_TYPE>::Ptr z_obstacle_cond;
    pcl::ConditionalRemoval<POINT_TYPE> spatial_obstacle_filter = pcl::ConditionalRemoval<POINT_TYPE>();
    void pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_publisher;


    rclcpp::Service<anymap_interfaces::srv::TriggerUpdate>::SharedPtr map_update_service;
    void update_anymap_callback(const std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Request> request,
                            std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Response> response);


    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr anymap_publisher;

    rclcpp::TimerBase::SharedPtr timer_;

    std::shared_ptr<nav_msgs::msg::OccupancyGrid> grid_msg_ptr;
    std::shared_ptr<grid_map::GridMap> anymap_ptr;


    int counter = 0;

    // to publish tf?
    void timer_callback();
};

AnyMapNode::AnyMapNode() : Node("anymap_node") {
    // initialize the anymap instance, to which layers will be added
    std::cout << "Initializing anymap\n";
    this->anymap_ptr = std::shared_ptr<grid_map::GridMap>(new grid_map::GridMap);
    *anymap_ptr.get() = anymap::init_anymap();

    // the anymap publisher
    anymap_publisher = this->create_publisher<nav_msgs::msg::OccupancyGrid>("anymap", 10);

    // initialize the service
    this->map_update_service = this->create_service<anymap_interfaces::srv::TriggerUpdate>
        ("~/trigger_update", std::bind(&AnyMapNode::update_anymap_callback, this, std::placeholders::_1, std::placeholders::_2));

    // subscribe to the pcl topics with individual calbacks
    pcl_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 15, std::bind(&AnyMapNode::pcl_callback, this, std::placeholders::_1)
        ); // this is for the realsense, which will not be used

        // the observation source that the above subscription will feed into
    this->test_source_ptr = std::shared_ptr<observation_source::ObservationSource>(
        new observation_source::ObservationSource("pcl", this->anymap_ptr));
    this->test_postprocessor.set_layer_name("pcl");
    this->test_postprocessor.set_input_grid(this->anymap_ptr);


    // this is a test
    pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);

    pcl::ConditionAnd<POINT_TYPE>::Ptr range_cond (new pcl::ConditionAnd<POINT_TYPE>());
    z_obstacle_cond = range_cond;
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::LT, 0.80)));
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("z", pcl::ComparisonOps::GT, -0.3)));
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::LT, 4.5)));
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("x", pcl::ComparisonOps::GT, -4)));
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::LT, 4.0)));
    z_obstacle_cond->addComparison(pcl::FieldComparison<POINT_TYPE>::Ptr (new pcl::FieldComparison<POINT_TYPE>("y", pcl::ComparisonOps::GT, -4.0)));

    spatial_obstacle_filter.setInputCloud(cloud);
    spatial_obstacle_filter.setCondition(z_obstacle_cond);


    this->grid_msg_ptr = std::shared_ptr<nav_msgs::msg::OccupancyGrid>(new nav_msgs::msg::OccupancyGrid);

    // TODO replace this with an action server that updates the map everytime it is called
    timer_ = this->create_wall_timer(150ms, std::bind(&AnyMapNode::timer_callback, this));
}

void AnyMapNode::pcl_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {

    sensor_msgs::msg::PointCloud2 obstacles_msg;
    pcl::fromROSMsg(*msg, *this->cloud);

    // std::cout << "Initial size : " << this->cloud->points.size() << std::endl;
    pcl::PointCloud<POINT_TYPE>::Ptr cloud_filtered (new pcl::PointCloud<POINT_TYPE>());
    pcl::VoxelGrid<POINT_TYPE> sor;
    sor.setInputCloud (cloud);
    sor.setLeafSize (0.13f, 0.13f, 0.13f);
    sor.filter (*cloud_filtered);
    this->test_source_ptr->set_point_weight(0.6);
    // cloud_filtered = this->cloud;
    // std::cout << "Downsampled size : " << cloud_filtered->points.size() << std::endl;

    Eigen::Affine3f transform_y = Eigen::Affine3f::Identity();

    /*
    transform_y.rotate(Eigen::AngleAxisf(-3.14159/2.0, Eigen::Vector3f::UnitX()));
    transform_y.rotate(Eigen::AngleAxisf(3.14159/2.0, Eigen::Vector3f::UnitY()));
    */

    transform_y.rotate(Eigen::AngleAxisf(3.14159/2.0, Eigen::Vector3f::UnitY()));
    transform_y.rotate(Eigen::AngleAxisf(-3.14159/2.0, Eigen::Vector3f::UnitZ()));
    // NOTE camera +z became map +x, camera +x became map -y
    // transform_y.translate(Eigen::Vector3f(2, 0, -5));
    pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<POINT_TYPE>());
    pcl::transformPointCloud(*cloud_filtered, *transformed_cloud, transform_y);

    spatial_obstacle_filter.setInputCloud(transformed_cloud);
    spatial_obstacle_filter.filter(*transformed_cloud);


    pcl::toROSMsg(*transformed_cloud, obstacles_msg);
    obstacles_msg.header.frame_id = "camera_link";
    this->pcl_publisher->publish(obstacles_msg);

    cloud = transformed_cloud;

    // TODO test and see whetHer this works well in realtime
    this->counter++;
    if (counter == 8) {
        this->test_source_ptr->clear_layer();
        this->test_source_ptr->set_update_flag();
        counter = 0;
        this->test_source_ptr->set_input_cloud(transformed_cloud);
        this->test_source_ptr->update_layer();

        this->test_postprocessor.process_layer();
   }

}

void AnyMapNode::update_anymap_callback(const std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Request> request,
                            std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Response> response) {
    std::cout << "anymap update map service called\n";
    response->success = true;
}

void AnyMapNode::timer_callback() {
    if (this->anymap_ptr->exists("pclProcessed")) {
        // std::cout << "found pcl layer publishing gridmap \n";
        conv.toOccupancyGrid(*this->anymap_ptr.get(), "pclProcessed", 0, 1, *this->grid_msg_ptr.get());
        grid_msg_ptr->header.frame_id = "camera_link";
        this->anymap_publisher->publish(*this->grid_msg_ptr.get());
    } else {
        // std::cout << "pcl layer not found\n";
    }
}


int main(int argc, char **argv)
{
    (void) argc;
    (void) argv;

    /*
    boost::shared_ptr<int> test_shared_ptr(new int);
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

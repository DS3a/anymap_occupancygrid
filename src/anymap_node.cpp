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

#include "pratham_observation_sources.hpp"

using namespace std::chrono_literals;

static grid_map::GridMapRosConverter conv;

class AnyMapNode : public rclcpp::Node
{
public:
    AnyMapNode();
private:
    // pcl source 1
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr potholes_subscription;
    pcl::PointCloud<POINT_TYPE>::Ptr potholes_cloud;
    void potholes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

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


    pcl::PointCloud<POINT_TYPE>::Ptr temp_potholes_cloud (new pcl::PointCloud<POINT_TYPE>());
    this->potholes_cloud = temp_potholes_cloud;
    // subscribe to the pcl topics with individual calbacks
    potholes_subscription = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/camera/depth/color/points", 15, std::bind(&AnyMapNode::potholes_callback, this, std::placeholders::_1)
        ); // this is for the realsense, which will not be used

    // test
    pcl_publisher = this->create_publisher<sensor_msgs::msg::PointCloud2>("filtered_points", 10);


    this->grid_msg_ptr = std::shared_ptr<nav_msgs::msg::OccupancyGrid>(new nav_msgs::msg::OccupancyGrid);

    // TODO replace this with an action server that updates the map everytime it is called
    timer_ = this->create_wall_timer(150ms, std::bind(&AnyMapNode::timer_callback, this));

    pratham_observation_sources::init_potholes_src(this->potholes_cloud, this->anymap_ptr);
}

void AnyMapNode::potholes_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 obstacles_msg;
    std::cout << "updating potholes_clouddd\n";
    pcl::fromROSMsg(*msg, *this->potholes_cloud);
    std::cout << "acquired potholes, the size before preprocessing is\n";
    // std::cout << this->potholes_cloud.points.size() << std::endl;

   std::cout << this->potholes_cloud->points.size() << std::endl;

   pratham_observation_sources::update_potholes_layer();


   std::cout << "attempting to publish processed cloud\n";
   std::cout << this->potholes_cloud->points.size() << std::endl;
   pcl::toROSMsg(*(this->potholes_cloud), obstacles_msg);
   obstacles_msg.header.frame_id = "camera_link";
   std::cout << "converted to rosmsg, not publishing\n";

   this->pcl_publisher->publish(obstacles_msg);
}

void AnyMapNode::update_anymap_callback(const std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Request> request,
                            std::shared_ptr<anymap_interfaces::srv::TriggerUpdate::Response> response) {
    std::cout << "anymap update map service called\n";
    response->success = true;
}

void AnyMapNode::timer_callback() {
    if (this->anymap_ptr->exists("potholes_layerProcessed")) {
        std::cout << "found pcl layer publishing gridmap \n";
        conv.toOccupancyGrid(*this->anymap_ptr.get(), "potholes_layerProcessed", 0, 1, *this->grid_msg_ptr.get());
        grid_msg_ptr->header.frame_id = "camera_link";
        this->anymap_publisher->publish(*this->grid_msg_ptr.get());
    } else {
        std::cout << "pcl layer not found\n";
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

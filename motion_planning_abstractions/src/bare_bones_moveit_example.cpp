#include <memory>
#include <chrono>
#include <functional>
#include "rclcpp/rclcpp.hpp"

#include "motion_planning_abstractions/bare_bones_moveit.hpp"

int main(int argc, const char** argv){
    rclcpp::init(argc,argv);
    auto node_ = std::make_shared<rclcpp::Node>("bare_bones_moveit_example");
    
    auto motion_planning_abstractions_interface = std::make_shared<BareBonesMoveit>(node_);
    
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);
    executor->spin();
    
    rclcpp::shutdown();
}
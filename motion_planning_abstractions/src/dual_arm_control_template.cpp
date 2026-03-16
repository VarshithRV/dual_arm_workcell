#include <functional>
#include <cmath>
#include <memory>
#include <cstdlib>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "motion_planning_abstractions/dual_arm_waypoint_programming.hpp"
#include "C01_deconing_waypoints.hpp"

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node_=std::make_shared<rclcpp::Node>("dual_arm_control_template");

    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor=std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node_);

    auto wp_entry_point = std::make_shared<BareBonesMoveit>(node_);
    
    auto execute_service_callback = [wp_entry_point,node_](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
        auto LOGGER = node_->get_logger();
        auto waypoints = Waypoints(); // this is where all the waypoints are
        
        auto current_left_pose = wp_entry_point->get_current_ee_pose("left");
        std::vector<geometry_msgs::msg::Pose>left_waypoints{
            *current_left_pose,
            [current_left_pose](){
                auto pose = *current_left_pose;
                pose.position.x += 0.1;
                return pose;
            }()
        };

        // // asynchronous calling of cubic trajectory example
        // RCLCPP_INFO(node_->get_logger(),"Started the left cubic server asyncly");
        // auto exec_future = async_start_execute_waypoints_cubic(left_waypoints,std::vector<double>{0,0.5},0.3,0.01,"left");
        // block_till_response_execute_cubic_trajectory(exec_future, 5000ms);

        RCLCPP_INFO(node_->get_logger(), "Starting left and right joint state motions in parallel");
        
        wp_entry_point->move_to_joint_positions(waypoints.right_rest_state.joint_values,wp_entry_point->right_move_group_interface_);
        
        //waypoint to the cone
        wp_entry_point->move_to_joint_positions(waypoints.right_wp1.joint_values,wp_entry_point->right_move_group_interface_);
        std::vector<geometry_msgs::msg::Pose> to_the_cone{
            waypoints.right_wp2.pose,
            waypoints.right_wp3.pose,
        };
        wp_entry_point->execute_waypoints_cubic(to_the_cone,std::vector<double>{0.75,0.5},0.3,0.07,"right");
        
        // twist
        std::vector<geometry_msgs::msg::Pose> twist{
            waypoints.right_wp4.pose,
        };
        wp_entry_point->execute_waypoints_cubic(twist,std::vector<double>{0.5},0.3,0.0,"right");

        // get down
        std::vector<geometry_msgs::msg::Pose> down{
            waypoints.right_wp5.pose,
        };
        wp_entry_point->execute_waypoints_cubic(down,std::vector<double>{0.7},0.3,0.05,"right");

        // backoff
        std::vector<geometry_msgs::msg::Pose> back_off{
            waypoints.right_wp6.pose,
            waypoints.right_wp7.pose,
            waypoints.right_wp8.pose,
        };
        wp_entry_point->execute_waypoints_cubic(back_off,std::vector<double>{1.0,1.0,0.7},0.3,0.05,"right");
    };

    auto callback_group = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto execute_server = node_->create_service<std_srvs::srv::Trigger>(
        "~/execute",
        execute_service_callback,
        rmw_qos_profile_services_default,
        callback_group
    );

    executor->spin();
}
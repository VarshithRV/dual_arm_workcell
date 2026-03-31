// library to manage servo
// creates and internal state machine
// hosts servers for unprepare, prepare, stop and start servo
// internal velocity publisher
// requires the parameters :
// std::string servo_node_ns
// std::string trajectory_controller
// std::string joint_vel_controller
// double alpha(range from 0 to 1), bigger alpha means more filtering
#ifndef MOTION_PLANNING_ABSTRACTIONS_EE_SERVO_HPP
#define MOTION_PLANNING_ABSTRACTIONS_EE_SERVO_HPP

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <sstream>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_msgs/msg/int16.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "motion_planning_abstractions_msgs/srv/generate_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class EEServo{

public:
    enum class State{
        NOT_READY = 0,
        READY = 1,
        ARMED = 2
    };

    EEServo();

    EEServo(rclcpp::Node::SharedPtr node);

    bool prepare_servo_();

    bool unprepare_servo_();

    bool start_servo_();

    bool stop_servo_();

    // set velocity setpoint
    void set_vel_setpoint_(geometry_msgs::msg::TwistStamped vel);

    // iir filter on velocity at a constant rate
    void iir_filter_(geometry_msgs::msg::TwistStamped input, geometry_msgs::msg::TwistStamped& output);

private:
    
    // internal data
    State current_state_; // current state of the state machine
    std::string servo_node_ns_;// name space of the servo node
    std::string joint_traj_controller_; // string of the trajectory controller
    std::string joint_vel_controller_; // string of the controller active during servo
    geometry_msgs::msg::TwistStamped current_velocity_setpoint_; // the current velocity to be published
    geometry_msgs::msg::TwistStamped filtered_velocity_setpoint_; // filtered velocity
    double alpha_;

    // clock
    rclcpp::Clock wall_clock_;
    
    // node
    rclcpp::Node::SharedPtr node_;

    // publisher
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr state_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr velocity_publisher_;

    // servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_servo_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_servo_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unprepare_servo_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_servo_server_;

    // clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;

    // timers
    rclcpp::TimerBase::SharedPtr state_publisher_timer_;
    rclcpp::TimerBase::SharedPtr velocity_publisher_timer_;
    rclcpp::TimerBase::SharedPtr filter_velocity_timer_;

    // callback group
    rclcpp::CallbackGroup::SharedPtr reentrant_callback_group_;
    rclcpp::CallbackGroup::SharedPtr mex_callback_group_;
};
#endif
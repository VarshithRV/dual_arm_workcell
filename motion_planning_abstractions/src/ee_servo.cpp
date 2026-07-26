// library to manage servo
// creates and internal state machine
// hosts servers for unprepare, prepare, stop and start servo
// internal velocity publisher
// std::string servo_node_ns
// std::string trajectory_controller
// std::string joint_vel_controller
// double alpha(range from 0 to 1)

// TODO list
// add the switch mode clients
// remove stop and start servo client
// modify the prepare servo call back to do the right things, prepare and unprepare only switches the controller now
// once the node spawns, it should call the switch mode server to set to twist mode

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
#include "moveit_msgs/srv/servo_command_type.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "motion_planning_abstractions_msgs/srv/generate_trajectory.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

#include "motion_planning_abstractions/ee_servo.hpp"

EEServo::EEServo(){}

EEServo::EEServo(rclcpp::Node::SharedPtr node){
    
    if(node!=nullptr)
        node_ = node;
    else{
        std::cout<<"No node passed!!!"<<std::endl;
        return;
    }

    auto LOGGER = node_->get_logger();
    
    wall_clock_ = rclcpp::Clock(rcl_clock_type_t::RCL_SYSTEM_TIME);

    current_state_ = State::NOT_READY;

    current_velocity_setpoint_ = geometry_msgs::msg::TwistStamped();
    current_velocity_setpoint_.header.frame_id = "world";

    filtered_velocity_setpoint_ = geometry_msgs::msg::TwistStamped();
    filtered_velocity_setpoint_.header.frame_id = "world";

    // callback group for servers
    reentrant_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    mex_callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

    // declare and get the ros parameters
    if(!node->has_parameter("servo_node_ns"))
        node->declare_parameter<std::string>("servo_node_ns");
    if(!node->has_parameter("joint_traj_controller"))
        node->declare_parameter<std::string>("joint_traj_controller");
    if(!node->has_parameter("joint_vel_controller"))
        node->declare_parameter<std::string>("joint_vel_controller");
    if(!node->has_parameter("alpha"))
        node->declare_parameter<double>("alpha",0.8);

    servo_node_ns_ = node_->get_parameter("servo_node_ns").as_string();
    joint_traj_controller_ = node_->get_parameter("joint_traj_controller").as_string();
    joint_vel_controller_ = node_->get_parameter("joint_vel_controller").as_string();
    alpha_ = node_->get_parameter("alpha").as_double();

    RCLCPP_INFO(node_->get_logger(), "servo_node_ns: %s", servo_node_ns_.c_str());
    RCLCPP_INFO(node_->get_logger(), "joint_traj_controller: %s", joint_traj_controller_.c_str());
    RCLCPP_INFO(node_->get_logger(), "joint_vel_controller: %s", joint_vel_controller_.c_str());
    RCLCPP_INFO(node_->get_logger(), "alpha: %.4f", alpha_);

    // init publishers
    auto qos_profile = rclcpp::QoS(10);
    state_publisher_ = node_->create_publisher<std_msgs::msg::Int16>("~/current_servo_state",qos_profile);

    velocity_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(servo_node_ns_ + "/delta_twist_cmds",qos_profile);

    // init clients
    switch_command_type_client_ = node_->create_client<moveit_msgs::srv::ServoCommandType>(servo_node_ns_+"/switch_command_type",rmw_qos_profile_services_default,reentrant_callback_group_);
    switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("/controller_manager/switch_controller",rmw_qos_profile_services_default,reentrant_callback_group_);

    // init servers
    prepare_servo_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/prepare_servo",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            res->success = prepare_servo_();
        },
        rmw_qos_profile_services_default,
        reentrant_callback_group_
    );
    unprepare_servo_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/unprepare_servo",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            res->success = unprepare_servo_();
        },
        rmw_qos_profile_services_default,
        reentrant_callback_group_
    );
    start_servo_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/start_servo",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            res->success = start_servo_();
        },
        rmw_qos_profile_services_default,
        reentrant_callback_group_
    );
    stop_servo_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/stop_servo",
        [this](std::shared_ptr<std_srvs::srv::Trigger::Request>,std::shared_ptr<std_srvs::srv::Trigger::Response> res){
            res->success = stop_servo_();
        },
        rmw_qos_profile_services_default,
        reentrant_callback_group_
    );

    // Init timers
    state_publisher_timer_ = node_->create_wall_timer(
        50ms,
        [this](){
            auto msg = std_msgs::msg::Int16();
            msg.data = static_cast<int>(current_state_);
            state_publisher_->publish(msg);
        },
        reentrant_callback_group_
    );

    velocity_publisher_timer_ = node_->create_wall_timer(
        10ms,
        [this,LOGGER](){
            filtered_velocity_setpoint_.header.stamp = wall_clock_.now();
            if(current_state_ != State::NOT_READY){
                velocity_publisher_->publish(filtered_velocity_setpoint_);
            }
        },
        reentrant_callback_group_
    );

    filter_velocity_timer_ = node_->create_wall_timer(
        50ms,
        [this,LOGGER](){
            iir_filter_(current_velocity_setpoint_,filtered_velocity_setpoint_);
        },
        reentrant_callback_group_
    );
}

void EEServo::initialize_servo_interface_(){
    auto LOGGER = node_->get_logger();
    auto switch_command_type_req = std::make_shared<moveit_msgs::srv::ServoCommandType::Request>();
    switch_command_type_req->command_type = switch_command_type_req->TWIST;
    auto switch_command_type_future = switch_command_type_client_->async_send_request(switch_command_type_req);
    if(switch_command_type_future.wait_for(5s)!=std::future_status::ready){
        RCLCPP_ERROR(LOGGER,"Switch command type client timed out!");
    }
    else{
        RCLCPP_INFO(LOGGER,"Switch command type to twist finished");
    }
}

void EEServo::set_vel_setpoint_(geometry_msgs::msg::TwistStamped vel){
    if(current_state_ == State::ARMED)
        current_velocity_setpoint_ = vel;
    else{
        RCLCPP_ERROR(node_->get_logger(),"Not armed, cannot set");
        current_velocity_setpoint_ = geometry_msgs::msg::TwistStamped();
    }
}

void EEServo::iir_filter_(geometry_msgs::msg::TwistStamped input, geometry_msgs::msg::TwistStamped& output){
    auto linear_x = (1-alpha_)*input.twist.linear.x + alpha_*output.twist.linear.x;
    auto linear_y = (1-alpha_)*input.twist.linear.y + alpha_*output.twist.linear.y;
    auto linear_z = (1-alpha_)*input.twist.linear.z + alpha_*output.twist.linear.z;
    auto angular_x = (1-alpha_)*input.twist.angular.x + alpha_*output.twist.angular.x;
    auto angular_y = (1-alpha_)*input.twist.angular.y + alpha_*output.twist.angular.y;
    auto angular_z = (1-alpha_)*input.twist.angular.z + alpha_*output.twist.angular.z;
    output.twist.linear.x = std::abs(linear_x)<1e-6?0.0:linear_x;
    output.twist.linear.y = std::abs(linear_y)<1e-6?0.0:linear_y;
    output.twist.linear.z = std::abs(linear_z)<1e-6?0.0:linear_z;
    output.twist.angular.x = std::abs(angular_x)<1e-6?0.0:angular_x;
    output.twist.angular.y = std::abs(angular_y)<1e-6?0.0:angular_y;
    output.twist.angular.z = std::abs(angular_z)<1e-6?0.0:angular_z;
}


bool EEServo::prepare_servo_(){
    auto LOGGER = node_->get_logger();

    if(current_state_==State::ARMED){
        RCLCPP_ERROR(LOGGER,"In ARMED state, not safe to switch controllers");
        return false;
    }
    if(current_state_==State::READY){
        RCLCPP_WARN(LOGGER,"Warning, already in READY state, switching controller anyways");
    }

    auto switch_controller_msg = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_controller_msg->activate_controllers = std::vector<std::string>{joint_vel_controller_};
    switch_controller_msg->deactivate_controllers = std::vector<std::string>{joint_traj_controller_};
    switch_controller_msg->strictness = switch_controller_msg->BEST_EFFORT;
    switch_controller_msg->timeout = rclcpp::Duration(5s);

    auto switch_controller_future = switch_controller_client_->async_send_request(switch_controller_msg);
    if(switch_controller_future.wait_for(15s) != std::future_status::ready){
        RCLCPP_ERROR(LOGGER,"Switch Controller Timed out");
        return false;
    }
    else{
        if(switch_controller_future.get()->ok){
            RCLCPP_INFO(LOGGER,"Switched to %s controller from %s controller",joint_vel_controller_.c_str(),joint_traj_controller_.c_str());
            current_velocity_setpoint_.twist = geometry_msgs::msg::Twist();
            current_state_ = State::READY;
            return true;
        }
        else{
            RCLCPP_ERROR(LOGGER,"Switching controller failed while tryint to switch to %s from %s",joint_vel_controller_.c_str(),joint_traj_controller_.c_str());
            return false;
        }
    }
}

bool EEServo::unprepare_servo_(){
    auto LOGGER = node_->get_logger();

    if(current_state_==State::ARMED){
        RCLCPP_ERROR(LOGGER,"In ARMED state, not safe to switch controllers");
        return false;
    }
    if(current_state_==State::NOT_READY){
        RCLCPP_WARN(LOGGER,"Warning, already in NOT READY state, switching controller anyways");
    }

    auto switch_controller_msg = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
    switch_controller_msg->activate_controllers = std::vector<std::string>{joint_traj_controller_};
    switch_controller_msg->deactivate_controllers = std::vector<std::string>{joint_vel_controller_};
    switch_controller_msg->strictness = switch_controller_msg->BEST_EFFORT;
    switch_controller_msg->timeout = rclcpp::Duration(5s);

    auto switch_controller_future = switch_controller_client_->async_send_request(switch_controller_msg);
    if(switch_controller_future.wait_for(5s)!=std::future_status::ready){
        RCLCPP_ERROR(LOGGER,"Switch controller timed out while switching from %s to %s",joint_vel_controller_,joint_traj_controller_);
        return false;
    }
    else{
        RCLCPP_INFO(LOGGER,"Switch controller finished");
        auto success = switch_controller_future.get()->ok;
        if(success)
            current_state_ = State::NOT_READY;
        return success;
    }
    
}

bool EEServo::start_servo_(){
    auto LOGGER = node_->get_logger();
    if(current_state_!=State::READY){
        RCLCPP_ERROR(LOGGER,"Current state is not ready, make sure to prepare_servo before starting");
        return false;
    }
    current_state_=State::ARMED;
    return true;
}

bool EEServo::stop_servo_(){
    auto LOGGER = node_->get_logger();
    if(current_state_!=State::NOT_READY){
        current_state_ = State::READY;
        current_velocity_setpoint_ = geometry_msgs::msg::TwistStamped();
        return true;
    }
    return false;
}
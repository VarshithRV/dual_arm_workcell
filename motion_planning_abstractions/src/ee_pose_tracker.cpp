#include <functional>
#include <memory>
#include <cmath>
#include <chrono>
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

#include "motion_planning_abstractions/ee_pose_tracker.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/bare_bones_moveit.hpp"

PoseTracker::PoseTracker(rclcpp::Node::SharedPtr node){
    if(node == nullptr){
        std::cout<<"Node passed is a nullptr"<<std::endl;
        return;
    }
    else{
        node_=node;
    }

    // init variables
    servo_interface_ = std::make_shared<EEServo>(node_);
    single_arm_control_interface_ = std::make_shared<BareBonesMoveit>(node_);

    output_velocity_ = geometry_msgs::msg::Twist();
    current_state_=State::UN_PREPPED;
    target_pose_ = nullptr;

    // get ros parameters
    node_->declare_parameter<double>("linear_P",1.0);
    node_->declare_parameter<double>("linear_D",0.0);
    node_->declare_parameter<double>("angular_P",1.0);
    node_->declare_parameter<double>("angular_D",0.0);

    linear_P_ = node_->get_parameter("linear_P").as_double();
    linear_D_ = node_->get_parameter("linear_D").as_double();
    angular_P_ = node_->get_parameter("angular_P").as_double();
    angular_D_ = node_->get_parameter("angular_D").as_double();

    mex_cb_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parallel_cb_group_=node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    wall_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // services
    prepare_tracking_server_=node_->create_service<std_srvs::srv::Trigger>(
        "~/prepare_tracker",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success=prepare_tracker_();
        },
        rmw_qos_profile_services_default,
        mex_cb_group_
    );

    unprepare_tracking_server_=node_->create_service<std_srvs::srv::Trigger>(
        "~/unprepare_tracker",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success=unprepare_tracker_();
        },
        rmw_qos_profile_services_default,
        mex_cb_group_
    );

    start_tracking_server_=node_->create_service<std_srvs::srv::Trigger>(
        "~/start_tracker",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success=start_tracking_();
        },
        rmw_qos_profile_services_default,
        mex_cb_group_
    );

    stop_tracking_server_=node_->create_service<std_srvs::srv::Trigger>(
        "~/stop_tracker",
        [this](std_srvs::srv::Trigger::Request::SharedPtr, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success=stop_tracking_();
        },
        rmw_qos_profile_services_default,
        mex_cb_group_
    );
}

bool PoseTracker::prepare_tracker_(){
    if(node_==nullptr){
        std::cout<<"Node is not initialized"<<std::endl;
        return false;
    }
    if(single_arm_control_interface_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"Bare Bones Moveit Interface is not initialized");
        return false;
    }
    if(servo_interface_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"Servo Interface is not initialized");
        return false;
    }
    if(current_state_==State::PREPPED){
        RCLCPP_INFO(node_->get_logger(),"Already prepped");
        return true;
    }
    if(current_state_==State::TRACKING){
        RCLCPP_INFO(node_->get_logger(),"Already Tracking, dangerous to stop, stop tracking first");
        return false;
    }
    
    auto prep_servo_output = servo_interface_->prepare_servo_();
    auto start_servo_output = servo_interface_->start_servo_();
    
    if(prep_servo_output&&start_servo_output){
        current_state_=State::PREPPED;
        return true;
    }
    else
        return false;
}

bool PoseTracker::unprepare_tracker_(){
    if(node_==nullptr){
        std::cout<<"Node is not initialized"<<std::endl;
        return false;
    }
    if(single_arm_control_interface_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"Bare Bones Moveit Interface is not initialized");
        return false;
    }
    if(servo_interface_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"Servo Interface is not initialized");
        return false;
    }
    if(current_state_==State::UN_PREPPED){
        RCLCPP_INFO(node_->get_logger(),"Already un prepped");
        return true;
    }
    if(current_state_==State::TRACKING){
        RCLCPP_INFO(node_->get_logger(),"Already Tracking, dangerous to stop, stop tracking first");
        return false;
    }

    auto stop_servo_output = servo_interface_->stop_servo_();
    auto unprep_servo_output = servo_interface_->unprepare_servo_();
    
    if(unprep_servo_output&&stop_servo_output){
        current_state_=State::UN_PREPPED;
        return true;
    }
    else
        return false;
}

bool PoseTracker::start_tracking_(){
    if(target_pose_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"No target pose set, not gonna start tracking");
        return false;
    }
    if(current_state_==State::TRACKING){
        RCLCPP_INFO(node_->get_logger(),"Already tracking");
        return true;
    }
    if(current_state_==State::UN_PREPPED){
        RCLCPP_ERROR(node_->get_logger(),"The State is un prepared, prepare tracker first");
        return false;
    }
    current_state_=State::TRACKING;
    return true;
}

bool PoseTracker::stop_tracking_(){
    if(current_state_==State::PREPPED){
        RCLCPP_INFO(node_->get_logger(),"Not Tracking already");
        return true;
    }
    if(current_state_==State::UN_PREPPED){
        RCLCPP_ERROR(node_->get_logger(),"The State is un prepared, prepare tracker first");
        return false;
    }
    current_state_=State::PREPPED;
    return true;
}

void PoseTracker::set_target_pose_(const geometry_msgs::msg::Pose& target_pose){
    if(target_pose_==nullptr){
        std::make_shared<geometry_msgs::msg::Pose>(target_pose);
    }
    else{
        *target_pose_ = target_pose;
    }
}

void PoseTracker::clear_target_pose_(){
    if(target_pose_!=nullptr){
        target_pose_ = nullptr;
    }
}

void PoseTracker::control_robot_timer_cb_(){
    // get the current pose
    // get the target pose
    // if in the right state, set the right velocity, other wise set velocity as 0
    auto current_vel_setpoint = geometry_msgs::msg::TwistStamped();
    current_vel_setpoint.header.frame_id = "world";
    current_vel_setpoint.header.stamp = wall_clock_->now();
    
    if(node_==nullptr || servo_interface_==nullptr || single_arm_control_interface_==nullptr){
        return;
    }
    if(current_state_==State::UN_PREPPED){
        return;
    }
    if(current_state_==State::PREPPED){
        servo_interface_->set_vel_setpoint_(geometry_msgs::msg::TwistStamped());
    }
    if(current_state_==State::TRACKING){
        if(target_pose_==nullptr)
        return;
        // compute a velocity and publish
    }
}

Eigen::Vector3d PoseTracker::get_linear_error(
    const Eigen::Vector3d& current_position,
    const Eigen::Vector3d& target_position
){
    return target_position-current_position;
}

Eigen::Vector3d PoseTracker::get_angular_error( 
    const Eigen::Quaterniond& current_orientation,
    const Eigen::Quaterniond& target_orientation
){
    auto error_q = target_orientation*current_orientation.inverse();
    auto error_angle_axis = Eigen::AngleAxisd(error_q);
    return Eigen::Vector3d{error_angle_axis.angle()*error_angle_axis.axis()};
}
#include <functional>
#include <memory>
#include <cmath>
#include <chrono>
#include "std_msgs/msg/int16.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>

using namespace std::chrono_literals;

#include "motion_planning_abstractions/ee_pose_tracker.hpp"

PoseTracker::PoseTracker(rclcpp::Node::SharedPtr node){
    std::cout<<"Starting to setup the pose tracker"<<std::endl;

    if(node == nullptr){
        std::cout<<"Node passed is a nullptr"<<std::endl;
        return;
    }
    else{
        node_=node;
    }

    // init variables
    RCLCPP_INFO(node_->get_logger(),"Creating a servo interface");
    servo_interface_ = std::make_shared<EEServo>(node_);
    RCLCPP_INFO(node_->get_logger(),"Createing a single_arm_control_interface");
    single_arm_control_interface_ = std::make_shared<BareBonesMoveit>(node_);
    RCLCPP_INFO(node_->get_logger(),"Created both the interfaces");

    if(servo_interface_==nullptr){
        RCLCPP_INFO(node_->get_logger(),"Servo interface is null");
    }
    if(single_arm_control_interface_==nullptr){
        RCLCPP_INFO(node_->get_logger(),"Single arm control interface is null");
    }

    output_velocity_ = geometry_msgs::msg::Twist();
    current_state_=State::UN_PREPPED;
    target_pose_ = nullptr;

    // get ros parameters
    if(!node->has_parameter("linear_P"))
        node_->declare_parameter<double>("linear_P",1.0);
    if(!node->has_parameter("linear_D"))
        node_->declare_parameter<double>("linear_D",0.0);
    if(!node->has_parameter("angular_P"))
        node_->declare_parameter<double>("angular_P",1.0);
    if(!node->has_parameter("angular_D"))
        node_->declare_parameter<double>("angular_D",0.0);
    if(!node->has_parameter("max_velocity"))
        node_->declare_parameter("max_velocity",1.0);

    linear_P_ = node_->get_parameter("linear_P").as_double();
    linear_D_ = node_->get_parameter("linear_D").as_double();
    angular_P_ = node_->get_parameter("angular_P").as_double();
    angular_D_ = node_->get_parameter("angular_D").as_double();
    max_velocity_ = node_->get_parameter("max_velocity").as_double();
    
    RCLCPP_INFO(node_->get_logger(),"linear_P : %.2f",linear_P_);
    RCLCPP_INFO(node_->get_logger(),"linear_D : %.2f",linear_D_);
    RCLCPP_INFO(node_->get_logger(),"angular_P : %.2f",angular_P_);
    RCLCPP_INFO(node_->get_logger(),"angular_D : %.2f",angular_D_);

    mex_cb_group_=node_->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    parallel_cb_group_=node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    wall_clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    // publishers
    current_state_publisher_ = node_->create_publisher<std_msgs::msg::Int16>("~/current_pose_tracker_state",10);

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

    RCLCPP_INFO(node_->get_logger(),"All services are initialized, creating timers");

    current_state_publisher_timer_ = node_->create_wall_timer(
        50ms,
        [this](){
            auto msg = std_msgs::msg::Int16();
            if(current_state_publisher_ != nullptr){
                msg.data = static_cast<int>(current_state_);
                current_state_publisher_->publish(msg);
            }
        },
        parallel_cb_group_
    );
    RCLCPP_INFO(node_->get_logger(),"All timers initialized, pose tracker setup done");

    control_robot_timer_ = node_->create_wall_timer(
        100ms,
        [this](){
            control_robot_timer_cb_();
        },
        parallel_cb_group_
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
        // return true;
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
        // return true;
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
    if(current_state_==State::TRACKING){
        RCLCPP_INFO(node_->get_logger(),"Already tracking");
        return true;
    }
    if(current_state_==State::UN_PREPPED){
        RCLCPP_ERROR(node_->get_logger(),"The State is un prepared, prepare tracker first");
        return false;
    }
    if(target_pose_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"No target pose set, not gonna start tracking");
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

void PoseTracker::set_target_pose_( geometry_msgs::msg::Pose target_pose){
    if(target_pose_==nullptr){
        target_pose_ = std::make_shared<geometry_msgs::msg::Pose>(target_pose);
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
        current_vel_setpoint.twist = geometry_msgs::msg::Twist();
    }
    if(current_state_==State::TRACKING){
        // compute a velocity and publish
        if(target_pose_==nullptr){ // if the state machine is working right, this case never happens
            current_vel_setpoint.twist = geometry_msgs::msg::Twist();
        }
        else{
            auto current_pose = single_arm_control_interface_->get_current_ee_pose();
            Eigen::Vector3d current_position{
                current_pose->position.x,
                current_pose->position.y,
                current_pose->position.z
            };
            Eigen::Vector3d target_position{
                target_pose_->position.x,
                target_pose_->position.y,
                target_pose_->position.z
            };
            auto linear_error = target_position-current_position;
            auto linear_vel = linear_P_ * linear_error;
            // cap velocity
            Eigen::Vector3d capped_linear_vel;
            if(linear_vel.norm()>max_velocity_){
                capped_linear_vel = linear_vel * max_velocity_/linear_vel.norm();
            }
            
            Eigen::Quaterniond current_orientation{
                current_pose->orientation.w,
                current_pose->orientation.x,
                current_pose->orientation.y,
                current_pose->orientation.z
            };
            Eigen::Quaterniond target_orientation{
                target_pose_->orientation.w,
                target_pose_->orientation.x,
                target_pose_->orientation.y,
                target_pose_->orientation.z
            };
            current_orientation.normalize();
            target_orientation.normalize();
            auto error_orientation = Eigen::AngleAxisd(target_orientation * current_orientation.inverse());
            auto angular_vel = angular_P_ * error_orientation.angle() * error_orientation.axis();

            current_vel_setpoint.twist.linear.x = capped_linear_vel[0];
            current_vel_setpoint.twist.linear.y = capped_linear_vel[1];
            current_vel_setpoint.twist.linear.z = capped_linear_vel[2];
            current_vel_setpoint.twist.angular.x = angular_vel[0];
            current_vel_setpoint.twist.angular.y = angular_vel[1];
            current_vel_setpoint.twist.angular.z = angular_vel[2];
        }
    }
    
    servo_interface_->set_vel_setpoint_(current_vel_setpoint);
}
// pose tracker node
// tracks a pose provided by a topic
// to prepare, need to switch controller from ${non_servo_controller} ${servo_controller} to if required
// to reset before exiting, switch controller from ${servo_controller} ${non_servo_controller} and stop_servo
// need to update topic health status
// start_tracker service : if the topic health status is good, start tracking until reach, after reaching exit and set state to 
// stop_tracker : if

/*
internal methods required  : 
1. switch_controller 
2. start/stop servo
*/

/*
external methods/ services : 
1. prepare_tracker
2. unprepare_tracker
3. start_tracker
4. stop_tracker
*/

/*
parameters required : 
1. PID gains
2. planning group
3. arm_side
4. max_speed
5. servo_controller
6. non_servo_controller
7. servo_namespace (name of the servo node)
9. end_effector_link
*/

/*
The node uses a state machine with three states
NOT_READY 0 // node state when all the prerequisites are not met (not the right controller and start servo), the robot should not move
READY 1 // node state when all the prerequisites are met (the right controller and start servo), the robot should not move
ACTIVE_TRACKING 2 // node state when the node is actively tracking, the robot should move until it reaches and transition to READY state
*/

#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "std_msgs/msg/int16.hpp"
#include "std_msgs/msg/float32.hpp"
#include "motion_planning_abstractions_msgs/srv/pick.hpp"
#include "ur_msgs/srv/set_io.hpp"
#include "open_set_object_detection_msgs/srv/get_object_locations.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rmw/qos_profiles.h"
#include "geometry_msgs/msg/wrench_stamped.hpp"
#include "controller_manager_msgs/srv/switch_controller.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <Eigen/Geometry>

#include "pid_linear_velocity.hpp"
#include "pid_angular_velocity.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/exceptions.hpp"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

#define NOT_READY 0 // node state when all the prerequisites are not met (not the right controller and start servo)
#define READY 1 // node state when all the prerequisites are met (the right controller and start servo)
#define ACTIVE_TRACKING 2 // node state when the node is actively tracking


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class PoseTracker
{
public:
    PoseTracker()
    {
        node_ = std::make_shared<rclcpp::Node>("pose_tracker");

        // declare parameters
        node_->declare_parameter<std::string>("planning_group", "left_ur16e");
        node_->declare_parameter<std::string>("endeffector_link", "left_tool0");
        node_->declare_parameter<std::string>("servo_controller", "left_forward_position_controller");
        node_->declare_parameter<std::string>("non_servo_controller", "left_scaled_joint_trajectory_controller");
        node_->declare_parameter<std::string>("servo_node_namespace", "left_servo_node_main");
        node_->declare_parameter<std::string>("planning_frame", "world");
        node_->declare_parameter<std::string>("servo_frame", "left_base_link");
        
        node_->declare_parameter<bool>("terminate",true); // should the tracker terminate?

        node_->declare_parameter<double>("P_GAIN", 1.0);
        node_->declare_parameter<double>("I_GAIN", 1.0);
        node_->declare_parameter<double>("D_GAIN", 1.0);
        node_->declare_parameter<double>("K_GAIN", 1.0);
        node_->declare_parameter<double>("linear_stop_threshold",0.001); // m
        node_->declare_parameter<double>("angular_stop_threshold",0.01); // rad
        node_->declare_parameter<double>("target_pose_timeout",2.0); // s
        node_->declare_parameter<double>("linear_iir_alpha",0.2); // from [0.0,1.0] more means filter more
        node_->declare_parameter<double>("angular_iir_alpha",0.2); // from [0.0,1.0] more means filter more
        
        // get parameters
        planning_group_ = node_->get_parameter("planning_group").as_string();
        endeffector_link_ = node_->get_parameter("endeffector_link").as_string();
        servo_controller_ = node_->get_parameter("servo_controller").as_string();
        non_servo_controller_ = node_->get_parameter("non_servo_controller").as_string();
        servo_node_namespace_ = node_->get_parameter("servo_node_namespace").as_string();
        planning_frame_ = node_->get_parameter("planning_frame").as_string();

        terminate_ = node_->get_parameter("terminate").as_bool();
        
        P_GAIN_ = node_->get_parameter("P_GAIN").as_double();
        I_GAIN_ = node_->get_parameter("I_GAIN").as_double();
        D_GAIN_ = node_->get_parameter("D_GAIN").as_double();
        K_GAIN_ = node_->get_parameter("K_GAIN").as_double();
        linear_stop_threshold_ = node_->get_parameter("linear_stop_threshold").as_double();
        angular_stop_threshold_ = node_->get_parameter("angular_stop_threshold").as_double();
        target_pose_timeout_ = node_->get_parameter("target_pose_timeout").as_double();
        linear_iir_alpha_ = node_->get_parameter("linear_iir_alpha").as_double();
        angular_iir_alpha_ = node_->get_parameter("angular_iir_alpha").as_double();

        // pid linear and angular interfaces
        pid_linear_velocity_interface_ = PIDLinearVelocity(P_GAIN_, I_GAIN_, D_GAIN_, K_GAIN_, 0.3, 50.0, 10.0);
        pid_angular_velocity_interface_ = PIDAngularVelocity(P_GAIN_, I_GAIN_, D_GAIN_, K_GAIN_, 0.3, 50.0, 10.0);

        // move group interface setup
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node_options.use_global_arguments(false);
        std::string moveit_node_name = std::string(node_->get_name()) + "_moveit";

        moveit_node_ = std::make_shared<rclcpp::Node>(moveit_node_name, node_options);
        move_group_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);

        move_group_interface_->setEndEffectorLink(endeffector_link_);
        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(15);
        move_group_interface_->setMaxVelocityScalingFactor(0.1);
        move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
        move_group_interface_->startStateMonitor();

        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        executor_->add_node(node_);
        moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
        moveit_executor_->add_node(moveit_node_);

        rclcpp::sleep_for(3s);

        // display some mgi shit
        auto planning_frame = this->move_group_interface_->getPlanningFrame();
        RCLCPP_INFO(node_->get_logger(), "Planning frame : %s", planning_frame.c_str());

        auto endeffector = this->move_group_interface_->getEndEffectorLink();
        RCLCPP_INFO(node_->get_logger(), "End Effector Link : %s", endeffector.c_str());

        auto current_pose = this->move_group_interface_->getCurrentPose(endeffector);
        RCLCPP_INFO(node_->get_logger(),
                    "x : %f, y : %f, z : %f",
                    current_pose.pose.position.x,
                    current_pose.pose.position.y,
                    current_pose.pose.position.z);

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // servers
        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",
            std::bind(&PoseTracker::print_state, this,std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );

        prepare_tracker_ = node_->create_service<std_srvs::srv::Trigger>("~/prepare_tracker",
            std::bind(&PoseTracker::prepare_tracker_callback_,this,std::placeholders::_1,std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );

        unprepare_tracker_ = node_->create_service<std_srvs::srv::Trigger>("~/unprepare_tracker",
            std::bind(&PoseTracker::unprepare_tracker_callback_,this,std::placeholders::_1,std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );
        
        start_tracker_ = node_->create_service<std_srvs::srv::Trigger>("~/start_tracker",
            std::bind(&PoseTracker::start_tracker_callback_,this,std::placeholders::_1,std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );

        stop_tracker_ = node_->create_service<std_srvs::srv::Trigger>("~/stop_tracker",
            std::bind(&PoseTracker::stop_tracker_callback_,this,std::placeholders::_1,std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_
        );

        // FOR TESTING STUFF //
        auto test_tracker_ = node_->create_service<std_srvs::srv::Trigger>("~/test_tracker",
            std::bind(&PoseTracker::track_target_pose_,this,std::placeholders::_1,std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_  
        );
        ////
        
        // clients
        switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller",
        rmw_qos_profile_services_default, callback_group_);

        std::string start_servo_service_name = servo_node_namespace_ + "/start_servo";
        start_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(start_servo_service_name,
        rmw_qos_profile_services_default,
        callback_group_);

        std::string stop_servo_service_name = servo_node_namespace_ + "/stop_servo";
        stop_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(stop_servo_service_name,
        rmw_qos_profile_services_default,
        callback_group_);

        if(!switch_controller_client_->wait_for_service(3s))
            RCLCPP_ERROR(node_->get_logger(),"Switch controller service is not connected!");

        if(!start_servo_client_->wait_for_service(3s))
            RCLCPP_ERROR(node_->get_logger(),"Start servo service is not connected!");

        if(!stop_servo_client_->wait_for_service(3s))
            RCLCPP_ERROR(node_->get_logger(),"Stop servo service is not connected!");
        
        // publishers
        std::string delta_twist_cmd_topic = servo_node_namespace_ + "/delta_twist_cmds";
        delta_twist_cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(delta_twist_cmd_topic,10);
        tracker_status_publisher_ = node_->create_publisher<std_msgs::msg::Int16>("~/tracker_status",10);
        linear_error_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("~/linear_error",10);
        angular_error_publisher_ = node_->create_publisher<std_msgs::msg::Float32>("~/angular_error",10);
        
        // subscribers
        target_pose_subscription_ = node_->create_subscription<geometry_msgs::msg::Pose>("~/target_pose",
            10,
            std::bind(&PoseTracker::target_pose_callback_,this,std::placeholders::_1)
        );

        // timers
        control_loop_timer_ = node_->create_wall_timer(20ms,[this](){control_loop_();});
        target_pose_watchdog_timer_ = node_->create_wall_timer(20ms,[this](){target_pose_watchdog_timer_callback_();});
        publish_tracker_status_timer_ = node_->create_wall_timer(100ms,
            [this](){
                auto msg = std_msgs::msg::Int16(); 
                msg.data=current_state_; 
                tracker_status_publisher_->publish(msg);
            }
        );

        thread_ = std::thread([this](){moveit_executor_->spin();});
        executor_->spin();
    }

    bool switch_controller(){
        RCLCPP_INFO(node_->get_logger(),"Switching controller");
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = std::vector<std::string>{servo_controller_};
        request->deactivate_controllers = std::vector<std::string>{non_servo_controller_};
        request->strictness = request->BEST_EFFORT;

        auto future = switch_controller_client_->async_send_request(request);

        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
        }
        else{
            auto resp = future.get();
            if (resp->ok){
                RCLCPP_INFO(node_->get_logger(),"Service successful");
                return true;
            }
            else{
                RCLCPP_ERROR(node_->get_logger(),"Service couldn't swith controller");
                return false;
            }
        }
    }

    bool switch_back_controller(){
        RCLCPP_INFO(node_->get_logger(),"Switching back controller");
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = std::vector<std::string>{non_servo_controller_};
        request->deactivate_controllers = std::vector<std::string>{servo_controller_};
        request->strictness = request->BEST_EFFORT;

        auto future = switch_controller_client_->async_send_request(request);

        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
        }
        else{
            auto resp = future.get();
            if (resp->ok){
                RCLCPP_INFO(node_->get_logger(),"Service successful");
                return true;
            }
            else{
                RCLCPP_ERROR(node_->get_logger(),"Service couldn't swith controller");
                return false;
            }
        }
    }

    bool start_servo(){
        RCLCPP_INFO(node_->get_logger(),"Starting servo service call");
        auto request = std::make_shared<std_srvs::srv::Trigger_Request>();
        auto future = start_servo_client_->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Start servo service call timed out!");
            return false;
        }
        return future.get()->success;
    }

    bool stop_servo(){
        RCLCPP_INFO(node_->get_logger(),"Stopping servo");
        auto request = std::make_shared<std_srvs::srv::Trigger_Request>();
        auto future = stop_servo_client_->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Stop servo service call timed out!");
            return false;
        }
        return future.get()->success;
    }

    void move_to_pose(const geometry_msgs::msg::Pose &pose)
    {
        move_group_interface_->setPoseTarget(pose);
        auto const [success, plan] = [this]
        {
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface_->plan(msg));
            return std::make_pair(ok, msg);
        }();
        if (success)
        {
            move_group_interface_->execute(plan);
        }
        else
        {
            RCLCPP_ERROR(node_->get_logger(), "Planning Failed");
        }
        move_group_interface_->clearPoseTargets();
    }

    void print_state(const std_srvs::srv::Trigger::Request::SharedPtr,std_srvs::srv::Trigger::Response::SharedPtr response){
        auto endeffector = move_group_interface_->getEndEffectorLink();
        auto planning_frame = move_group_interface_->getPlanningFrame();
        auto current_state = move_group_interface_->getCurrentState();
        (void)current_state;
        auto current_pose = move_group_interface_->getCurrentPose();
        auto current_joint_values = move_group_interface_->getCurrentJointValues();
        auto print_pose = [this, endeffector, planning_frame, current_joint_values, current_pose]()
        {
            double x = current_pose.pose.position.x;
            double y = current_pose.pose.position.y;
            double z = current_pose.pose.position.z;
            double qx = current_pose.pose.orientation.x;
            double qy = current_pose.pose.orientation.y;
            double qz = current_pose.pose.orientation.z;
            double qw = current_pose.pose.orientation.w;

            RCLCPP_INFO(this->node_->get_logger(), "Endeffector : %s", endeffector.c_str());
            RCLCPP_INFO(this->node_->get_logger(), "Planning frame : %s", planning_frame.c_str());
            RCLCPP_INFO(this->node_->get_logger(), "X : %f", x);
            RCLCPP_INFO(this->node_->get_logger(), "Y : %f", y);
            RCLCPP_INFO(this->node_->get_logger(), "Z : %f", z);
            RCLCPP_INFO(this->node_->get_logger(), "Qx : %f", qx);
            RCLCPP_INFO(this->node_->get_logger(), "Qy : %f", qy);
            RCLCPP_INFO(this->node_->get_logger(), "Qz : %f", qz);
            RCLCPP_INFO(this->node_->get_logger(), "Qw : %f", qw);

            std::string message;
            for (std::size_t i = 0; i < current_joint_values.size(); i++)
            {
                message += "Joint " + std::to_string(i) + ": " +
                           std::to_string(current_joint_values[i]) + "\n";
            }

            message += "X : " + std::to_string(x) +
                       " Y : " + std::to_string(y) +
                       " Z : " + std::to_string(z);
            return message;
        };

        response->message = print_pose();
        response->success = true;
    }

    void prepare_tracker_callback_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
        RCLCPP_INFO(node_->get_logger(),"Switching controllers to servo type controller");
        if(switch_controller()){
            RCLCPP_INFO(node_->get_logger(),"Done");
            RCLCPP_INFO(node_->get_logger(),"Starting servo");
            if(start_servo()){
                RCLCPP_INFO(node_->get_logger(),"Done");
                response->message = "preparation complete";
                response->success = true;
                current_state_ = READY;
            }
            else{
                RCLCPP_ERROR(node_->get_logger(),"Error starting servo, switching back controller to non servo type");
                if(switch_back_controller())
                    RCLCPP_INFO(node_->get_logger(),"Done");
                else
                    RCLCPP_INFO(node_->get_logger(),"Error switching back controller! manually switch back controllers");
            }
        }
        else
            RCLCPP_ERROR(node_->get_logger(),"Error switching to servo type controller");
    }

    void unprepare_tracker_callback_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
        RCLCPP_INFO(node_->get_logger(),"Switching controllers to non servo type controllers");
        if(switch_back_controller()){
            RCLCPP_INFO(node_->get_logger(),"Done");
            RCLCPP_INFO(node_->get_logger(),"Stopping servo");
            if(stop_servo()){
                RCLCPP_INFO(node_->get_logger(),"Done");
                response->message = "unpreparation complete";
                response->success = true;
                current_state_ = NOT_READY;
            }
            else{
                RCLCPP_ERROR(node_->get_logger(),"Error stopping servo");
            }
        }
        else
            RCLCPP_ERROR(node_->get_logger(),"Error switching to non servo type controller");
    }

    void start_tracker_callback_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
        if(node_->now().seconds() - last_message_time_ > target_pose_timeout_ || target_pose_ == nullptr){
            RCLCPP_ERROR(node_->get_logger(),"The last message not received or timed out");
            response->success = false;
            return;
        }
        if(current_state_==ACTIVE_TRACKING){
            RCLCPP_INFO(node_->get_logger(),"State is already in active tracking");
        }
        if(current_state_==READY){
            current_state_ = ACTIVE_TRACKING;
            RCLCPP_INFO(node_->get_logger(),"State is in active tracking");
        }
        if(current_state_==NOT_READY){
            RCLCPP_INFO(node_->get_logger(),"Not ready to track");
            return;
        }
        response->success = true;
        return;
    }

    void stop_tracker_callback_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
        if(current_state_ == ACTIVE_TRACKING){
            current_state_ = READY; // no condition, just stop the robot
            RCLCPP_INFO(node_->get_logger(),"Stopped actively tracking");
            response->success = true;
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"The node is not actively tracking");
            response->success = true;
        }
    }

    void target_pose_callback_(const geometry_msgs::msg::Pose::SharedPtr msg){
        this->target_pose_ = msg;
        last_message_time_ = node_->now().seconds();
    }

    void target_pose_watchdog_timer_callback_(){
        if(node_->now().seconds() - last_message_time_ > target_pose_timeout_ && current_state_==ACTIVE_TRACKING){
            current_state_ = READY;
            RCLCPP_ERROR(node_->get_logger(),"Target pose timed out");
        }
    }

    void track_target_pose_(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
        /*
            currently using a service to test stuff
            need to add a watchdog timer to the target 
            need to add a low pass filter to output velocity
            for d, need to compute error velocity and need to compute velocity of the orientation error
        */
        
        RCLCPP_INFO(node_->get_logger(),"Started service");
        auto current_pose = this->move_group_interface_->getCurrentPose();
        
        Eigen::Vector3d current_pose_linear = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
        Eigen::Vector3d target_pose_linear = {target_pose_->position.x,target_pose_->position.y,target_pose_->position.z};
        Eigen::Vector3d linear_error_ = target_pose_linear - current_pose_linear;

        Eigen::Quaterniond current_orientation_q = {current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z};
        Eigen::Quaterniond target_orientation_q = {target_pose_->orientation.w, target_pose_->orientation.x, target_pose_->orientation.y, target_pose_->orientation.z};
        
        auto target_orientation_normalized_q = target_orientation_q.normalized();

        // test the PIDLinearVelocity
        auto pid_linear_interface = PIDLinearVelocity(P_GAIN_, I_GAIN_, D_GAIN_, K_GAIN_, 0.3, 50.0, 10.0);
        auto linear_velocity = pid_linear_interface.get_velocity(current_pose_linear, target_pose_linear);
        // test the PIDAngularVelocity
        auto pid_angular_interface = PIDAngularVelocity(P_GAIN_, I_GAIN_, D_GAIN_, K_GAIN_, 0.3, 50.0, 10.0);
        auto angular_velocity = pid_angular_interface.get_velocity(current_orientation_q, target_orientation_q);

    }

    Eigen::Vector3d apply_linear_iir_filter(Eigen::Vector3d linear_velocity){
        filtered_linear_velocity_ = (1-linear_iir_alpha_)*linear_velocity + linear_iir_alpha_*filtered_linear_velocity_;
        return filtered_linear_velocity_;
    }

    Eigen::Vector3d apply_angular_iir_filter(Eigen::Vector3d angular_velocity){
        filtered_angular_velocity_ = (1-angular_iir_alpha_)*angular_velocity.norm() + angular_iir_alpha_*filtered_angular_velocity_;
        if(angular_velocity.norm()<0.001)
            return {0.0,0.0,0.0};
        return (filtered_angular_velocity_/angular_velocity.norm())*angular_velocity;
    }

    void zero_filtered_linear_velocity(){
        filtered_linear_velocity_ = {0.0,0.0,0.0};
    }

    void zero_filtered_angular_velocity(){
        filtered_angular_velocity_ = 0.0;
    }

    void control_loop_(){
        if(current_state_ == ACTIVE_TRACKING and target_pose_ != nullptr){
            auto current_pose = this->move_group_interface_->getCurrentPose();
            
            Eigen::Vector3d current_pose_linear = {current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z};
            Eigen::Vector3d target_pose_linear = {target_pose_->position.x,target_pose_->position.y,target_pose_->position.z};
            Eigen::Vector3d linear_error_ = target_pose_linear - current_pose_linear;
            
            Eigen::Quaterniond current_orientation_q = {current_pose.pose.orientation.w, current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z};
            Eigen::Quaterniond target_orientation_q = {target_pose_->orientation.w, target_pose_->orientation.x, target_pose_->orientation.y, target_pose_->orientation.z};
            
            auto target_orientation_normalized_q = target_orientation_q.normalized();
            auto orientation_error = target_orientation_q*current_orientation_q.inverse();

            if(terminate_ && linear_error_.norm()<linear_stop_threshold_ && std::abs(Eigen::AngleAxisd(orientation_error).angle())<angular_stop_threshold_){
                RCLCPP_INFO(node_->get_logger(),"finished tracking this pose");
                zero_filtered_linear_velocity();
                zero_filtered_angular_velocity();
                current_state_ = READY;
                return;
            }
            
            // get the PIDLinearVelocity
            auto linear_velocity = apply_linear_iir_filter(pid_linear_velocity_interface_.get_velocity(current_pose_linear, target_pose_linear));
            
            // get the PIDAngularVelocity
            auto angular_velocity = apply_angular_iir_filter(pid_angular_velocity_interface_.get_velocity(current_orientation_q, target_orientation_q));

            current_velocity_cmd_.header.frame_id = planning_frame_;
            current_velocity_cmd_.header.stamp = node_->now();
            current_velocity_cmd_.twist.linear.x = linear_velocity.x();
            current_velocity_cmd_.twist.linear.y = linear_velocity.y();
            current_velocity_cmd_.twist.linear.z = linear_velocity.z();
            current_velocity_cmd_.twist.angular.x = angular_velocity.x();
            current_velocity_cmd_.twist.angular.y = angular_velocity.y();
            current_velocity_cmd_.twist.angular.z = angular_velocity.z();
            this->delta_twist_cmd_publisher_->publish(current_velocity_cmd_); // this should be filtered velocity

            // publish the linear an angular errors
            this->linear_error_publisher_->publish([linear_error_]{std_msgs::msg::Float32 msg; msg.data = linear_error_.norm(); return msg;}());
            this->angular_error_publisher_->publish([orientation_error]{std_msgs::msg::Float32 msg; msg.data = Eigen::AngleAxisd(orientation_error).angle(); return msg;}());
        }

        if(current_state_ != ACTIVE_TRACKING){
            auto linear_velocity = apply_linear_iir_filter(Eigen::Vector3d{0.0,0.0,0.0});
            auto angular_velocity = apply_angular_iir_filter(Eigen::Vector3d{0.0,0.0,0.0});

            current_velocity_cmd_.header.frame_id = planning_frame_;
            current_velocity_cmd_.header.stamp = node_->now();
            current_velocity_cmd_.twist.linear.x = linear_velocity.x();
            current_velocity_cmd_.twist.linear.y = linear_velocity.y();
            current_velocity_cmd_.twist.linear.z = linear_velocity.z();
            current_velocity_cmd_.twist.angular.x = angular_velocity.x();
            current_velocity_cmd_.twist.angular.y = angular_velocity.y();
            current_velocity_cmd_.twist.angular.z = angular_velocity.z();
            this->delta_twist_cmd_publisher_->publish(current_velocity_cmd_); // this should be filtered velocity
        }
    }

private:
    std::thread thread_;
    
    
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr moveit_node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_twist_cmd_publisher_;
    rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr tracker_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr linear_error_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr angular_error_publisher_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_tracker_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unprepare_tracker_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_tracker_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_tracker_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscription_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    rclcpp::TimerBase::SharedPtr target_pose_watchdog_timer_;
    rclcpp::TimerBase::SharedPtr publish_tracker_status_timer_;
    
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    PIDLinearVelocity pid_linear_velocity_interface_;
    PIDAngularVelocity pid_angular_velocity_interface_;

    geometry_msgs::msg::Pose::SharedPtr target_pose_ ;
    geometry_msgs::msg::TwistStamped current_velocity_cmd_;

    Eigen::Vector3d filtered_linear_velocity_;
    double filtered_angular_velocity_;

    rclcpp::Clock system_clock_;
    
    // ros parameters
    std::string planning_group_;
    std::string endeffector_link_;
    std::string servo_controller_;
    std::string non_servo_controller_;
    std::string servo_node_namespace_;
    std::string planning_frame_;

    bool terminate_;
    
    double P_GAIN_;
    double I_GAIN_;
    double D_GAIN_;
    double K_GAIN_;
    double max_speed_;
    double linear_stop_threshold_;
    double angular_stop_threshold_;
    double target_pose_timeout_;
    double last_message_time_;
    double linear_iir_alpha_;
    double angular_iir_alpha_;

    uint8_t current_state_=NOT_READY;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto moveit_example = PoseTracker();

    rclcpp::shutdown();
    return 0;
}
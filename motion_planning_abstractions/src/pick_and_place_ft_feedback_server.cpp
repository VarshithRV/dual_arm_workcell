// pick_and_place_local_perception_server.cpp

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
#include "geometry_msgs/msg/point.hpp"
#include "std_srvs/srv/trigger.hpp"
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

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class PickPlace
{
public:
    PickPlace()
    {
        node_ = std::make_shared<rclcpp::Node>("pick_and_place_server");

        node_->declare_parameter<std::string>("planning_group", "right_ur16e");
        node_->declare_parameter<double>("place_x", 0.0);
        node_->declare_parameter<double>("place_y", 0.0);
        node_->declare_parameter<double>("place_z", 0.0);
        node_->declare_parameter<double>("orientation_w", 1.0);
        node_->declare_parameter<double>("orientation_x", 0.0);
        node_->declare_parameter<double>("orientation_y", 0.0);
        node_->declare_parameter<double>("orientation_z", 0.0);
        node_->declare_parameter<double>("pick_offset_x", 0.0);
        node_->declare_parameter<double>("pick_offset_y", 0.0);
        node_->declare_parameter<double>("pick_offset_z", 0.0);
        node_->declare_parameter<double>("look_offset_x", 0.0);
        node_->declare_parameter<double>("look_offset_y", 0.0);
        node_->declare_parameter<double>("look_offset_z", 0.0);
        node_->declare_parameter<double>("place_step_x", 0.05);
        node_->declare_parameter<double>("place_step_y", 0.05);
        node_->declare_parameter<double>("pretouch_distance", 0.1);
        node_->declare_parameter<double>("speed", 0.05);
        node_->declare_parameter<int>("pin_out1", 0);
        node_->declare_parameter<int>("pin_out2", 0);
        node_->declare_parameter<std::string>("arm_side", "left");
        node_->declare_parameter<std::string>("servo_controller", "left_forward_velocity_controller");
        node_->declare_parameter<std::string>("joint_trajectory_controller", "left_scaled_joint_trajectory_controller");
        node_->declare_parameter<double>("height_of_movement", 0.25);
        node_->declare_parameter<std::string>("endeffector_link", "right_tool0");
        node_->declare_parameter<double>("ft_threshold", 0.25);
        
        planning_group_ = node_->get_parameter("planning_group").as_string();
        orientation_.push_back(node_->get_parameter("orientation_w").as_double());
        orientation_.push_back(node_->get_parameter("orientation_x").as_double());
        orientation_.push_back(node_->get_parameter("orientation_y").as_double());
        orientation_.push_back(node_->get_parameter("orientation_z").as_double());

        {
            tf2::Quaternion q(
                orientation_[1], 
                orientation_[2], 
                orientation_[3], 
                orientation_[0]);
            if (q.length2() > 1e-8)
            {
                q.normalize();
            }
            else
            {
                RCLCPP_WARN(node_->get_logger(),
                            "Orientation parameter nearly zero, using identity quaternion");
                q.setRPY(0.0, 0.0, 0.0);
            }
            param_orientation_ = tf2::toMsg(q);
        }

        place_position_.push_back(node_->get_parameter("place_x").as_double());
        place_position_.push_back(node_->get_parameter("place_y").as_double());
        place_position_.push_back(node_->get_parameter("place_z").as_double());
        pick_offset_.push_back(node_->get_parameter("pick_offset_x").as_double());
        pick_offset_.push_back(node_->get_parameter("pick_offset_y").as_double());
        pick_offset_.push_back(node_->get_parameter("pick_offset_z").as_double());
        look_offset_.push_back(node_->get_parameter("look_offset_x").as_double());
        look_offset_.push_back(node_->get_parameter("look_offset_y").as_double());
        look_offset_.push_back(node_->get_parameter("look_offset_z").as_double());
        place_step_x_ = node_->get_parameter("place_step_x").as_double();
        place_step_y_ = node_->get_parameter("place_step_y").as_double();
        pretouch_distance_ = node_->get_parameter("pretouch_distance").as_double();
        speed_ = node_->get_parameter("speed").as_double();
        pin_out1_ = node_->get_parameter("pin_out1").as_int();
        pin_out2_ = node_->get_parameter("pin_out2").as_int();
        ft_threshold_ = node_->get_parameter("ft_threshold").as_double();
        arm_side = node_->get_parameter("arm_side").as_string();
        servo_controller_ = node_->get_parameter("servo_controller").as_string();
        joint_trajectory_controller_ = node_->get_parameter("joint_trajectory_controller").as_string();
        height_of_movement_ = node_->get_parameter("height_of_movement").as_double();
        endeffector_link_ = node_->get_parameter("endeffector_link").as_string();
        system_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);

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

        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/print_robot_state",
            std::bind(&PickPlace::print_state, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        pick_and_place_server_ =
            node_->create_service<motion_planning_abstractions_msgs::srv::Pick>(
                "~/pick_and_place",
                std::bind(&PickPlace::pick_and_place_server, this,
                          std::placeholders::_1, std::placeholders::_2),
                rmw_qos_profile_services_default,
                callback_group_);

        std::string io_service_name = arm_side + "_io_and_status_controller/set_io";
        set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>(io_service_name);

        switch_controller_client_ = node_->create_client<controller_manager_msgs::srv::SwitchController>("controller_manager/switch_controller",
        rmw_qos_profile_services_default, callback_group_);

        std::string zero_ft_service_name = arm_side +"_io_and_status_controller/zero_ftsensor";
        zeroft_client_ = node_->create_client<std_srvs::srv::Trigger>(zero_ft_service_name,
        rmw_qos_profile_services_default, callback_group_);

        std::string start_servo_service_name = arm_side + "_servo_node_main/start_servo";
        start_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(start_servo_service_name,
        rmw_qos_profile_services_default,
        callback_group_);

        std::string stop_servo_service_name = arm_side + "_servo_node_main/stop_servo";
        stop_servo_client_ = node_->create_client<std_srvs::srv::Trigger>(stop_servo_service_name,
        rmw_qos_profile_services_default,
        callback_group_);

        std::string get_object_locations_service_name = arm_side + "_get_object_locations";
        get_object_locations_client_ =
            node_->create_client<open_set_object_detection_msgs::srv::GetObjectLocations>(
                get_object_locations_service_name,
                rmw_qos_profile_services_default,
                callback_group_);

        std::string wrench_topic_name = arm_side + "_force_torque_sensor_broadcaster/wrench";
        wrench_subscription_ = node_->create_subscription<geometry_msgs::msg::WrenchStamped>(wrench_topic_name,10,std::bind(&PickPlace::wrench_callback,this,std::placeholders::_1));

        std::string delta_twist_cmd_topic = arm_side + "_servo_node_main/delta_twist_cmds";
        delta_twist_cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(delta_twist_cmd_topic,10);

        if (!set_io_client_->wait_for_service(3s))
        {
            RCLCPP_ERROR(node_->get_logger(), "Set IO client service is not connected!");
        }

        if (!get_object_locations_client_->wait_for_service(3s))
        {
            RCLCPP_ERROR(node_->get_logger(), "GetObjectLocations service is not connected!");
        }
        
        thread_ = std::thread([this](){moveit_executor_->spin();});
        executor_->spin();
    }


    void gripper_on()
    {
        if (pin_out1_)
        {
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = request->FUN_SET_DIGITAL_OUT;
            request->pin = pin_out1_;
            request->state = request->STATE_ON;
            auto result = set_io_client_->async_send_request(request);
            (void)result;
            RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out1)");
        }
        if (pin_out2_)
        {
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = request->FUN_SET_DIGITAL_OUT;
            request->pin = pin_out2_;
            request->state = request->STATE_ON;
            auto result = set_io_client_->async_send_request(request);
            (void)result;
            RCLCPP_INFO(node_->get_logger(), "Gripper on (pin_out2)");
        }
    }

    void gripper_off()
    {
        if (pin_out1_)
        {
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = request->FUN_SET_DIGITAL_OUT;
            request->pin = pin_out1_;
            request->state = request->STATE_OFF;
            auto result = set_io_client_->async_send_request(request);
            (void)result;
            RCLCPP_INFO(node_->get_logger(), "Gripper off (pin_out1)");
        }
        if (pin_out2_)
        {
            auto request = std::make_shared<ur_msgs::srv::SetIO::Request>();
            request->fun = request->FUN_SET_DIGITAL_OUT;
            request->pin = pin_out2_;
            request->state = request->STATE_OFF;
            auto result = set_io_client_->async_send_request(request);
            (void)result;
            RCLCPP_INFO(node_->get_logger(), "Gripper off (pin_out2)");
        }
    }

    bool switch_controller(){
        RCLCPP_INFO(node_->get_logger(),"Switching controller");
        auto request = std::make_shared<controller_manager_msgs::srv::SwitchController::Request>();
        request->activate_controllers = std::vector<std::string>{servo_controller_};
        request->deactivate_controllers = std::vector<std::string>{joint_trajectory_controller_};
        request->strictness = request->BEST_EFFORT;

        auto future = switch_controller_client_->async_send_request(request);

        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
        }
        else
        {
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
        request->activate_controllers = std::vector<std::string>{joint_trajectory_controller_};
        request->deactivate_controllers = std::vector<std::string>{servo_controller_};
        request->strictness = request->BEST_EFFORT;

        auto future = switch_controller_client_->async_send_request(request);

        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Switch controller service call timed out!");
        }
        else
        {
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

    bool zeroft(){
        RCLCPP_INFO(node_->get_logger(),"Zeroing ft sensor");
        auto request = std::make_shared<std_srvs::srv::Trigger_Request>();
        auto future = zeroft_client_->async_send_request(request);
        if (future.wait_for(10s) != std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(), "Zero Ft service call timed out!");
            return false;
        }
        return future.get()->success;
    }

    void touch_object(){
        // std::this_thread::sleep_for(1s);
        if(!this->switch_controller()){
            RCLCPP_ERROR(node_->get_logger(),"Switching controller failed");
            return;
        }
        else{
            if(!this->start_servo()){
                RCLCPP_ERROR(node_->get_logger(),"Start servo failed, switching back controller");
                if(!this->switch_back_controller())
                    RCLCPP_ERROR(node_->get_logger(),"Switching back controller failed");
            }
            else{
                this->zeroft();
                // loop here looking for ft feedback
                while(wrench_z_<ft_threshold_){
                    geometry_msgs::msg::TwistStamped message;
                    message.header.stamp = system_clock_.now();
                    message.twist.linear.z = speed_;
                    delta_twist_cmd_publisher_->publish(message);
                    std::this_thread::sleep_for(50ms);
                }
                this->stop_servo();
                // std::this_thread::sleep_for(1s);
                this->switch_back_controller();
            }
        }

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

    bool execute_waypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints)
    {
        move_group_interface_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.002;
        const double jump_threshold = 0.0;

        RCLCPP_INFO(node_->get_logger(), "Computing cartesian path");
        double fraction = move_group_interface_->computeCartesianPath(
            waypoints, eef_step, jump_threshold, trajectory);

        if (fraction < 1.0)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "Cartesian path planning failed, fraction: %f", fraction);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "Trajectory created, attempting to execute now");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto result = move_group_interface_->execute(plan);

        if (result != moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path execution failed");
            return false;
        }
        move_group_interface_->setStartStateToCurrentState();
        return true;
    }

    void pick_and_place_server(
        const motion_planning_abstractions_msgs::srv::Pick::Request::SharedPtr request,
        motion_planning_abstractions_msgs::srv::Pick::Response::SharedPtr response)
    {
        geometry_msgs::msg::Pose approx_pick;

        move_group_interface_->setStartStateToCurrentState();
        auto current_pose = this->move_group_interface_->getCurrentPose().pose;


        approx_pick.position = request->object_position;
        approx_pick.orientation = param_orientation_;

        geometry_msgs::msg::Pose place;
        // Grid pattern on place_x, place_y
        place.position.x = place_position_[0] + ((request->index / 2) * place_step_x_);
        bool index_even = ((request->index % 2) == 0);
        place.position.y = place_position_[1] - (index_even ? 0.0 : place_step_y_);
        place.position.z = place_position_[2];
        place.orientation = param_orientation_;

        // Move to "look" pose first
        move_group_interface_->setStartStateToCurrentState();
        geometry_msgs::msg::Pose target =
            move_group_interface_->getCurrentPose(this->endeffector_link_).pose;

        std::vector<geometry_msgs::msg::Pose> waypoints;
        waypoints.push_back(target);

        geometry_msgs::msg::Pose look_pose = approx_pick;
        look_pose.position.x += look_offset_[0];
        look_pose.position.y += look_offset_[1];
        look_pose.position.z += look_offset_[2];
        waypoints.push_back(look_pose);

        if (!execute_waypoints(waypoints))
        {
            response->result = false;
            return;
        }

        geometry_msgs::msg::Pose pick;

        // Perception call from look pose
        auto get_object_locations_request =
            std::make_shared<open_set_object_detection_msgs::srv::GetObjectLocations::Request>();
        get_object_locations_request->prompt.data = request->prompt;
        get_object_locations_request->is_local = true;

        auto future =
            get_object_locations_client_->async_send_request(get_object_locations_request);


        if (future.wait_for(5s) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(), "Local perception timed out!");
            pick = approx_pick;
        }
        else
        {
            auto resp = future.get();
            if (resp->result.object_position.empty())
            {
                RCLCPP_ERROR(node_->get_logger(), "Perception returned no objects!");
                pick = approx_pick;
            }
            else
            {
                pick.position.x = resp->result.object_position[0].pose.pose.position.x;
                pick.position.y = resp->result.object_position[0].pose.pose.position.y;
                pick.position.z = resp->result.object_position[0].pose.pose.position.z;
            }
        }

        // Apply pick offsets + orientation
        pick.position.x += pick_offset_[0];
        pick.position.y += pick_offset_[1];
        pick.position.z += pick_offset_[2] + pretouch_distance_;

        pick.orientation = param_orientation_;

        // Cartesian move to pick (xy first, then down)
        waypoints.clear();

        move_group_interface_->setStartStateToCurrentState();
        current_pose = move_group_interface_->getCurrentPose().pose;
        current_pose.position.x = pick.position.x;
        current_pose.position.y = pick.position.y;

        waypoints.push_back(current_pose);
        waypoints.push_back(pick);

        if (!execute_waypoints(waypoints))
        {
            response->result = false;
            return;
        }

        touch_object();

        std::this_thread::sleep_for(0.5s);
        // Activate gripper
        gripper_on();
        std::this_thread::sleep_for(0.5s);

        // Post-pick → pre-place → place
        waypoints.clear();
        geometry_msgs::msg::Pose post_pick = pick;
        post_pick.position.z = height_of_movement_;
        waypoints.push_back(post_pick);

        if (!execute_waypoints(waypoints))
        {
            response->result = false;
            return;
        }

        waypoints.clear();
        geometry_msgs::msg::Pose pre_place = place;
        pre_place.position.z = height_of_movement_;
        waypoints.push_back(pre_place);
        waypoints.push_back(place);

        if (!execute_waypoints(waypoints))
        {
            response->result = false;
            return;
        }

        // Deactivate gripper
        std::this_thread::sleep_for(0.5s);
        gripper_off();
        std::this_thread::sleep_for(0.5s);

        waypoints.clear();
        waypoints.push_back(pre_place);

        if (!execute_waypoints(waypoints))
        {
            response->result = false;
            return;
        }

        move_group_interface_->setStartStateToCurrentState();

        response->result = true;
    }

    void wrench_callback(const geometry_msgs::msg::WrenchStamped::SharedPtr msg){
        wrench_z_ = msg->wrench.force.z;
    }

    void print_state(
        const std_srvs::srv::Trigger::Request::SharedPtr /*request*/,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        auto current_state = move_group_interface_->getCurrentState();
        (void)current_state;
        auto current_pose = move_group_interface_->getCurrentPose();
        auto current_joint_values = move_group_interface_->getCurrentJointValues();

        auto print_pose = [this, current_joint_values, current_pose]()
        {
            double x = current_pose.pose.position.x;
            double y = current_pose.pose.position.y;
            double z = current_pose.pose.position.z;
            double qx = current_pose.pose.orientation.x;
            double qy = current_pose.pose.orientation.y;
            double qz = current_pose.pose.orientation.z;
            double qw = current_pose.pose.orientation.w;

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

private:
    std::thread thread_;
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    rclcpp::Node::SharedPtr node_;
    rclcpp::Node::SharedPtr moveit_node_;
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
    rclcpp::Service<motion_planning_abstractions_msgs::srv::Pick>::SharedPtr pick_and_place_server_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
    rclcpp::Client<open_set_object_detection_msgs::srv::GetObjectLocations>::SharedPtr get_object_locations_client_;
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr zeroft_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
    rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr wrench_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_twist_cmd_publisher_;
    rclcpp::Clock system_clock_;
    double pretouch_distance_;
    double speed_;
    double wrench_z_;
    std::string arm_side;
    std::string servo_controller_;
    std::string joint_trajectory_controller_;
    std::string planning_group_;
    std::vector<double> orientation_;
    std::vector<double> place_position_;
    std::vector<double> pick_offset_;
    std::vector<double> look_offset_;
    double height_of_movement_;
    std::string endeffector_link_;
    double ft_threshold_;
    double place_step_x_, place_step_y_;
    int pin_out1_, pin_out2_;
    geometry_msgs::msg::Quaternion param_orientation_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto moveit_example = PickPlace();

    rclcpp::shutdown();
    return 0;
}

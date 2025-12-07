// pose tracker node
// tracks a pose provided by a topic
// to prepare, need to switch controller from ${non_servo_controller} ${servo_controller} to if required
// to reset before exiting, switch controller from ${servo_controller} ${non_servo_controller} and stop_servo

/*
internal methods required  : 
1. switch_controller 
2. start/stop servo
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
        node_ = std::make_shared<rclcpp::Node>("pose_tracker");

        // declare parameters
        node_->declare_parameter<std::string>("planning_group", "left_ur16e");
        node_->declare_parameter<std::string>("arm_side", "left");
        node_->declare_parameter<std::string>("endeffector_link", "left_tool0");
        node_->declare_parameter<std::string>("servo_controller", "left_forward_position_controller");
        node_->declare_parameter<std::string>("non_servo_controller", "left_scaled_joint_trajectory_controller");
        node_->declare_parameter<std::string>("servo_node_namespace", "left_servo_node_main");

        node_->declare_parameter<double>("P_GAIN", 1.0);
        node_->declare_parameter<double>("I_GAIN", 1.0);
        node_->declare_parameter<double>("D_GAIN", 1.0);
        node_->declare_parameter<double>("K_GAIN", 1.0);
        node_->declare_parameter<double>("max_speed", 1.0); // in ms-1
        
        // get parameters
        planning_group_ = node_->get_parameter("planning_group").as_string();
        arm_side_ = node_->get_parameter("arm_side").as_string();
        endeffector_link_ = node_->get_parameter("endeffector_link").as_string();
        servo_controller_ = node_->get_parameter("servo_controller").as_string();
        non_servo_controller_ = node_->get_parameter("non_servo_controller").as_string();
        servo_node_namespace_ = node_->get_parameter("servo_node_namespace").as_string();

        P_GAIN_ = node_->get_parameter("P_GAIN").as_double();
        I_GAIN_ = node_->get_parameter("I_GAIN").as_double();
        D_GAIN_ = node_->get_parameter("D_GAIN").as_double();
        K_GAIN_ = node_->get_parameter("K_GAIN").as_double();
        max_speed_ = node_->get_parameter("max_speed").as_double();

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
        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>(
            "~/print_robot_state",
            std::bind(&PickPlace::print_state, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        // prepare_tracker_ = node_->create_service<>
        
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

        // publishers
        std::string delta_twist_cmd_topic = servo_node_namespace_ + "/delta_twist_cmds";
        delta_twist_cmd_publisher_ = node_->create_publisher<geometry_msgs::msg::TwistStamped>(delta_twist_cmd_topic,10);
        
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
    rclcpp::Client<controller_manager_msgs::srv::SwitchController>::SharedPtr switch_controller_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr start_servo_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr stop_servo_client_;
    rclcpp::Publisher<geometry_msgs::msg::TwistStamped>::SharedPtr delta_twist_cmd_publisher_;
    
    rclcpp::Clock system_clock_;
    
    // ros parameters
    std::string planning_group_;
    std::string arm_side_;
    std::string endeffector_link_;
    std::string servo_controller_;
    std::string non_servo_controller_;
    std::string servo_node_namespace_;
    
    double P_GAIN_;
    double I_GAIN_;
    double D_GAIN_;
    double K_GAIN_;
    double max_speed_;

};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto moveit_example = PickPlace();

    rclcpp::shutdown();
    return 0;
}

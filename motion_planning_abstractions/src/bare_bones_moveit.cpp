// task space cubic polnomial traj server

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
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rmw/qos_profiles.h"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class TSCubicPolynomialTraj
{
public:
    TSCubicPolynomialTraj()
    {
        node_ = std::make_shared<rclcpp::Node>("ts_cubic_polnomial_traj_server");

        // parameter declaration
        node_->declare_parameter<std::string>("planning_group", "left_ur16e");
        node_->declare_parameter<double>("maximum_task_space_velocity",1.0); // in ms-1
        node_->declare_parameter<double>("maximum_task_space_acceleration",3.0); // in ms-2
        node_->declare_parameter<double>("maximum_joint_space_velocity",M_PI); // in rads-1
        node_->declare_parameter<double>("maximum_joint_space_acceleration",M_PI); // in rads-2
        node_->declare_parameter<std::string>("arm_side", "left");
        node_->declare_parameter<std::string>("joint_trajectory_controller", "left_scaled_joint_trajectory_controller");
        node_->declare_parameter<std::string>("endeffector_link", "right_tool0");
        
        // parameter assignment
        planning_group_ = node_->get_parameter("planning_group").as_string();
        maximum_task_space_velocity_ = node_->get_parameter("maximum_task_space_velocity").as_double();
        maximum_task_space_acceleration_ = node_->get_parameter("maximum_task_space_acceleration").as_double();
        maximum_joint_space_velocity_ = node_->get_parameter("maximum_joint_space_velocity").as_double();
        maximum_joint_space_acceleration_ = node_->get_parameter("maximum_joint_space_acceleration").as_double();
        arm_side = node_->get_parameter("arm_side").as_string();
        joint_trajectory_controller_ = node_->get_parameter("joint_trajectory_controller").as_string();
        endeffector_link_ = node_->get_parameter("endeffector_link").as_string();

        system_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
        
        // moveit node shit
        rclcpp::NodeOptions node_options;
        node_options.automatically_declare_parameters_from_overrides(true);
        node_options.use_global_arguments(false);
        std::string moveit_node_name = std::string(node_->get_name()) + "_moveit";
        moveit_node_ = std::make_shared<rclcpp::Node>(moveit_node_name, node_options);
        
        // move group interface shit
        move_group_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);
        move_group_interface_->setEndEffectorLink(endeffector_link_);
        move_group_interface_->setPlanningTime(10.0);
        move_group_interface_->setNumPlanningAttempts(15);
        move_group_interface_->setMaxVelocityScalingFactor(0.1);
        move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
        move_group_interface_->startStateMonitor();

        // more node shit
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
        RCLCPP_INFO(node_->get_logger(),"x : %f, y : %f, z : %f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",std::bind(&TSCubicPolynomialTraj::print_state, this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_);

        thread_ = std::thread([this](){moveit_executor_->spin();});
        executor_->spin();
    }

    void move_to_pose(const geometry_msgs::msg::Pose &pose){
        move_group_interface_->setPoseTarget(pose);
        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok = static_cast<bool>(this->move_group_interface_->plan(msg));
            return std::make_pair(ok, msg);
        }();

        if(success){
            move_group_interface_->execute(plan);
        }
        else{
            RCLCPP_ERROR(node_->get_logger(), "Planning Failed");
        }
        move_group_interface_->clearPoseTargets();
    }

    bool execute_waypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints){
        move_group_interface_->setStartStateToCurrentState();
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.002;
        const double jump_threshold = 0.0;

        RCLCPP_INFO(node_->get_logger(), "Computing cartesian path");
        double fraction = move_group_interface_->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if(fraction < 1.0){
            RCLCPP_ERROR(node_->get_logger(),"Cartesian path planning failed, fraction: %f", fraction);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),"Trajectory created, attempting to execute now");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto result = move_group_interface_->execute(plan);
        if(result != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path execution failed");
            return false;
        }
        move_group_interface_->setStartStateToCurrentState();
        return true;
    }

    void print_state(const std_srvs::srv::Trigger::Request::SharedPtr request,std_srvs::srv::Trigger::Response::SharedPtr response){
        auto current_state = move_group_interface_->getCurrentState();
        (void)current_state;
        auto current_pose = move_group_interface_->getCurrentPose();
        auto current_joint_values = move_group_interface_->getCurrentJointValues();

        auto print_pose = [this, current_joint_values, current_pose](){
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
            for (std::size_t i = 0; i < current_joint_values.size(); i++){
                message += "Joint " + std::to_string(i) + ": " +std::to_string(current_joint_values[i]) + "\n";
            }
            message += "X : " + std::to_string(x) +" Y : " + std::to_string(y) +" Z : " + std::to_string(z);
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
    rclcpp::Clock system_clock_;
    
    std::string arm_side;
    std::string joint_trajectory_controller_;
    std::string planning_group_;
    std::string endeffector_link_;
    double maximum_task_space_velocity_;
    double maximum_task_space_acceleration_;
    double maximum_joint_space_velocity_;
    double maximum_joint_space_acceleration_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);

    auto moveit_example = TSCubicPolynomialTraj();

    rclcpp::shutdown();
    return 0;
}

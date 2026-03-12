// task space cubic polnomial traj server

#include <memory>
#include <string>
#include <chrono>
#include <vector>
#include <cmath>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"

#include "geometry_msgs/msg/pose.hpp"

#include "std_srvs/srv/trigger.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"

#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class BareBonesMoveit{
public:

    BareBonesMoveit()
    {
        node_ = std::make_shared<rclcpp::Node>("bare_bones_moveit");

        node_->declare_parameter<std::string>("planning_group", "right_ur16e");
        node_->declare_parameter<std::string>("arm_side", "right");
        node_->declare_parameter<std::string>("endeffector_link", "right_tool0");
        
        planning_group_ = node_->get_parameter("planning_group").as_string();
        arm_side_ = node_->get_parameter("arm_side").as_string();
        endeffector_link_ = node_->get_parameter("endeffector_link").as_string();

        system_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
        
        // move group interface shit
        move_group_interface_ = std::make_shared<MoveGroupInterface>(node_, planning_group_);
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

        rclcpp::sleep_for(1s);

        // robot model stuff for fk,ik and jacobian
        robot_model_loader::RobotModelLoader robot_model_loader(node_);
        kinematic_model_ = robot_model_loader.getModel();
        RCLCPP_INFO(node_->get_logger(),"Kinematic model loaded,model frame : %s",kinematic_model_->getModelFrame().c_str());
        
        current_robot_state_=std::make_shared<moveit::core::RobotState>(kinematic_model_);
        current_robot_state_->setToDefaultValues();

        joint_group_model_ = kinematic_model_->getJointModelGroup(planning_group_);
        const std::vector<std::string>& joint_names = joint_group_model_->getVariableNames();
        std::vector<double> joint_values;
        current_robot_state_->copyJointGroupPositions(joint_group_model_,joint_values);

        // servers
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",std::bind(&BareBonesMoveit::print_state, this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_);
        test_server_ = node_->create_service<std_srvs::srv::Trigger>("~/test_server",
            [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                res->success = test_server_callback_();
                return;
            }
        );

        // publisher
        sample_publisher_ = node_->create_publisher<geometry_msgs::msg::Pose>("~/sample_topic",10);

        // service clients
        sample_client_ = node_->create_client<std_srvs::srv::Trigger>("/sample_service");

        // timers
        sample_timer_ = node_->create_wall_timer(200ms,
            [this](){
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
        );

        executor_->spin();
    }

    // TEST SERVER CALLBACK HERE
    bool test_server_callback_(){
        auto eepose = move_group_interface_->getCurrentPose();
        do_ik(eepose.pose);
        do_fk(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
        move_to_joint_positions(std::vector<double>{
            0.7448494020060257,
            -1.6898253014238567,
            -2.456773372167499,
            -0.6417658203742237,
            -1.5154170572868646,
            2.4306718044994686
        });
        return true;
    }

    void move_to_joint_positions(const std::vector<double>joint_positions){
        move_group_interface_->setStartStateToCurrentState();
        move_group_interface_->setJointValueTarget(joint_positions);
        auto const [success, plan] = [this]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok=static_cast<bool>(this->move_group_interface_->plan(msg));
            return std::make_pair(ok,msg);
        }();
        if(success){
            move_group_interface_->execute(plan);
        }
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

    // EXECUTE WAYPOINTS AND MOVE TO POSE NEED TO BE INVESTIGATED
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

    void do_fk(std::vector<double> joint_state){
        robot_model_loader::RobotModelLoader robot_model_loader(node_);
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
        RCLCPP_INFO(node_->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
        robot_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(this->planning_group_);
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        robot_state->copyJointGroupPositions(joint_model_group, joint_state);
        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_state[i]);
        }
        robot_state->setToRandomPositions(joint_model_group);
        const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(arm_side_ + "_tool0");
        /* Print end-effector pose. Remember that this is in the model frame */
        RCLCPP_INFO_STREAM(node_->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
        RCLCPP_INFO_STREAM(node_->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
    }

    void do_ik(const geometry_msgs::msg::Pose& eepose){
        Eigen::Isometry3d ee_state;
        ee_state.translation().x() = eepose.position.x;
        ee_state.translation().y() = eepose.position.y;
        ee_state.translation().z() = eepose.position.z;
        ee_state.rotate(Eigen::Quaterniond(eepose.orientation.w,eepose.orientation.x,eepose.orientation.y,eepose.orientation.z));
        bool found_ik = current_robot_state_->setFromIK(joint_group_model_,ee_state,0.1);
        std::vector<double> joint_values;
        std::vector<std::string> joint_names = joint_group_model_->getVariableNames();
        if(found_ik){
            current_robot_state_->copyJointGroupPositions(joint_group_model_, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
              RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
        }
        else
            RCLCPP_INFO(node_->get_logger(), "Did not find IK solution");

        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        current_robot_state_->getJacobian(joint_group_model_,
                                     current_robot_state_->getLinkModel(joint_group_model_->getLinkModelNames().back()),
                                     reference_point_position, jacobian);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Jacobian: \n" << jacobian << "\n");
    }

private:
    std::thread thread_;
    
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    rclcpp::Clock system_clock_;

    // moveit stuff
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr current_robot_state_;
    const moveit::core::JointModelGroup* joint_group_model_;

    // servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_server_;

    // clients

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr sample_publisher_;

    // subscribers
    
    // clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr sample_client_;

    // timers
    rclcpp::TimerBase::SharedPtr sample_timer_;

    // parameters and data
    std::string arm_side_;
    std::string planning_group_;
    std::string endeffector_link_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto moveit_example = BareBonesMoveit();
    rclcpp::shutdown();
    return 0;
}
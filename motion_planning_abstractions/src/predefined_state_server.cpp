// predefined state server node

#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>

#include "moveit/move_group_interface/move_group_interface.h"
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "moveit/robot_model_loader/robot_model_loader.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_model/robot_model.h"
#include "moveit/robot_state/robot_state.h"

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class PredefinedStateServer{
    public: 
        PredefinedStateServer(){
            node_ = std::make_shared<rclcpp::Node>("predefined_state_server");
            
            node_->declare_parameter<std::string>("planning_group", "left_ur16e");
            node_->declare_parameter<double>("shoulder_pan", 0.0);
            node_->declare_parameter<double>("shoulder_lift", 0.0);
            node_->declare_parameter<double>("elbow", 0.0);
            node_->declare_parameter<double>("wrist_1", 0.0);
            node_->declare_parameter<double>("wrist_2", 0.0);
            node_->declare_parameter<double>("wrist_3", 0.0);
            node_->declare_parameter<std::string>("side", "left");
            
            planning_group_ = node_->get_parameter("planning_group").as_string();
            joint_targets_.push_back(node_->get_parameter("shoulder_pan").as_double());
            joint_targets_.push_back(node_->get_parameter("shoulder_lift").as_double());
            joint_targets_.push_back(node_->get_parameter("elbow").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_1").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_2").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_3").as_double());
            side_=node_->get_parameter("side").as_string();
            
            
            rclcpp::NodeOptions opts;
            opts.automatically_declare_parameters_from_overrides(true);
            opts.use_global_arguments(false);
            std::string moveit_node_name = std::string(node_->get_name()) + "_moveit";
            moveit_node_ = std::make_shared<rclcpp::Node>(moveit_node_name, opts);

            move_group_interface_ = std::make_shared<MoveGroupInterface>(moveit_node_, planning_group_);
            
            move_group_interface_->startStateMonitor();
            
            executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
            executor_->add_node(node_);
            moveit_executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
            moveit_executor_->add_node(moveit_node_);

            print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",std::bind(&PredefinedStateServer::print_state, this, std::placeholders::_1, std::placeholders::_2));
            move_to_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/move_to_state",std::bind(&PredefinedStateServer::move_to_joint_state, this, std::placeholders::_1, std::placeholders::_2));
            fk_server_ = node_->create_service<std_srvs::srv::Trigger>("~/fk",std::bind(&PredefinedStateServer::forward_kinematics, this, std::placeholders::_1, std::placeholders::_2));
            
            RCLCPP_INFO(node_->get_logger(),"Started the tutorials node");
            
            thread_ = std::thread([this](){moveit_executor_->spin();});
            executor_->spin();
        }

        // pose setpoint movement
        void move_to_pose(const geometry_msgs::msg::Pose &pose){
            move_group_interface_->setPoseTarget(pose);
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok=static_cast<bool>(this->move_group_interface_->plan(msg));
                return std::make_pair(ok,msg);
            }();
            if(success)
                move_group_interface_->execute(plan);
            else
                RCLCPP_ERROR(node_->get_logger(),"Planning Failed");
            move_group_interface_->clearPoseTargets();
        }

        // joint state setpoint movement
        void move_to_joint_state(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
            std::vector<double> group_variable_values = joint_targets_;
            move_group_interface_->setStartStateToCurrentState();
            move_group_interface_->setJointValueTarget(group_variable_values);
            auto const [success, plan] = [this]{
                moveit::planning_interface::MoveGroupInterface::Plan msg;
                auto const ok=static_cast<bool>(this->move_group_interface_->plan(msg));
                return std::make_pair(ok,msg);
            }();
            if(success){
                move_group_interface_->execute(plan);
                RCLCPP_INFO(node_->get_logger(),"Finished execution");
                response->success = true;
            }
            else
                RCLCPP_ERROR(node_->get_logger(),"Planning Failed");
        }

        // some basic forward kinematics
        void forward_kinematics(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){
            robot_model_loader::RobotModelLoader robot_model_loader(moveit_node_);
            const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
            RCLCPP_INFO(node_->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
            moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
            robot_state->setToDefaultValues();
            const moveit::core::JointModelGroup* joint_model_group = kinematic_model->getJointModelGroup(this->planning_group_);
            const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
            std::vector<double> joint_values;
            robot_state->copyJointGroupPositions(joint_model_group, joint_values);
            for (std::size_t i = 0; i < joint_names.size(); ++i)
            {
              RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
            }
            robot_state->setToRandomPositions(joint_model_group);
            const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(side_ + "_tool0");

            /* Print end-effector pose. Remember that this is in the model frame */
            RCLCPP_INFO_STREAM(node_->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
            RCLCPP_INFO_STREAM(node_->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
        }

        void print_state(const std_srvs::srv::Trigger_Request::SharedPtr request, std_srvs::srv::Trigger_Response::SharedPtr response){ // not working, stupid timer issue
            auto current_state = move_group_interface_->getCurrentState();
            auto current_pose = move_group_interface_->getCurrentPose();
            auto current_joint_values = move_group_interface_->getCurrentJointValues();
            auto print_pose = [this,current_state, current_joint_values, current_pose](){
                double x = current_pose.pose.position.x;
                double y = current_pose.pose.position.y;
                double z = current_pose.pose.position.z;
                double qx = current_pose.pose.orientation.x;
                double qy = current_pose.pose.orientation.y;
                double qz = current_pose.pose.orientation.z;
                double qw = current_pose.pose.orientation.w;
                RCLCPP_INFO(this->node_->get_logger(),"X : %f",x);
                RCLCPP_INFO(this->node_->get_logger(),"Y : %f",y);
                RCLCPP_INFO(this->node_->get_logger(),"Z : %f",z);
                RCLCPP_INFO(this->node_->get_logger(),"Qx : %f",qx);
                RCLCPP_INFO(this->node_->get_logger(),"Qy : %f",qy);
                RCLCPP_INFO(this->node_->get_logger(),"Qz : %f",qz);
                RCLCPP_INFO(this->node_->get_logger(),"Qw : %f",qw);
                std::string message;
                for(std::size_t i=0; i<current_joint_values.size(); i++)
                    message = "Joint " + std::to_string(i) + ": " + std::to_string(current_joint_values[i]);
                message += "     X : " + std::to_string(x) + " Y : " + std::to_string(y) + " Z : " + std::to_string(z);
                return message;
            };
            response->message = print_pose();
            response->success = true;
        }

    private:
        std::shared_ptr<MoveGroupInterface> move_group_interface_;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Executor::SharedPtr executor_;
        rclcpp::Node::SharedPtr moveit_node_;
        rclcpp::Executor::SharedPtr moveit_executor_;
        std::thread thread_;

        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr move_to_state_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr fk_server_;
        std::string planning_group_;
        std::vector<double> joint_targets_;
        std::string side_;
};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    auto moveit_example = PredefinedStateServer();
}
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
#include "example_interfaces/srv/trigger.hpp"

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
            
            planning_group_ = node_->get_parameter("planning_group").as_string();
            joint_targets_.push_back(node_->get_parameter("shoulder_pan").as_double());
            joint_targets_.push_back(node_->get_parameter("shoulder_lift").as_double());
            joint_targets_.push_back(node_->get_parameter("elbow").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_1").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_2").as_double());
            joint_targets_.push_back(node_->get_parameter("wrist_3").as_double());
            
            
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

            print_state_server_ = node_->create_service<example_interfaces::srv::Trigger>("~/print_robot_state",std::bind(&PredefinedStateServer::print_state, this, std::placeholders::_1, std::placeholders::_2));
            move_to_state_server_ = node_->create_service<example_interfaces::srv::Trigger>("~/move_to_state",std::bind(&PredefinedStateServer::move_to_joint_state, this, std::placeholders::_1, std::placeholders::_2));
            
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
        void move_to_joint_state(const example_interfaces::srv::Trigger_Request::SharedPtr request, example_interfaces::srv::Trigger_Response::SharedPtr response){
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

        void print_state(const example_interfaces::srv::Trigger_Request::SharedPtr request, example_interfaces::srv::Trigger_Response::SharedPtr response){ // not working, stupid timer issue
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

        rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr print_state_server_;
        rclcpp::Service<example_interfaces::srv::Trigger>::SharedPtr move_to_state_server_;
        std::string planning_group_;
        std::vector<double> joint_targets_;
};

int main(int argc, char* argv[]){

    rclcpp::init(argc,argv);
    auto moveit_example = PredefinedStateServer();
}
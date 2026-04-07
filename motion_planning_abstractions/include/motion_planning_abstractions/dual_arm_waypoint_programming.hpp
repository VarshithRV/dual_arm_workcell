// dual arm control client for demos

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
#include "ur_msgs/srv/set_io.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "motion_planning_abstractions_msgs/srv/generate_trajectory.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;

class DualArmControlInterface{
public:
    std::shared_ptr<MoveGroupInterface> left_move_group_interface_;
    std::shared_ptr<MoveGroupInterface> right_move_group_interface_;
    
    DualArmControlInterface(rclcpp::Node::SharedPtr node)
    {

        node_ = node;
        left_planning_group_="left_ur16e";
        right_planning_group_="right_ur16e";
        left_endeffector_link_="left_tool0";
        right_endeffector_link_="right_tool0";

        system_clock_ = rclcpp::Clock(RCL_SYSTEM_TIME);
        
        // move group interface shit
        left_move_group_interface_ = std::make_shared<MoveGroupInterface>(node_, left_planning_group_);
        left_move_group_interface_->setEndEffectorLink(left_endeffector_link_);
        left_move_group_interface_->setPlanningTime(10.0);
        left_move_group_interface_->setNumPlanningAttempts(15);
        left_move_group_interface_->setMaxVelocityScalingFactor(0.1);
        left_move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        left_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
        left_move_group_interface_->startStateMonitor();
        right_move_group_interface_ = std::make_shared<MoveGroupInterface>(node_, right_planning_group_);
        right_move_group_interface_->setEndEffectorLink(right_endeffector_link_);
        right_move_group_interface_->setPlanningTime(10.0);
        right_move_group_interface_->setNumPlanningAttempts(15);
        right_move_group_interface_->setMaxVelocityScalingFactor(0.1);
        right_move_group_interface_->setMaxAccelerationScalingFactor(0.1);
        right_move_group_interface_->setPlannerId("RRTConnectkConfigDefault");
        right_move_group_interface_->startStateMonitor();

        rclcpp::sleep_for(1s);

        // robot model stuff for fk,ik and jacobian
        robot_model_loader::RobotModelLoader robot_model_loader(node_);
        kinematic_model_ = robot_model_loader.getModel();
        RCLCPP_INFO(node_->get_logger(),"Kinematic model loaded,model frame : %s",kinematic_model_->getModelFrame().c_str());
        
        current_robot_state_=std::make_shared<moveit::core::RobotState>(kinematic_model_);
        current_robot_state_->setToDefaultValues();

        left_joint_group_model_ = kinematic_model_->getJointModelGroup(left_planning_group_);
        right_joint_group_model_ = kinematic_model_->getJointModelGroup(right_planning_group_);

        const std::vector<std::string>& left_joint_names = left_joint_group_model_->getVariableNames();
        const std::vector<std::string>& right_joint_names = right_joint_group_model_->getVariableNames();

        std::vector<double> left_joint_values;
        current_robot_state_->copyJointGroupPositions(left_joint_group_model_,left_joint_values);

        std::vector<double> right_joint_values;
        current_robot_state_->copyJointGroupPositions(right_joint_group_model_,right_joint_values);

        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        // servers
        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",std::bind(&DualArmControlInterface::print_state, this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_);

        // service clients
        left_execute_trajectory_client_ = node_->create_client<std_srvs::srv::Trigger>("/left_task_space_cubic_polynomial_trajectory_server/execute_trajectory");
        right_execute_trajectory_client_ = node_->create_client<std_srvs::srv::Trigger>("/right_task_space_cubic_polynomial_trajectory_server/execute_trajectory");
        
        left_generate_trajectory_client_ = node_->create_client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>("/left_task_space_cubic_polynomial_trajectory_server/generate_trajectory");
        right_generate_trajectory_client_ = node_->create_client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>("/right_task_space_cubic_polynomial_trajectory_server/generate_trajectory");

        left_set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>("left_io_and_status_controller/set_io");
        right_set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>("right_io_and_status_controller/set_io");
        
        if(!left_execute_trajectory_client_->wait_for_service(5s)){
            RCLCPP_ERROR(node_->get_logger(),"No /left_task_space_cubic_polynomial_trajectory_server/execute_trajectory, timed out while waiting");
        }
        if(!right_execute_trajectory_client_->wait_for_service(5s)){
            RCLCPP_ERROR(node_->get_logger(),"No /right_task_space_cubic_polynomial_trajectory_server/execute_trajectory, timed out while waiting");
        }
        if(!left_generate_trajectory_client_->wait_for_service(5s)){
            RCLCPP_ERROR(node_->get_logger(),"No /left_task_space_cubic_polynomial_trajectory_server/generate_trajectory, timed out while waiting");
        }
        if(!right_generate_trajectory_client_->wait_for_service(5s)){
            RCLCPP_ERROR(node_->get_logger(),"No /right_task_space_cubic_polynomial_trajectory_server/generate_trajectory, timed out while waiting");
        }
    }

    // // driver SERVER CALLBACK example
    // bool driver_server_callback_(){
    //     auto LOGGER = node_->get_logger();
    //     auto waypoints = Waypoints(); // this is where all the waypoints are
        
    //     auto current_left_pose = get_current_ee_pose("left");
    //     std::vector<geometry_msgs::msg::Pose>left_waypoints{
    //         *current_left_pose,
    //         [current_left_pose](){
    //             auto pose = *current_left_pose;
    //             pose.position.x += 0.1;
    //             return pose;
    //         }()
    //     };

    //     // // asynchronous calling of cubic trajectory example
    //     // RCLCPP_INFO(node_->get_logger(),"Started the left cubic server asyncly");
    //     // auto exec_future = async_start_execute_waypoints_cubic(left_waypoints,std::vector<double>{0,0.5},0.3,0.01,"left");
    //     // block_till_response_execute_cubic_trajectory(exec_future, 5000ms);

    //     RCLCPP_INFO(node_->get_logger(), "Starting left and right joint state motions in parallel");
        
    //     move_to_joint_positions(waypoints.right_rest_state.joint_values,right_move_group_interface_);
        
    //     //waypoint to the cone
    //     move_to_joint_positions(waypoints.right_wp1.joint_values,right_move_group_interface_);
    //     std::vector<geometry_msgs::msg::Pose> to_the_cone{
    //         waypoints.right_wp2.pose,
    //         waypoints.right_wp3.pose,
    //     };
    //     execute_waypoints_cubic(to_the_cone,std::vector<double>{0.75,0.5},0.3,0.07,"right");
        
    //     // twist
    //     std::vector<geometry_msgs::msg::Pose> twist{
    //         waypoints.right_wp4.pose,
    //     };
    //     execute_waypoints_cubic(twist,std::vector<double>{0.5},0.3,0.0,"right");

    //     // get down
    //     std::vector<geometry_msgs::msg::Pose> down{
    //         waypoints.right_wp5.pose,
    //     };
    //     execute_waypoints_cubic(down,std::vector<double>{0.7},0.3,0.05,"right");

    //     // backoff
    //     std::vector<geometry_msgs::msg::Pose> back_off{
    //         waypoints.right_wp6.pose,
    //         waypoints.right_wp7.pose,
    //         waypoints.right_wp8.pose,
    //     };
    //     execute_waypoints_cubic(back_off,std::vector<double>{1.0,1.0,0.7},0.3,0.05,"right");
        
    //     return true;
    // }
    
    // use this to start the execute trajectory asynchronously, note that at a time, asyncly execute trajectories for different arms, do not do it for the same arm, unexpected behavior
    std::shared_ptr<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose> waypoints, 
        std::vector<double> durations, double average_speed, 
        double waypoint_speed, std::string side){
            auto goal = std::make_shared<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request>();
            goal->average_speed = average_speed;
            goal->durations = durations;
            goal->waypoint_speed = waypoint_speed;
            goal->waypoints = waypoints;

            std::shared_ptr<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Response> gen_response;

            if(side =="left"){
                auto future = left_generate_trajectory_client_->async_send_request(goal);
                if(future.wait_for(2s)!=std::future_status::ready){
                    RCLCPP_ERROR(node_->get_logger(),"Generate left trajectory service call timed out!");
                    return nullptr;
                }
                gen_response = future.get();    
            }
            else if(side == "right"){
                auto future = right_generate_trajectory_client_->async_send_request(goal);
                if(future.wait_for(2s)!=std::future_status::ready){
                    RCLCPP_ERROR(node_->get_logger(),"Generate right trajectory service call timed out!");
                    return nullptr;
                }
                gen_response = future.get();    
            }
            else{
                RCLCPP_INFO(node_->get_logger(),"The variable \"side\" is wrong");
                return nullptr;
            }

            if(gen_response->success==true && gen_response->fraction == 100)
                RCLCPP_INFO(node_->get_logger(),"Generate trajectory succeeded, message : %s", gen_response->message.c_str());
            else{
                RCLCPP_INFO(node_->get_logger(),"Generate trajectory failed, message : %s", gen_response->message.c_str());
                return nullptr;
            }
            
            RCLCPP_INFO(node_->get_logger(),"Starting the trajectory execution");

            auto exec_future = std::make_shared<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>>();

            // start the execution of the trajectory
            if(side=="left"){
                std::shared_ptr<std_srvs::srv::Trigger::Request> exec_req=  std::make_shared<std_srvs::srv::Trigger::Request>();
                *exec_future = left_execute_trajectory_client_->async_send_request(exec_req);
            }
            else{
                std::shared_ptr<std_srvs::srv::Trigger::Request> exec_req=  std::make_shared<std_srvs::srv::Trigger::Request>();
                *exec_future = right_execute_trajectory_client_->async_send_request(exec_req);
            }

            return exec_future;
    }

    // use this to block till execute cubic trajectory returns a response
    template<typename Rep, typename Period>
    std_srvs::srv::Trigger::Response::SharedPtr block_till_response_execute_cubic_trajectory(
        std::shared_ptr<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> exec_future, 
        std::chrono::duration<Rep,Period> wait_duration
    ){
        if(exec_future==nullptr){
            RCLCPP_ERROR(node_->get_logger(),"Nullptr received for future, looks like the service didn't execute well");
            return nullptr;
        }
        if(exec_future->wait_for(wait_duration)!=std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(),"execute cubic trajectory timed out");
            return nullptr;
        }
        else{
            auto response = exec_future->get();
            return response;
        }
    }

    // easy function to get the current pose of the end effector
    geometry_msgs::msg::Pose::SharedPtr get_current_ee_pose(std::string side){
        if(side =="left"){
            return std::make_shared<geometry_msgs::msg::Pose>(left_move_group_interface_->getCurrentPose().pose);
        }
        else if(side =="right"){
            return std::make_shared<geometry_msgs::msg::Pose>(right_move_group_interface_->getCurrentPose().pose);
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"The variable \"side\" is wrong");
            return nullptr;
        }
    }

    // ease function to get the current joint states of robot
    std::vector<double> get_current_joint_state(std::string side){
        if(side =="left")
            return left_move_group_interface_->getCurrentJointValues();
        else if(side =="right")
            return right_move_group_interface_->getCurrentJointValues();
    }

    // generate and synchronous execution of cubic trajectory using waypoints, durations/avg_velocity, waypoint_speeds, side
    bool execute_waypoints_cubic(std::vector<geometry_msgs::msg::Pose> waypoints, std::vector<double> durations, double average_speed, double waypoint_speed, std::string side){
        auto req = std::make_shared<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request>();
        req->durations = durations;
        req->waypoints = waypoints;
        req->average_speed = average_speed;
        req->waypoint_speed = waypoint_speed;

        if(side=="left"){
            auto gen_traj_future = left_generate_trajectory_client_->async_send_request(req);
            if(gen_traj_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"Waited for 5s, no traj generated");
                return false;
            }
            auto gen_res = gen_traj_future.get();
            if(!gen_res->success){
                RCLCPP_ERROR(node_->get_logger(),"Traj generation failed %s",gen_res->message.c_str());
                return false;
            }
            RCLCPP_INFO(node_->get_logger(),"Traj generated, now executing");
            auto exec_traj_future = left_execute_trajectory_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            if(exec_traj_future.wait_for(10s) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"Waited for 10s, no traj executed");
                return false;
            }
            auto exec_res = exec_traj_future.get();
            if(!exec_res->success){
                RCLCPP_ERROR(node_->get_logger(),"Traj exec failed");
                return false;
            }
        }
        else if(side=="right"){
            auto gen_traj_future = right_generate_trajectory_client_->async_send_request(req);
            if(gen_traj_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"Waited for 5s, no traj generated");
                return false;
            }
            auto gen_res = gen_traj_future.get();
            if(!gen_res->success){
                RCLCPP_ERROR(node_->get_logger(),"Traj generation failed %s",gen_res->message.c_str());
                return false;
            }
            RCLCPP_INFO(node_->get_logger(),"Traj generated, now executing");
            auto exec_traj_future = right_execute_trajectory_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
            if(exec_traj_future.wait_for(10s) != std::future_status::ready){
                RCLCPP_ERROR(node_->get_logger(),"Waited for 10s, no traj executed");
                return false;
            }
            auto exec_res = exec_traj_future.get();
            if(!exec_res->success){
                RCLCPP_ERROR(node_->get_logger(),"Traj exec failed");
                return false;
            }
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"No side \"%s\"",side.c_str());
            return false;
        }
        RCLCPP_INFO(node_->get_logger(),"Traj exec succeded");
        return true;
    }
    
    // synchronous joint state planning based on input joint_positions and move_group_interface
    void move_to_joint_positions(const std::vector<double>joint_positions, std::shared_ptr<MoveGroupInterface> move_group_interface){
        move_group_interface->setStartStateToCurrentState();
        // Speed control
        move_group_interface->setMaxVelocityScalingFactor(1.0);     // 0–1
        move_group_interface->setMaxAccelerationScalingFactor(1.0); // 0–1
        move_group_interface->setJointValueTarget(joint_positions);
        auto const [success, plan] = [this,move_group_interface]{
            moveit::planning_interface::MoveGroupInterface::Plan msg;
            auto const ok=static_cast<bool>(move_group_interface->plan(msg));
            return std::make_pair(ok,msg);
        }();
        if(success){
            move_group_interface->execute(plan);
        }
    }

    void print_state(const std_srvs::srv::Trigger::Request::SharedPtr request,std_srvs::srv::Trigger::Response::SharedPtr response){
        auto current_state = left_move_group_interface_->getCurrentState();
        (void)current_state;
        auto current_pose = left_move_group_interface_->getCurrentPose();
        auto current_joint_values = left_move_group_interface_->getCurrentJointValues();

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
        auto left_details = print_pose();

        current_state = right_move_group_interface_->getCurrentState();
        current_pose = right_move_group_interface_->getCurrentPose();
        current_joint_values = right_move_group_interface_->getCurrentJointValues();

        response->message = left_details + print_pose();
        response->success = true;
    }

    // synchronous cartesian trajectory generation and execution given waypoints and move_group_interface
    bool execute_cartesian_waypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints,std::shared_ptr<MoveGroupInterface> move_group_interface){
        move_group_interface->setStartStateToCurrentState();
        move_group_interface->setMaxVelocityScalingFactor(1.0);     // 0–1
        move_group_interface->setMaxAccelerationScalingFactor(1.0);
        moveit_msgs::msg::RobotTrajectory trajectory;
        const double eef_step = 0.002;
        const double jump_threshold = 0.0;

        RCLCPP_INFO(node_->get_logger(), "Computing cartesian path");
        double fraction = move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

        if(fraction < 1.0){
            RCLCPP_ERROR(node_->get_logger(),"Cartesian path planning failed, fraction: %f", fraction);
            return false;
        }

        RCLCPP_INFO(node_->get_logger(),"Trajectory created, attempting to execute now");

        moveit::planning_interface::MoveGroupInterface::Plan plan;
        plan.trajectory_ = trajectory;

        auto result = move_group_interface->execute(plan);
        if(result != moveit::core::MoveItErrorCode::SUCCESS){
            RCLCPP_ERROR(node_->get_logger(), "Cartesian path execution failed");
            return false;
        }
        move_group_interface->setStartStateToCurrentState();
        return true;
    }

    void do_fk(std::vector<double> joint_state, std::string side){
        robot_model_loader::RobotModelLoader robot_model_loader(node_);
        const moveit::core::RobotModelPtr& kinematic_model = robot_model_loader.getModel();
        RCLCPP_INFO(node_->get_logger(), "Model frame: %s", kinematic_model->getModelFrame().c_str());
        moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(kinematic_model));
        robot_state->setToDefaultValues();
        const moveit::core::JointModelGroup* joint_model_group;
        if(side=="left"){
            joint_model_group = kinematic_model->getJointModelGroup(this->left_planning_group_);
        }
        else if(side=="right"){
            joint_model_group = kinematic_model->getJointModelGroup(this->right_planning_group_);
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"No side \"%s\"",side.c_str());
            return;
        }
        const std::vector<std::string>& joint_names = joint_model_group->getVariableNames();
        robot_state->setJointGroupPositions(joint_model_group, joint_state);
        robot_state->update();

        for (std::size_t i = 0; i < joint_names.size(); ++i)
        {
          RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_state[i]);
        }

        Eigen::Isometry3d end_effector_state;
        if(side=="left")
            end_effector_state = robot_state->getGlobalLinkTransform(left_endeffector_link_);
        else if(side=="right")
            end_effector_state = robot_state->getGlobalLinkTransform(right_endeffector_link_);    
        else{
            RCLCPP_INFO(node_->get_logger(),"No side \"%s\"",side.c_str());
            return;
        }
        /* Print end-effector pose. Remember that this is in the model frame */
        RCLCPP_INFO_STREAM(node_->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
        RCLCPP_INFO_STREAM(node_->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
    }

    void do_ik(const geometry_msgs::msg::Pose& eepose, std::string side) {
        Eigen::Isometry3d ee_state = Eigen::Isometry3d::Identity();
        ee_state.translation() = Eigen::Vector3d(
            eepose.position.x,
            eepose.position.y,
            eepose.position.z
        );
        ee_state.linear() =
            Eigen::Quaterniond(
                eepose.orientation.w,
                eepose.orientation.x,
                eepose.orientation.y,
                eepose.orientation.z
            ).normalized().toRotationMatrix();

        bool found_ik = false;
        std::vector<std::string> joint_names;
        std::vector<double> joint_values;

        if (side == "left") {
            current_robot_state_->setJointGroupPositions(
                left_joint_group_model_,
                left_move_group_interface_->getCurrentJointValues()
            );
            current_robot_state_->update();
            found_ik = current_robot_state_->setFromIK(left_joint_group_model_, ee_state, 0.1);
            joint_names = left_joint_group_model_->getVariableNames();
            if (found_ik) {
                current_robot_state_->copyJointGroupPositions(left_joint_group_model_, joint_values);
            }
        } else if (side == "right") {
            current_robot_state_->setJointGroupPositions(
                right_joint_group_model_,
                right_move_group_interface_->getCurrentJointValues()
            );
            current_robot_state_->update();
            found_ik = current_robot_state_->setFromIK(right_joint_group_model_, ee_state, 0.1);
            joint_names = right_joint_group_model_->getVariableNames();
            if (found_ik) {
                current_robot_state_->copyJointGroupPositions(right_joint_group_model_, joint_values);
            }
        } else {
            RCLCPP_INFO(node_->get_logger(), "No side \"%s\"", side.c_str());
            return;
        }

        if (!found_ik) {
            RCLCPP_INFO(node_->get_logger(), "Did not find IK solution");
            return;
        }

        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            RCLCPP_INFO(node_->get_logger(), "Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
        }
    }

private:
    std::thread thread_;
    
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr moveit_executor_;
    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    rclcpp::Clock system_clock_;

    // moveit stuff
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr current_robot_state_;
    const moveit::core::JointModelGroup* left_joint_group_model_;
    const moveit::core::JointModelGroup* right_joint_group_model_;

    // servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr sample_publisher_;

    // subscribers
    
    // clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr left_execute_trajectory_client_;
    rclcpp::Client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>::SharedPtr left_generate_trajectory_client_;
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr right_execute_trajectory_client_;
    rclcpp::Client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>::SharedPtr right_generate_trajectory_client_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr left_set_io_client_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr right_set_io_client_;

    // timers
    rclcpp::TimerBase::SharedPtr sample_timer_;

    // parameters and data
    std::string left_planning_group_;
    std::string left_endeffector_link_;
    std::string right_planning_group_;
    std::string right_endeffector_link_;
    std::vector<double> left_joint_state_target_;
    std::vector<double> right_joint_state_target_;
};
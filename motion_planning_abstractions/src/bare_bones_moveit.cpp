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
#include "ur_msgs/srv/set_io.hpp"
#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "motion_planning_abstractions_msgs/srv/generate_trajectory.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include "motion_planning_abstractions/bare_bones_moveit.hpp"


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


BareBonesMoveit::BareBonesMoveit(rclcpp::Node::SharedPtr node)
{   
    node_ = node;
    
    node_->declare_parameter<std::string>("planning_group", "right_ur16e");
    node_->declare_parameter<std::string>("endeffector_link", "right_tool0");
    node_->declare_parameter<std::string>("tscubic_gen_traj_ns", "/right_task_space_cubic_polynomial_trajectory_server/generate_trajectory");
    node_->declare_parameter<std::string>("tscubic_exec_traj_ns", "/right_task_space_cubic_polynomial_trajectory_server/execute_trajectory");
    node_->declare_parameter<std::string>("set_io_ns","/right_io_and_status_controller/set_io");

    planning_group_ = node_->get_parameter("planning_group").as_string();
    endeffector_link_ = node_->get_parameter("endeffector_link").as_string();
    tscubic_gen_traj_ns_ = node_->get_parameter("tscubic_gen_traj_ns").as_string();
    tscubic_exec_traj_ns_ = node_->get_parameter("tscubic_exec_traj_ns").as_string();
    set_io_ns_ = node_->get_parameter("set_io_ns").as_string();

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
    
    // callback group as reentrant for parallel execution
    callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    
    // servers
    print_state_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/print_robot_state",
        std::bind(&BareBonesMoveit::print_state, this,std::placeholders::_1, std::placeholders::_2),
        rmw_qos_profile_services_default,
        callback_group_
    );
    test_server_ = node_->create_service<std_srvs::srv::Trigger>(
        "~/test_server",
        [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
            res->success = test_server_callback_();
            return;
        },
        rmw_qos_profile_services_default,
        callback_group_
    );
    
    // service clients
    execute_trajectory_client_ = node_->create_client<std_srvs::srv::Trigger>(tscubic_exec_traj_ns_);
    generate_trajectory_client_ = node_->create_client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>(tscubic_gen_traj_ns_);
    set_io_client_ = node_->create_client<ur_msgs::srv::SetIO>(set_io_ns_,rmw_qos_profile_services_default,callback_group_);

    // create checks for each client
    if(!set_io_client_->wait_for_service(3s)){
        RCLCPP_ERROR(node_->get_logger(),"Set Io client is not connected, ignoring all service calls");
        is_set_io_client_ready_ = false;
    }
    else{
        RCLCPP_INFO(node_->get_logger(),"Set Io client is connected");
        is_set_io_client_ready_ = true;
    }

    if(!generate_trajectory_client_->wait_for_service(3s)){
        RCLCPP_ERROR(node_->get_logger(),"Generate Cubic trajectory is not connected, ignoring all service calls");
        is_generate_trajectory_client_ready_ = false;
    }
    else{
        RCLCPP_INFO(node_->get_logger(),"Generate trajectory client is connected");
        is_generate_trajectory_client_ready_ =true;
    }
    if(!execute_trajectory_client_->wait_for_service(3s)){
        RCLCPP_ERROR(node_->get_logger(),"Ececute Cubic trajectory is not connected, ignoring all service calls");
        is_execute_trajectory_client_ready_ = false;
    }
    else{
        RCLCPP_INFO(node_->get_logger(),"Execute trajectory client is connected");
        is_execute_trajectory_client_ready_ = true;
    }
    
}

// use this to start the execute trajectory asynchronously, note that at a time, asyncly execute trajectories for different arms, do not do it for the same arm, unexpected behavior
std::shared_ptr<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> BareBonesMoveit::async_start_execute_waypoints_cubic(
    std::vector<geometry_msgs::msg::Pose> waypoints, 
    std::vector<double> durations, 
    double average_speed, 
    double waypoint_speed
){
        if (!(is_execute_trajectory_client_ready_ && is_generate_trajectory_client_ready_)){
            RCLCPP_ERROR(node_->get_logger(),"The cubic polynomial trajectory services are not connected, exiting");
            return nullptr;
        }
        auto goal = std::make_shared<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request>();
        goal->average_speed = average_speed;
        goal->durations = durations;
        goal->waypoint_speed = waypoint_speed;
        goal->waypoints = waypoints;
        std::shared_ptr<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Response> gen_response;

        auto future = generate_trajectory_client_->async_send_request(goal);
        if(future.wait_for(2s)!=std::future_status::ready){
            RCLCPP_ERROR(node_->get_logger(),"Generate left trajectory service call timed out!");
            return nullptr;
        }
        gen_response = future.get();    

        if(gen_response->success==true && gen_response->fraction == 100)
            RCLCPP_INFO(node_->get_logger(),"Generate trajectory succeeded, message : %s", gen_response->message.c_str());
        else{
            RCLCPP_INFO(node_->get_logger(),"Generate trajectory failed, message : %s", gen_response->message.c_str());
            return nullptr;
        }
        
        RCLCPP_INFO(node_->get_logger(),"Starting the trajectory execution");
        auto exec_future = std::make_shared<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>>();
        
        // start the execution of the trajectory
        std::shared_ptr<std_srvs::srv::Trigger::Request> exec_req=  std::make_shared<std_srvs::srv::Trigger::Request>();
        *exec_future = execute_trajectory_client_->async_send_request(exec_req);
        return exec_future;
}

// use this to block till execute cubic trajectory returns a response
template<typename Rep, typename Period>
std_srvs::srv::Trigger::Response::SharedPtr BareBonesMoveit::block_till_response_execute_cubic_trajectory(
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
geometry_msgs::msg::Pose::SharedPtr BareBonesMoveit::get_current_ee_pose(){
    return std::make_shared<geometry_msgs::msg::Pose>(move_group_interface_->getCurrentPose().pose);
}

// ease function to get the current joint states of robot
std::vector<double> BareBonesMoveit::get_current_joint_state(){
    return move_group_interface_->getCurrentJointValues();
}

// TEST SERVER CALLBACK HERE
bool BareBonesMoveit::test_server_callback_(){
    auto eepose = move_group_interface_->getCurrentPose();
    do_ik(eepose.pose);
    do_fk(std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0});
    
    auto LOGGER = node_->get_logger();
    
    auto current_pose = move_group_interface_->getCurrentPose().pose;
    RCLCPP_INFO(LOGGER,"Current x:%.2f, y:%.2f, z:%.2f, qw:%.2f,qx:%.2f,qy:%.2f,qz:%.2f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.w,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z
    );
    move_to_joint_positions(std::vector<double>{
        -1.3648873614431747,
        -1.7595786208070698,
        2.0876774906060356,
        -1.7340977760567944,
        -5.633357407467017,
        1.3341727624436612
    });
    
    rclcpp::sleep_for(std::chrono::milliseconds(3000));
    RCLCPP_INFO(LOGGER,"Finished the boring stuff, starting cubic traj now");
    // cubic trajectory client
    current_pose = move_group_interface_->getCurrentPose().pose;
    RCLCPP_INFO(LOGGER,"Current x:%.2f, y:%.2f, z:%.2f, qw:%.2f,qx:%.2f,qy:%.2f,qz:%.2f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.w,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z
    );
    auto wp1(current_pose),wp2(current_pose),wp3(current_pose);
    wp1.position.x += 0.3;
    wp2.position.y -= 0.1;
    wp3.position.z += 0.05;
    auto req = std::make_shared<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request>();
    req->durations = std::vector<double>{0.0,1,0.8,2};
    req->waypoints = std::vector<geometry_msgs::msg::Pose>{current_pose,wp1,wp2,wp3};
    auto gen_traj_future = generate_trajectory_client_->async_send_request(req);
    if(gen_traj_future.wait_for(std::chrono::seconds(5)) != std::future_status::ready){
        RCLCPP_ERROR(node_->get_logger(),"Waited for 5s, no traj generated");
        return false;
    }
    auto gen_res = gen_traj_future.get();
    if(!gen_res->success){
        RCLCPP_ERROR(node_->get_logger(),"Traj generation failed %s",gen_res->message.c_str());
        RCLCPP_INFO(LOGGER,"Failed to generate trajectory");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(),"Traj generated, now executing");
    auto exec_traj_future = execute_trajectory_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if(exec_traj_future.wait_for(5s) != std::future_status::ready){
        RCLCPP_ERROR(node_->get_logger(),"Waited for 5s, no traj executed");
        return false;
    }
    auto exec_res = exec_traj_future.get();
    if(!exec_res->success){
        RCLCPP_ERROR(node_->get_logger(),"Traj exec failed");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(),"Traj exec succeded");
    rclcpp::sleep_for(std::chrono::milliseconds(1000));
    
    current_pose = move_group_interface_->getCurrentPose().pose;
    RCLCPP_INFO(LOGGER,"Current x:%.2f, y:%.2f, z:%.2f, qw:%.2f,qx:%.2f,qy:%.2f,qz:%.2f",
        current_pose.position.x,
        current_pose.position.y,
        current_pose.position.z,
        current_pose.orientation.w,
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z
    );
    execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{current_pose,wp1,wp2,wp3},
        std::vector<double>{0.0,1.0,0.8,2},
        0.3,
        0.05
    );
    execute_waypoints(std::vector<geometry_msgs::msg::Pose>{current_pose,wp1,wp2,wp3});

    auto exec_future = async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose>{current_pose,wp1,wp2,wp3},
        std::vector<double>{0.0,1.0,0.8,2},
        0.3,
        0.05
    );

    block_till_response_execute_cubic_trajectory(exec_future,15s);
}

// TS CUBIC POLYNOMIAL EXECUTION
bool BareBonesMoveit::execute_waypoints_cubic(std::vector<geometry_msgs::msg::Pose> waypoints, std::vector<double> durations, double average_speed, double waypoint_speed){
    
    if (is_execute_trajectory_client_ready_ && is_generate_trajectory_client_ready_){
        RCLCPP_ERROR(node_->get_logger(),"The cubic polynomial trajectory services are not connected, exiting");
        return false;
    }
    
    auto req = std::make_shared<motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request>();
    req->durations = durations;
    req->waypoints = waypoints;
    req->average_speed = average_speed;
    req->waypoint_speed = waypoint_speed;
    auto gen_traj_future = generate_trajectory_client_->async_send_request(req);
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
    auto exec_traj_future = execute_trajectory_client_->async_send_request(std::make_shared<std_srvs::srv::Trigger::Request>());
    if(exec_traj_future.wait_for(5s) != std::future_status::ready){
        RCLCPP_ERROR(node_->get_logger(),"Waited for 5s, no traj executed");
        return false;
    }
    auto exec_res = exec_traj_future.get();
    if(!exec_res->success){
        RCLCPP_ERROR(node_->get_logger(),"Traj exec failed");
        return false;
    }
    RCLCPP_INFO(node_->get_logger(),"Traje exec succeded");
    return true;
}

// JOINT SPACE PLANNING AND EXECUTION 
void BareBonesMoveit::move_to_joint_positions(const std::vector<double>joint_positions){
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

//  PRINT STATE CALLBACK
void BareBonesMoveit::print_state(const std_srvs::srv::Trigger::Request::SharedPtr request,std_srvs::srv::Trigger::Response::SharedPtr response){
    if(move_group_interface_==nullptr){
        RCLCPP_ERROR(node_->get_logger(),"Move group interface is uninitialized, exiting");
        return;
    }

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

// CARTESIAN MOVEMENT CALLBACK
bool BareBonesMoveit::execute_waypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints){
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

// FORWARD KINEMATICS CALLBACK
void BareBonesMoveit::do_fk(std::vector<double> joint_state){
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
    const Eigen::Isometry3d& end_effector_state = robot_state->getGlobalLinkTransform(endeffector_link_);
    /* Print end-effector pose. Remember that this is in the model frame */
    RCLCPP_INFO_STREAM(node_->get_logger(), "Translation: \n" << end_effector_state.translation() << "\n");
    RCLCPP_INFO_STREAM(node_->get_logger(), "Rotation: \n" << end_effector_state.rotation() << "\n");
}

// INVERSE KINEMATICS CALLBACK
void BareBonesMoveit::do_ik(const geometry_msgs::msg::Pose& eepose){
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
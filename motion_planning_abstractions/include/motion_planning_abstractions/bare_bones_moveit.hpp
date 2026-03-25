// single arm control abstraction header

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


class BareBonesMoveit{
public:

    BareBonesMoveit(rclcpp::Node::SharedPtr node_);

    // callback bounded to "~/test_server"
    bool test_server_callback_();

    // asynchronous start to cubic trajectory execution
    std::shared_ptr<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> async_start_execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose> waypoints, 
        std::vector<double> durations, 
        double average_speed, 
        double waypoint_speed
    );

    // to block till the execution of asynchronous cubic trajectory is complete
    template<typename Rep, typename Period>
    std_srvs::srv::Trigger::Response::SharedPtr block_till_response_execute_cubic_trajectory(
        std::shared_ptr<std::shared_future<std_srvs::srv::Trigger::Response::SharedPtr>> exec_future,
        std::chrono::duration<Rep,Period> wait_duration
    );

    // get the current ee pose
    geometry_msgs::msg::Pose::SharedPtr get_current_ee_pose();

    // get the current joint positions
    std::vector<double> get_current_joint_state();

    // ts cubic waypoint execution
    bool execute_waypoints_cubic(
        std::vector<geometry_msgs::msg::Pose> waypoints, 
        std::vector<double> durations, 
        double average_speed, 
        double waypoint_speed
    );

    // joint space movement
    void move_to_joint_positions(const std::vector<double>joint_positions);

    // print the current joint angles
    void print_state(
        const std_srvs::srv::Trigger::Request::SharedPtr request,
        std_srvs::srv::Trigger::Response::SharedPtr response
    );

    // cartesian waypoints execution
    bool execute_waypoints(const std::vector<geometry_msgs::msg::Pose> &waypoints);

    // ft on given joint states
    void do_fk(std::vector<double> joint_state);

    // ik on given pose
    void do_ik(const geometry_msgs::msg::Pose& eepose);

private:
    std::thread thread_;
    
    std::shared_ptr<MoveGroupInterface> move_group_interface_;
    rclcpp::Node::SharedPtr node_;
    
    rclcpp::executors::MultiThreadedExecutor::SharedPtr executor_;
    
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    
    rclcpp::Clock system_clock_;

    // moveit stuff
    moveit::core::RobotModelPtr kinematic_model_;
    moveit::core::RobotStatePtr current_robot_state_;
    const moveit::core::JointModelGroup* joint_group_model_;

    // servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_server_;

    // publishers
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr sample_publisher_;

    // subscribers
    
    // clients
    rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr execute_trajectory_client_;
    rclcpp::Client<motion_planning_abstractions_msgs::srv::GenerateTrajectory>::SharedPtr generate_trajectory_client_;
    rclcpp::Client<ur_msgs::srv::SetIO>::SharedPtr set_io_client_;
    bool is_execute_trajectory_client_ready_=false;
    bool is_generate_trajectory_client_ready_=false;
    bool is_set_io_client_ready_=false;

    // timers
    rclcpp::TimerBase::SharedPtr sample_timer_;

    // parameters and data
    std::string planning_group_;
    std::string endeffector_link_;
    std::string tscubic_gen_traj_ns_;
    std::string tscubic_exec_traj_ns_;
    std::string set_io_ns_;
};
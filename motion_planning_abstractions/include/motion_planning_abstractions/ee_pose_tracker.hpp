#include <functional>
#include <memory>
#include <chrono>
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"

using namespace std::chrono_literals;

#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/bare_bones_moveit.hpp"

class PoseTracker{
    public:
        enum class State{
            UN_PREPPED= 0,
            PREPPED= 1,
            TRACKING= 2
        };

        PoseTracker(){}

        PoseTracker(rclcpp::Node::SharedPtr node);

        bool prepare_tracker_();

        bool unprepare_tracker_();
        
        bool start_tracking_();

        bool stop_tracking_();

        void set_target_pose_(const geometry_msgs::msg::Pose& target_pose);

        void clear_target_pose_();

    private:
        // internal data
        geometry_msgs::msg::Pose::SharedPtr target_pose_;
        geometry_msgs::msg::Twist output_velocity_;
        
        std::shared_ptr<EEServo> servo_interface_;
        std::shared_ptr<BareBonesMoveit> single_arm_control_interface_;
        
        double linear_P_ = 1.0;
        double linear_D_ = 1.0;
        double angular_P_ = 1.0;
        double angular_D_ = 1.0;

        State current_state_=State::UN_PREPPED;

        rclcpp::Clock::SharedPtr wall_clock_;

        // node
        rclcpp::Node::SharedPtr node_;

        // servers
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr prepare_tracking_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr unprepare_tracking_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr start_tracking_server_;
        rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr stop_tracking_server_;

        // callback group
        rclcpp::CallbackGroup::SharedPtr mex_cb_group_;
        rclcpp::CallbackGroup::SharedPtr parallel_cb_group_;

        // Timer for velocity publication
        rclcpp::TimerBase::SharedPtr control_robot_timer_;

        // private functions
        void control_robot_timer_cb_();

        Eigen::Vector3d get_linear_error(const Eigen::Vector3d& current_position, const Eigen::Vector3d& target_position);
        Eigen::Vector3d get_angular_error(const Eigen::Quaterniond& current_orientation, const Eigen::Quaterniond& target_orientation);


};
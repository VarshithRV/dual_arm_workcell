// Node for pose tracking for a single arm
// Right now hardcode for a single arm, later generalize for different move groups
// Apart from the the servo parameters, no other parameters

#include <std_msgs/msg/int8.hpp>

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rmw/qos_profiles.h"
#include <moveit_servo/servo.h>
#include <moveit_servo/pose_tracking.h>
#include <moveit_servo/status_codes.h>
#include <moveit_servo/servo_parameters.h>
#include <moveit_servo/make_shared_from_pool.h>

#include <thread>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("left_pose_tracker");

// class for monitoring status of moveit_servo, basically monitors a status topic and then interprets it
class StatusMonitor{
    public:
        StatusMonitor(const rclcpp::Node::SharedPtr& node, const std::string& topic){
            sub_=node->create_subscription<std_msgs::msg::Int8>(topic, rclcpp::SystemDefaultsQoS(),
                                                                [this](const std_msgs::msg::Int8::ConstSharedPtr& msg){ //lambda for subscription lmao, nice
                                                                    return statusCB(msg);
                                                                });
        }

    private:
        void statusCB(const std_msgs::msg::Int8::ConstSharedPtr& msg){ //need to revise how to learn the reference operator
            moveit_servo::StatusCode latest_status = static_cast<moveit_servo::StatusCode>(msg->data);
            if(latest_status!=status_){
                status_ = latest_status;
                const auto& status_str = moveit_servo::SERVO_STATUS_CODE_MAP.at(status_);
                RCLCPP_INFO_STREAM(LOGGER,"Servo status : "<< status_str);
            }
        }
        moveit_servo::StatusCode status_=moveit_servo::StatusCode::INVALID;
        rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr sub_;
};

int main(int argc, char** argv){
    rclcpp::init(argc,argv);
    rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("left_pose_tracker"); //ungeneralized.
    
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread executor_thread([&executor](){executor.spin();});

    auto servo_parameters = moveit_servo::ServoParameters::makeServoParameters(node,"left_servo_node_main"); //ungeneralized
    // this probably gets the servo parameters through the node shared ptr, 
    // but not sure how this will work when there are multiple servo nodes, 
    // hence multiple servo parameters, also, this might be only the ros servo 
    // parameters local to this node
    
    if(servo_parameters == nullptr){
        RCLCPP_FATAL(LOGGER,"Could not get servo parameters!");
        exit(EXIT_FAILURE);
    }

    // load the planning scene monitor
    planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor;
    planning_scene_monitor = std::make_shared<planning_scene_monitor::PlanningSceneMonitor>(node,"robot_description");
    if(!planning_scene_monitor->getPlanningScene()){
        RCLCPP_ERROR_STREAM(LOGGER,"Error in setting up the PlanningSceneMonitor");
        exit(EXIT_FAILURE);
    }

    planning_scene_monitor->providePlanningSceneService();
    planning_scene_monitor->startSceneMonitor();
    planning_scene_monitor->startWorldGeometryMonitor(
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_COLLISION_OBJECT_TOPIC,
        planning_scene_monitor::PlanningSceneMonitor::DEFAULT_PLANNING_SCENE_WORLD_TOPIC,
        false //skip octomap monitor (i'm assuming this will be true when we have depth maps)
    );
    planning_scene_monitor->startStateMonitor(servo_parameters->joint_topic);
    planning_scene_monitor->startPublishingPlanningScene(planning_scene_monitor::PlanningSceneMonitor::UPDATE_SCENE);

    // wait for planning scene monitor to setup
    if(!planning_scene_monitor->waitForCurrentRobotState(node->now(),5.0/*seconds*/)){
        RCLCPP_ERROR_STREAM(LOGGER,"Error waiting for current robot state in PlanningSceneMonitor.");
        exit(EXIT_FAILURE);
    }

    // create the pose tracker?
    moveit_servo::PoseTracking tracker(node, servo_parameters, planning_scene_monitor);

    // make a publisher for sending pose commands
    auto target_pose_pub = node->create_publisher<geometry_msgs::msg::PoseStamped>( //ungeneralized
        "target_pose",
        rclcpp::SystemDefaultsQoS()
    );

    // subscribe to servo status, earlier written class
    StatusMonitor status_monitor(node, servo_parameters->status_topic);

    Eigen::Vector3d lin_tol{0.001, 0.001, 0.001}; //ahhh tolerance?
    double rot_tol = 0.01; // rotation tolerance hahaha, we had this too lmao

    // get the current ee tf
    geometry_msgs::msg::TransformStamped current_ee_tf;
    tracker.getCommandFrameTransform(current_ee_tf);

    // convert it to a pose
    geometry_msgs::msg::PoseStamped target_pose;
    target_pose.header.frame_id=current_ee_tf.header.frame_id;
    target_pose.pose.position.x = current_ee_tf.transform.translation.x;
    target_pose.pose.position.y = current_ee_tf.transform.translation.y;
    target_pose.pose.position.z = current_ee_tf.transform.translation.z;
    target_pose.pose.orientation = current_ee_tf.transform.rotation;

    // modify it a bit ig
    target_pose.pose.position.x += 0.1;

    // reset target pose
    tracker.resetTargetPose();

    // publish target pose
    target_pose.header.stamp = node->now();
    target_pose_pub->publish(target_pose);

    // run the pose tracking in a new thread
    std::thread move_to_pose_thread([&tracker,&lin_tol,&rot_tol]{
        moveit_servo::PoseTrackingStatusCode tracking_status =
        tracker.moveToPose(lin_tol, rot_tol, 0.1/*target pose timeout*/);
        RCLCPP_INFO_STREAM(LOGGER,"Pose tracker exited with status: " << moveit_servo::POSE_TRACKING_STATUS_CODE_MAP.at(tracking_status));
    });

    rclcpp::WallRate loop_rate(50);
    for(size_t i=0; i<500; ++i){
        // modify the pose target a little bit each cycle
        // this is a dynamic pose target
        target_pose.pose.position.z += 0.0004;
        target_pose.header.stamp = node->now();
        target_pose_pub->publish(target_pose);

        loop_rate.sleep();
    }

    // make sure the tracker is stopped and clean up
    move_to_pose_thread.join();

    // kill executor thread before shutdown
    executor.cancel();
    executor_thread.join();

    rclcpp::shutdown();
    return EXIT_SUCCESS;
}
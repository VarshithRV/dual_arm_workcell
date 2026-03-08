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
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rmw/qos_profiles.h"
#include "std_srvs/srv/trigger.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class TSCubicPolynomialTraj
{
public:
    struct trajPoint{
        geometry_msgs::msg::Pose waypoint;
        geometry_msgs::msg::Twist velocity;
        geometry_msgs::msg::Twist acceleration;
        double duration_from_start;
    };

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

        // print shit
        auto planning_frame = this->move_group_interface_->getPlanningFrame();
        RCLCPP_INFO(node_->get_logger(), "Planning frame : %s", planning_frame.c_str());

        auto endeffector = this->move_group_interface_->getEndEffectorLink();
        RCLCPP_INFO(node_->get_logger(), "End Effector Link : %s", endeffector.c_str());

        auto current_pose = this->move_group_interface_->getCurrentPose(endeffector);
        RCLCPP_INFO(node_->get_logger(),"x : %f, y : %f, z : %f",current_pose.pose.position.x,current_pose.pose.position.y,current_pose.pose.position.z);

        // servers
        callback_group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        print_state_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_robot_state",std::bind(&TSCubicPolynomialTraj::print_state, this,std::placeholders::_1, std::placeholders::_2),rmw_qos_profile_services_default,callback_group_);
        test_server_ = node_->create_service<std_srvs::srv::Trigger>("~/test_server",
            [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                res->success = test_server_callback_();
                return;
            }
        );
        print_latest_trajectory_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_latest_trajectory",
            [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                res->success = print_latest_trajectory_server_callback_();
                return;
            }
        );

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

    // get_coeffs_ function for the cubic polynomial given time, initial_conditions and final_conditions for one axes at a time
    Eigen::Vector<double,4> get_coeffs_(Eigen::Vector2d initial_conditions, Eigen::Vector2d final_conditions, double duration){ // conditions are in format [p_;v_]
        Eigen::Vector<double,4> conditions;
        conditions << initial_conditions,final_conditions;
        Eigen::Matrix<double,4,4> A;
        double t = duration;
        A <<
            1.0,0.0,0.0,0.0,
            0.0,1.0,0.0,0.0,
            1.0,t,std::pow(t,2),std::pow(t,3),
            0.0,1,2*t,3*pow(t,2);
        return A.colPivHouseholderQr().solve(conditions); 
    }

    // w_in can have R6 space
    // w_f can have R6 space
    // v_in can have R3 in translation but 0 in rotation
    // v_f can have R3 in translation but 0 in rotation
    // dt should be small
    std::vector<TSCubicPolynomialTraj::trajPoint> generate_trajectory_(
        geometry_msgs::msg::Pose w_in, geometry_msgs::msg::Twist v_in, 
        geometry_msgs::msg::Pose w_f, geometry_msgs::msg::Twist v_f, double duration, double dt
    ){
        std::vector<TSCubicPolynomialTraj::trajPoint> trajectory_msg; // initialize a trajectory message
        
        int size;
        if(duration/dt - int(duration/dt)!=0)
            size = duration/dt + 2;
        else
            size = duration/dt + 1;
            
        // init point,velocity,acceleration
        double timesteps[size];
        double x[size]={0};
        double y[size]={0};
        double z[size]={0};
        double theta[size]={0}; //angular displacement
        double dx[size]={0};
        double dy[size]={0};
        double dz[size]={0};
        double dtheta[size]={0}; //angular velocity
        double ddx[size]={0};
        double ddy[size]={0};
        double ddz[size]={0};
        double ddtheta[size]={0}; //angular acceleration
        Eigen::Vector3d a_cap; //axis of rotation

        // make timesteps
        for(int i=0; i<size; i++){
            if(i<size-1)
                timesteps[i] = i*dt;
            else
                timesteps[i] = duration;
        }

        // get the rotations stuff
        Eigen::Quaterniond q_in(w_in.orientation.w,w_in.orientation.x,w_in.orientation.y,w_in.orientation.z);
        Eigen::Quaterniond q_f(w_f.orientation.w,w_f.orientation.x,w_f.orientation.y,w_f.orientation.z);

        q_in.normalize();
        q_f.normalize();
        if(q_in.dot(q_f) < 0.0){
            q_f.coeffs() *= -1.0;
        }

        Eigen::Quaternion q_d = q_f * q_in.inverse();
        Eigen::AngleAxisd angleaxisd(q_d);
        double theta_f = angleaxisd.angle();
        a_cap = angleaxisd.axis();

        if(std::abs(theta_f) < 1e-8){
            a_cap = Eigen::Vector3d::UnitX();
            theta_f = 0.0;
        } 
        else{
            a_cap = angleaxisd.axis();
        }

        Eigen::Vector2d x_initial_conditions(w_in.position.x,v_in.linear.x);
        Eigen::Vector2d x_final_conditions(w_f.position.x,v_f.linear.x);
        Eigen::Vector4d x_coeffs = get_coeffs_(x_initial_conditions,x_final_conditions,duration);
        
        Eigen::Vector2d y_initial_conditions(w_in.position.y,v_in.linear.y);
        Eigen::Vector2d y_final_conditions(w_f.position.y,v_f.linear.y);
        Eigen::Vector4d y_coeffs = get_coeffs_(y_initial_conditions,y_final_conditions,duration);
        
        Eigen::Vector2d z_initial_conditions(w_in.position.z,v_in.linear.z);
        Eigen::Vector2d z_final_conditions(w_f.position.z,v_f.linear.z);
        Eigen::Vector4d z_coeffs = get_coeffs_(z_initial_conditions,z_final_conditions,duration);
        
        Eigen::Vector2d theta_initial_conditions(0,0);
        Eigen::Vector2d theta_final_conditions(theta_f,0);
        Eigen::Vector4d theta_coeffs = get_coeffs_(theta_initial_conditions,theta_final_conditions,duration);
        
        Eigen::Quaterniond current_orientation(q_in);
        current_orientation.normalize();

        for(int i=0; i <size; i++){

            // positions
            x[i] = x_coeffs[0] + x_coeffs[1]*timesteps[i] + x_coeffs[2]*pow(timesteps[i],2) + x_coeffs[3]*pow(timesteps[i],3);
            y[i] = y_coeffs[0] + y_coeffs[1]*timesteps[i] + y_coeffs[2]*pow(timesteps[i],2) + y_coeffs[3]*pow(timesteps[i],3);
            z[i] = z_coeffs[0] + z_coeffs[1]*timesteps[i] + z_coeffs[2]*pow(timesteps[i],2) + z_coeffs[3]*pow(timesteps[i],3);
            theta[i] = theta_coeffs[0] + theta_coeffs[1]*timesteps[i] + theta_coeffs[2]*pow(timesteps[i],2) + theta_coeffs[3]*pow(timesteps[i],3);

            // velocities
            dx[i] = x_coeffs[1] + 2*x_coeffs[2]*timesteps[i] + 3*x_coeffs[3]*pow(timesteps[i],2);
            dy[i] = y_coeffs[1] + 2*y_coeffs[2]*timesteps[i] + 3*y_coeffs[3]*pow(timesteps[i],2);
            dz[i] = z_coeffs[1] + 2*z_coeffs[2]*timesteps[i] + 3*z_coeffs[3]*pow(timesteps[i],2);
            dtheta[i] = theta_coeffs[1] + 2*theta_coeffs[2]*timesteps[i] + 3*theta_coeffs[3]*pow(timesteps[i],2);
            
            // accelerations
            ddx[i] = 2*x_coeffs[2] + 6*x_coeffs[3]*timesteps[i];
            ddy[i] = 2*y_coeffs[2] + 6*y_coeffs[3]*timesteps[i];
            ddz[i] = 2*z_coeffs[2] + 6*z_coeffs[3]*timesteps[i];
            ddtheta[i] = 2*theta_coeffs[2] + 6*theta_coeffs[3]*timesteps[i];
            
            current_orientation = Eigen::Quaterniond(Eigen::AngleAxisd(theta[i],a_cap)) * q_in;
            current_orientation.normalize();
            
            trajPoint point;
            point.waypoint.position.x = x[i];
            point.waypoint.position.y = y[i];
            point.waypoint.position.z = z[i];
            point.waypoint.orientation.w = current_orientation.w();
            point.waypoint.orientation.x = current_orientation.x();
            point.waypoint.orientation.y = current_orientation.y();
            point.waypoint.orientation.z = current_orientation.z();

            point.velocity.linear.x = dx[i];
            point.velocity.linear.y = dy[i];
            point.velocity.linear.z = dz[i];
            point.velocity.angular.x = dtheta[i]*a_cap[0];
            point.velocity.angular.y = dtheta[i]*a_cap[1];
            point.velocity.angular.z = dtheta[i]*a_cap[2];

            point.acceleration.linear.x = ddx[i];
            point.acceleration.linear.y = ddy[i];
            point.acceleration.linear.z = ddz[i];
            point.acceleration.angular.x = ddtheta[i]*a_cap[0];
            point.acceleration.angular.y = ddtheta[i]*a_cap[1];
            point.acceleration.angular.z = ddtheta[i]*a_cap[2];

            point.duration_from_start = timesteps[i];

            trajectory_msg.push_back(point);
        }

        return trajectory_msg;
    }

    // provide a vector of poses are waypoints and a vector of doubles that give the velocity magnitudes at the corresponding waypoints in ms-1
    // provide the starting and ending velocity as 0.0, if not given, will be enforced anyways
    // if the lengths of the two vectors are different, then it will fail
    std::shared_ptr<std::vector<TSCubicPolynomialTraj::trajPoint>>
    waypointPlanning(std::vector<geometry_msgs::msg::Pose> waypoints,std::vector<double> waypoint_velocities){
        if (waypoints.size() != waypoint_velocities.size() || waypoints.empty()) {
            return nullptr;
        }
    
        std::vector<Eigen::Vector3d> waypoints_positions;
        std::vector<Eigen::Quaterniond> waypoints_orientations;
    
        for (const geometry_msgs::msg::Pose& waypoint : waypoints) {
            waypoints_positions.push_back(
                Eigen::Vector3d(waypoint.position.x, waypoint.position.y, waypoint.position.z));
            waypoints_orientations.push_back(
                Eigen::Quaterniond(waypoint.orientation.w, waypoint.orientation.x,
                                   waypoint.orientation.y, waypoint.orientation.z));
        }
    
        auto trajectory = std::make_shared<std::vector<TSCubicPolynomialTraj::trajPoint>>();
        std::vector<Eigen::Vector3d> waypoint_velocity_vectors;
        std::vector<double> durations;
    
        waypoint_velocities.front() = 0.0;
        waypoint_velocities.back() = 0.0;
    
        for (int i = 0; i < static_cast<int>(waypoints.size()); ++i) {
            if (i == 0 || i == static_cast<int>(waypoints.size()) - 1) {
                waypoint_velocity_vectors.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
            } else {
                Eigen::Vector3d velocity_direction = waypoints_positions[i + 1] - waypoints_positions[i - 1];
                if (velocity_direction.norm() > 1e-9) {
                    waypoint_velocity_vectors.push_back(
                        (velocity_direction / velocity_direction.norm()) * waypoint_velocities[i]);
                } else {
                    waypoint_velocity_vectors.push_back(Eigen::Vector3d(0.0, 0.0, 0.0));
                }
            }
        }
    
        for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; ++i) {
            Eigen::Vector3d displacement = waypoints_positions[i + 1] - waypoints_positions[i];
            durations.push_back(displacement.norm() / average_velocity_);
        }
        durations.push_back(0.0);

        // generate trajectories for the segments and return
        double cumulative_time = 0.0;

        for (int i = 0; i < static_cast<int>(waypoints_positions.size()) - 1; ++i) {
            geometry_msgs::msg::Pose w_in;
            w_in.position.x = waypoints_positions[i][0];
            w_in.position.y = waypoints_positions[i][1];
            w_in.position.z = waypoints_positions[i][2];
            w_in.orientation.x = waypoints_orientations[i].x();
            w_in.orientation.y = waypoints_orientations[i].y();
            w_in.orientation.z = waypoints_orientations[i].z();
            w_in.orientation.w = waypoints_orientations[i].w();
        
            geometry_msgs::msg::Pose w_f;
            w_f.position.x = waypoints_positions[i + 1][0];
            w_f.position.y = waypoints_positions[i + 1][1];
            w_f.position.z = waypoints_positions[i + 1][2];
            w_f.orientation.x = waypoints_orientations[i + 1].x();
            w_f.orientation.y = waypoints_orientations[i + 1].y();
            w_f.orientation.z = waypoints_orientations[i + 1].z();
            w_f.orientation.w = waypoints_orientations[i + 1].w();
        
            geometry_msgs::msg::Twist v_in;
            v_in.linear.x = waypoint_velocity_vectors[i][0];
            v_in.linear.y = waypoint_velocity_vectors[i][1];
            v_in.linear.z = waypoint_velocity_vectors[i][2];
            v_in.angular.x = 0.0;
            v_in.angular.y = 0.0;
            v_in.angular.z = 0.0;
        
            geometry_msgs::msg::Twist v_f;
            v_f.linear.x = waypoint_velocity_vectors[i + 1][0];
            v_f.linear.y = waypoint_velocity_vectors[i + 1][1];
            v_f.linear.z = waypoint_velocity_vectors[i + 1][2];
            v_f.angular.x = 0.0;
            v_f.angular.y = 0.0;
            v_f.angular.z = 0.0;
        
            double duration = durations[i];
        
            if (duration <= 1e-9) {
                continue;
            }
        
            auto dtrajectory = generate_trajectory_(w_in, v_in, w_f, v_f, duration, dt_);
        
            if (dtrajectory.empty()) {
                RCLCPP_WARN(node_->get_logger(),
                            "Segment %d trajectory generation failed or returned empty trajectory",
                            i);
                return nullptr;
            }
        
            const std::size_t start_idx = (i == 0) ? 0 : 1;
        
            for (std::size_t j = start_idx; j < dtrajectory.size(); ++j) {
                TSCubicPolynomialTraj::trajPoint point = (dtrajectory)[j];
            
                point.duration_from_start += cumulative_time;
                trajectory->push_back(point);
            }
        
            cumulative_time += duration;
        }

        latest_trajectory_ = trajectory;
        return trajectory;
    }
    
    // TEST SERVER CALLBACK HERE
    bool test_server_callback_(){
        RCLCPP_INFO(node_->get_logger(),"Entered test service");
        
        geometry_msgs::msg::Pose w_in,w_f;
        geometry_msgs::msg::Twist v_in,v_f;
        
        w_in.position.x = 0.1;
        w_f.position.x = 1.0;
        w_in.position.y = -0.5;
        w_f.position.y = -0.1;
        w_in.position.z = 0.0;
        w_f.position.z = 0.1;
        v_f.linear.x = 0.1;
        v_f.linear.z = 0.5;
        w_in.orientation.w = 1;
        w_in.orientation.x = 1;
        double duration = 0.55;
        double dt = 0.05;

        std::vector<TSCubicPolynomialTraj::trajPoint> trajectory =  generate_trajectory_(w_in,v_in,w_f,v_f,duration,dt);
        if(this->latest_trajectory_ == nullptr){
            latest_trajectory_ = std::make_shared<std::vector<TSCubicPolynomialTraj::trajPoint>>();
        }
        *latest_trajectory_ = trajectory;

        geometry_msgs::msg::Pose wp1,wp2,wp3,wp4;
        wp1.position.x=0.1;
        wp2.position.y=1.2;
        wp3.position.z=0.3;
        wp4.position.x=0.5;
        wp1.orientation.w=1;
        wp1.orientation.w=1;
        wp1.orientation.w=1;
        wp1.orientation.w=1;
        waypointPlanning(std::vector<geometry_msgs::msg::Pose>{wp1,wp2,wp3,wp4},std::vector<double>{0.0,0.1,0.2,0.0});

        return true;
    }

    bool print_latest_trajectory_server_callback_(){
        
        
        for(TSCubicPolynomialTraj::trajPoint point :*latest_trajectory_){
            RCLCPP_INFO(
                node_->get_logger(),
                "t=%.3f | "
                "pos [%.4f %.4f %.4f] | "
                "quat [%.4f %.4f %.4f %.4f] | "
                "lin vel [%.4f %.4f %.4f] | "
                "ang vel [%.4f %.4f %.4f] | "
                "lin acc [%.4f %.4f %.4f] | "
                "ang acc [%.4f %.4f %.4f]",

                point.duration_from_start,
            
                point.waypoint.position.x,
                point.waypoint.position.y,
                point.waypoint.position.z,
            
                point.waypoint.orientation.w,
                point.waypoint.orientation.x,
                point.waypoint.orientation.y,
                point.waypoint.orientation.z,
            
                point.velocity.linear.x,
                point.velocity.linear.y,
                point.velocity.linear.z,
            
                point.velocity.angular.x,
                point.velocity.angular.y,
                point.velocity.angular.z,
            
                point.acceleration.linear.x,
                point.acceleration.linear.y,
                point.acceleration.linear.z,
            
                point.acceleration.angular.x,
                point.acceleration.angular.y,
                point.acceleration.angular.z
            );
        }
        RCLCPP_INFO(node_->get_logger(),"Size of the trajectory message : %d",latest_trajectory_->size());
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
    
    rclcpp::Clock system_clock_;

    // servers
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_state_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr test_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_latest_trajectory_server_;

    // clients

    // publishers

    // subscribers
    
    // parameters and data
    std::string arm_side;
    std::string joint_trajectory_controller_;
    std::string planning_group_;
    std::string endeffector_link_;
    double maximum_task_space_velocity_; // not in use right now
    double maximum_task_space_acceleration_; // not in use right now
    double maximum_joint_space_velocity_; // not in use right now
    double maximum_joint_space_acceleration_; // not in use right now
    std::shared_ptr<std::vector<TSCubicPolynomialTraj::trajPoint>> latest_trajectory_;
    double average_velocity_=0.5; // change this to make pt to pt traj faster or slower by making this bigger or smaller
    double dt_=0.05; //trajectory interval
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto moveit_example = TSCubicPolynomialTraj();
    rclcpp::shutdown();
    return 0;
}

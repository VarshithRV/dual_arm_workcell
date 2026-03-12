// task space cubic polnomial traj server

#include <memory>
#include <functional>
#include <string>
#include <chrono>
#include <cstdlib>
#include <thread>
#include <vector>
#include <cmath>
#include <future>
#include <sstream>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/wrench.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "control_msgs/action/follow_joint_trajectory.hpp"

#include "moveit/move_group_interface/move_group_interface.h"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rmw/qos_profiles.h"
#include "std_srvs/srv/trigger.hpp"
#include "rosidl_runtime_cpp/traits.hpp"
#include "Eigen/Dense"
#include "Eigen/Geometry"
#include "motion_planning_abstractions_msgs/srv/generate_trajectory.hpp"
#include "motion_planning_abstractions_msgs/srv/execute_trajectory.hpp"
#include "trajectory_msgs/msg/joint_trajectory.hpp"

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>


using namespace std::chrono_literals;
using moveit::planning_interface::MoveGroupInterface;


class TSCubicPolynomialTraj{
public:
    struct trajPoint{
        geometry_msgs::msg::Pose waypoint;
        geometry_msgs::msg::Twist velocity;
        geometry_msgs::msg::Twist acceleration;
        double duration_from_start;
    };
    
    struct jointPoint{
        double position;
        double velocity;
    };
    
    struct jointSpaceTrajPoint{
        TSCubicPolynomialTraj::jointPoint basejoint;
        TSCubicPolynomialTraj::jointPoint shoulderjoint;
        TSCubicPolynomialTraj::jointPoint elbowjoint;
        TSCubicPolynomialTraj::jointPoint wrist1;
        TSCubicPolynomialTraj::jointPoint wrist2;
        TSCubicPolynomialTraj::jointPoint wrist3;
        double duration_from_start;
    };

    TSCubicPolynomialTraj()
    {
        node_ = std::make_shared<rclcpp::Node>("ts_cubic_polnomial_traj_server");

        // parameter declaration
        node_->declare_parameter<std::string>("planning_group", "right_ur16e");
        node_->declare_parameter<double>("maximum_task_space_velocity",1.0); // in ms-1
        node_->declare_parameter<double>("maximum_task_space_acceleration",3.0); // in ms-2
        node_->declare_parameter<double>("maximum_joint_space_velocity",M_PI); // in rads-1
        node_->declare_parameter<double>("maximum_joint_space_acceleration",M_PI); // in rads-2
        node_->declare_parameter<std::string>("arm_side", "right");
        node_->declare_parameter<std::string>("joint_trajectory_controller", "right_scaled_joint_trajectory_controller");
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
        print_latest_joint_space_trajectory_server_ = node_->create_service<std_srvs::srv::Trigger>("~/print_latest_joint_space_trajectory",
            [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                res->success = print_latest_joint_space_trajectory_server_callback_();
                return;
            }
        );
        generate_trajectory_server_ = node_->create_service<motion_planning_abstractions_msgs::srv::GenerateTrajectory>("~/generate_trajectory", 
            [this](motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request::SharedPtr req, motion_planning_abstractions_msgs::srv::GenerateTrajectory::Response::SharedPtr res){
                generate_trajectory_server_callback_(req,res);
                return;
            }
        );

        //////// REPLACING THE SERVER WITH A TRIGGER BECAUSE ITS HARD TO TEST WITH JUST COMMAND LINE, need to change this before using it properly
        // execute_trajectory_server_ = node_->create_service<motion_planning_abstractions_msgs::srv::ExecuteTrajectory>("~/execute_trajectory", 
        //     [this](motion_planning_abstractions_msgs::srv::ExecuteTrajectory::Request::SharedPtr req, motion_planning_abstractions_msgs::srv::ExecuteTrajectory::Response::SharedPtr res){
        //         execute_trajectory_server_callback_(req,res);
        //         return;
        //     }
        // );
        execute_trajectory_server_ = node_->create_service<std_srvs::srv::Trigger>("~/execute_trajectory", 
            [this](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
                res->success = execute_trajectory_server_callback_();
            }
        );

        // publisher
        jt_publisher_ = node_->create_publisher<trajectory_msgs::msg::JointTrajectory>("~/latest_trajectory",10);

        // action clients
        sjtc_client_ptr_ = rclcpp_action::create_client<control_msgs::action::FollowJointTrajectory>(
            node_,
            joint_trajectory_controller_ +"/follow_joint_trajectory"            
        );
        // wait for the action
        if(!this->sjtc_client_ptr_->wait_for_action_server()){
            RCLCPP_ERROR(node_->get_logger(),"SJTC action server is not available");
            rclcpp::shutdown();
        }

        // timers
        latest_jt_publisher_timer_ = node_->create_wall_timer(200ms,
            [this](){
                if(latest_joint_trajectory_ !=nullptr){
                    auto msg = trajectory_msgs::msg::JointTrajectory(*latest_joint_trajectory_); 
                    jt_publisher_->publish(msg);
                }
            }
        );

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
    waypointPlanning(std::vector<geometry_msgs::msg::Pose> waypoints,std::vector<double> waypoint_velocities,std::vector<double> durations_request){
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
        
        if(durations_request.size()==0){
            for (int i = 0; i < static_cast<int>(waypoints.size()) - 1; ++i) {
                Eigen::Vector3d displacement = waypoints_positions[i + 1] - waypoints_positions[i];
                Eigen::AngleAxisd angular_displacement(waypoints_orientations[i+1]*waypoints_orientations[i].inverse());
                durations.push_back(std::max(displacement.norm() / average_velocity_, angular_displacement.angle()/average_angular_velocity_ ));
            }
        }
        else{
            // shift duration_request on unit to the left, and copy it to durations
            durations_request.erase(durations_request.begin());
            for(int i =0; i < static_cast<int>(waypoints.size()) -1; i++){
                Eigen::Vector3d displacement = waypoints_positions[i + 1] - waypoints_positions[i];
                Eigen::AngleAxisd angular_displacement(waypoints_orientations[i+1]*waypoints_orientations[i].inverse());
                double min_allowable_duration = std::max(displacement.norm() / max_average_velocity_, angular_displacement.angle()/max_average_angular_velocity_ );
                if(durations_request[i] < min_allowable_duration){
                    RCLCPP_WARN(node_->get_logger(),"duration request %d, %.2f was lesser than allowable, modified to %.2f"
                    , i, durations_request[i], min_allowable_duration
                    );
                    durations_request[i] = min_allowable_duration;
                }
            }
            durations = durations_request;
        }

        durations.push_back(0.0);

        // adding this print for debugging
        for (size_t i = 0; i < waypoints_positions.size(); ++i)
        {
            const auto &p = waypoints_positions[i];
            const auto &q = waypoints_orientations[i];
            const auto &v = waypoint_velocity_vectors[i];
        
            double duration = (i < durations.size()) ? durations[i] : 0.0;
        
            RCLCPP_INFO(
                node_->get_logger(),
                "Waypoint %zu | "
                "pos [%.6f %.6f %.6f] | "
                "quat [%.6f %.6f %.6f %.6f] | "
                "vel [%.6f %.6f %.6f] | "
                "duration %.6f",
                i,
                p.x(), p.y(), p.z(),
                q.w(), q.x(), q.y(), q.z(),
                v.x(), v.y(), v.z(),
                duration
            );

            if(i<waypoints_positions.size()-1){
                Eigen::Vector3d displacement = waypoints_positions[i + 1] - waypoints_positions[i];
                Eigen::AngleAxisd angular_displacement(waypoints_orientations[i+1]*waypoints_orientations[i].inverse());
                RCLCPP_INFO(node_->get_logger(),"Linear displacement : %.2f, Angular displacement : %.2f",displacement.norm(),angular_displacement.angle());
            }
        }

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
            RCLCPP_INFO(node_->get_logger(),"Duration : %.2f",duration);
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

    std::shared_ptr<std::vector<TSCubicPolynomialTraj::jointSpaceTrajPoint>> generate_js_traj(
    std::shared_ptr<std::vector<TSCubicPolynomialTraj::trajPoint>>& task_space_trajectory)
    {
        RCLCPP_INFO(node_->get_logger(), "generate_js_traj: entered");

        if (task_space_trajectory == nullptr || task_space_trajectory->empty()) {
            RCLCPP_ERROR(node_->get_logger(), "Task space trajectory is null or empty");
            return nullptr;
        }

        auto js_trajectory =
            std::make_shared<std::vector<TSCubicPolynomialTraj::jointSpaceTrajPoint>>();

        moveit::core::RobotStatePtr robot_state = move_group_interface_->getCurrentState(1.0);
        if (!robot_state) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get current robot state");
            return nullptr;
        }

        const moveit::core::JointModelGroup* joint_model_group =
            robot_state->getJointModelGroup(planning_group_);
        if (!joint_model_group) {
            RCLCPP_ERROR(node_->get_logger(), "Failed to get joint model group: %s", planning_group_.c_str());
            return nullptr;
        }

        std::vector<double> current_joint_values;
        robot_state->copyJointGroupPositions(joint_model_group, current_joint_values);

        if (current_joint_values.size() != 6) {
            RCLCPP_ERROR(
                node_->get_logger(),
                "Expected 6 joints in planning group %s, but got %zu",
                planning_group_.c_str(),
                current_joint_values.size());
            return nullptr;
        }

        constexpr double joint_velocity_limit = 3.0;  
        constexpr double ik_timeout = 0.02;
        constexpr double min_dt = 1e-4;

        auto wrap_to_pi = [](double angle) -> double {
            return std::atan2(std::sin(angle), std::cos(angle));
        };

        auto unwrap_to_nearest = [&](const std::vector<double>& reference,
                                     std::vector<double>& candidate) {
            for (std::size_t j = 0; j < candidate.size(); ++j) {
                candidate[j] = reference[j] + wrap_to_pi(candidate[j] - reference[j]);
            }
        };

        std::vector<double> prev_joint_values = current_joint_values;
        double accumulated_time = 0.0;

        for (std::size_t i = 0; i < task_space_trajectory->size(); ++i) {
            const auto& ts_point = task_space_trajectory->at(i);

            robot_state->setJointGroupPositions(joint_model_group, prev_joint_values);
            robot_state->update();

            bool found_ik = robot_state->setFromIK(
                joint_model_group,
                ts_point.waypoint,
                endeffector_link_,
                ik_timeout);

            if (!found_ik) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "IK failed at trajectory point %zu (t = %.3f)",
                    i,
                    ts_point.duration_from_start);
                return nullptr;
            }

            robot_state->update();

            std::vector<double> joint_positions;
            robot_state->copyJointGroupPositions(joint_model_group, joint_positions);

            if (joint_positions.size() != 6) {
                RCLCPP_ERROR(
                    node_->get_logger(),
                    "IK returned %zu joints instead of 6 at point %zu",
                    joint_positions.size(),
                    i);
                return nullptr;
            }

            unwrap_to_nearest(prev_joint_values, joint_positions);

            TSCubicPolynomialTraj::jointSpaceTrajPoint js_point;

            if (i == 0) {
                js_point.basejoint.position = joint_positions[0];
                js_point.basejoint.velocity = 0.0;

                js_point.shoulderjoint.position = joint_positions[1];
                js_point.shoulderjoint.velocity = 0.0;

                js_point.elbowjoint.position = joint_positions[2];
                js_point.elbowjoint.velocity = 0.0;

                js_point.wrist1.position = joint_positions[3];
                js_point.wrist1.velocity = 0.0;

                js_point.wrist2.position = joint_positions[4];
                js_point.wrist2.velocity = 0.0;

                js_point.wrist3.position = joint_positions[5];
                js_point.wrist3.velocity = 0.0;

                js_point.duration_from_start = 0.0;
                js_trajectory->push_back(js_point);

                prev_joint_values = joint_positions;
                continue;
            }

            double nominal_dt =
                task_space_trajectory->at(i).duration_from_start -
                task_space_trajectory->at(i - 1).duration_from_start;

            if (nominal_dt < min_dt) {
                nominal_dt = min_dt;
            }

            std::vector<double> dq(6, 0.0);
            double max_required_velocity = 0.0;

            for (std::size_t j = 0; j < 6; ++j) {
                dq[j] = joint_positions[j] - prev_joint_values[j];
                double required_velocity = std::abs(dq[j]) / nominal_dt;
                if (required_velocity > max_required_velocity) {
                    max_required_velocity = required_velocity;
                }
            }

            double scale = 1.0;
            if (max_required_velocity > joint_velocity_limit) {
                scale = max_required_velocity / joint_velocity_limit;
                RCLCPP_WARN(
                    node_->get_logger(),
                    "Point %zu exceeds joint velocity limit: max required %.4f rad/s. "
                    "Stretching local dt by %.4f to preserve path.",
                    i,
                    max_required_velocity,
                    scale);
            }

            double actual_dt = nominal_dt * scale;
            accumulated_time += actual_dt;

            std::vector<double> joint_velocities(6, 0.0);
            for (std::size_t j = 0; j < 6; ++j) {
                joint_velocities[j] = dq[j] / actual_dt;
            }

            js_point.basejoint.position = joint_positions[0];
            js_point.basejoint.velocity = joint_velocities[0];

            js_point.shoulderjoint.position = joint_positions[1];
            js_point.shoulderjoint.velocity = joint_velocities[1];

            js_point.elbowjoint.position = joint_positions[2];
            js_point.elbowjoint.velocity = joint_velocities[2];

            js_point.wrist1.position = joint_positions[3];
            js_point.wrist1.velocity = joint_velocities[3];

            js_point.wrist2.position = joint_positions[4];
            js_point.wrist2.velocity = joint_velocities[4];

            js_point.wrist3.position = joint_positions[5];
            js_point.wrist3.velocity = joint_velocities[5];

            js_point.duration_from_start = accumulated_time;
            js_trajectory->push_back(js_point);

            prev_joint_values = joint_positions;
        }

        latest_joint_space_trajectory_ = js_trajectory;
        RCLCPP_INFO(
            node_->get_logger(),
            "generate_js_traj: generated %zu joint-space points successfully",
            js_trajectory->size());

        return js_trajectory;
    }
    
    // external interface to generate the full joint space trajectory
    void generate_trajectory_server_callback_(
        motion_planning_abstractions_msgs::srv::GenerateTrajectory::Request::SharedPtr req, 
        motion_planning_abstractions_msgs::srv::GenerateTrajectory::Response::SharedPtr res
    ){
        std::vector<geometry_msgs::msg::Pose> waypoints(req->waypoints);
        std::vector<double> durations(req->durations);
        waypoint_velocity_ = req->waypoint_speed;
        average_velocity_ = req->average_speed;

        RCLCPP_INFO(node_->get_logger(),"Started generating trajectory");
        
        // get current pose
        move_group_interface_->setStartStateToCurrentState();
        auto current_pose = move_group_interface_->getCurrentPose().pose;
        Eigen::Vector3d current_position(current_pose.position.x,current_pose.position.y,current_pose.position.z);
        Eigen::Quaterniond current_orientation(current_pose.orientation.w,current_pose.orientation.x,current_pose.orientation.y,current_pose.orientation.z);
        
        auto first_waypoint=req->waypoints[0];
        Eigen::Vector3d first_waypoint_position(first_waypoint.position.x,first_waypoint.position.y,first_waypoint.position.z);
        Eigen::Quaterniond first_waypoint_orientation(first_waypoint.orientation.w,first_waypoint.orientation.x,first_waypoint.orientation.y,first_waypoint.orientation.z);
        
        // RCLCPP_INFO(node_->get_logger(),"Current robot position : %.2f, %.2f, %.2f",current_position[0],current_position[1],current_position[2]);
        // RCLCPP_INFO(node_->get_logger(),"First robot position : %.2f, %.2f, %.2f",first_waypoint_position[0],first_waypoint_position[1],first_waypoint_position[2]);
        // RCLCPP_INFO(node_->get_logger(),"Current robot orientation : %.2f, %.2f, %.2f, %.2f",current_orientation.w(),current_orientation.x(),current_orientation.y(),current_orientation.z());
        // RCLCPP_INFO(node_->get_logger(),"First robot orientation : %.2f, %.2f, %.2f, %.2f",first_waypoint_orientation.w(),first_waypoint_orientation.x(),first_waypoint_orientation.y(),first_waypoint_orientation.z());

        double linear_deviation=(first_waypoint_position-current_position).norm();
        double angular_deviation = (Eigen::AngleAxisd(first_waypoint_orientation*current_orientation.inverse())).angle();

        if(durations.size()!=0 && durations.size()!=waypoints.size()){
            RCLCPP_ERROR(node_->get_logger(),"Size mismatch between the durations and the waypoints, invalid request");
            res->fraction = 0;
            res->success = false;
            res->message = "No trajectory generated";
            return;
        }

        if(std::abs(linear_deviation)>1e-3 || std::abs(angular_deviation)>1e-2){
            waypoints.insert(waypoints.begin(),current_pose);
            if(durations.size()!=0.0){
                if(durations.front()==0.0){
                    RCLCPP_ERROR(node_->get_logger(),"The first pose is not the current pose and the the first duration is 0, invalid request");
                    res->fraction = 0;
                    res->success = false;
                    res->message = "No trajectory generated";
                    return;
                }
                durations.insert(durations.begin(),0.0);
            }
        }
        else{
            if(durations.size()!=0.0){
                if(durations.front()!=0.0){
                    RCLCPP_ERROR(node_->get_logger(),"The first pose is the current pose and the first duration is not 0, invalid request");
                    res->fraction = 0;
                    res->success = false;
                    res->message = "No trajectory generated";
                    return;
                }
            }
        }

        std::vector<double> waypoint_velocities;
        for(int i=0;i<waypoints.size();i++){
            if(i==0 || i==waypoints.size()-1)
                waypoint_velocities.push_back(0.0);
            else
                waypoint_velocities.push_back(waypoint_velocity_);
        }

        RCLCPP_INFO(node_->get_logger(),"Wayopints and corresponding velocities");

        // plan a task space path
        auto ts_traj = waypointPlanning(waypoints,waypoint_velocities,durations);
        if(ts_traj==nullptr){
            res->fraction = 0;
            res->message = "task space trajectory generation failed";
            return;
        }
        else{
            RCLCPP_INFO(node_->get_logger(),"Generated task space trajectory successfully");
        }

        latest_trajectory_ = ts_traj;

        // create a js trajectory
        std::shared_ptr<std::vector<TSCubicPolynomialTraj::jointSpaceTrajPoint>>
        js_traj = generate_js_traj(ts_traj);
        if(js_traj == nullptr){
            res->fraction = 0;
            res->success = false;
            res->message = "No trajectory generated";
            return;
        }
        res->trajectory.points.resize(js_traj->size());

        if(js_traj==nullptr){
            res->fraction = 0;
            res->message = "joint space trajectory generation failed";
            return;
        }

        latest_joint_space_trajectory_ = js_traj;

        res->trajectory.header.frame_id = "world";
        res->trajectory.header.stamp = node_->get_clock()->now();
        res->trajectory.joint_names = {
            arm_side + "_shoulder_pan_joint",
            arm_side + "_shoulder_lift_joint",
            arm_side + "_elbow_joint",
            arm_side + "_wrist_1_joint",
            arm_side + "_wrist_2_joint",
            arm_side + "_wrist_3_joint"
        };

        // populate the res->trajectory message here
        for(int i=0;i<js_traj->size();i++){
            res->trajectory.points[i].positions = {
                (*js_traj)[i].basejoint.position,
                (*js_traj)[i].shoulderjoint.position,
                (*js_traj)[i].elbowjoint.position,
                (*js_traj)[i].wrist1.position,
                (*js_traj)[i].wrist2.position,
                (*js_traj)[i].wrist3.position,
            };

            res->trajectory.points[i].velocities = {
                (*js_traj)[i].basejoint.velocity,
                (*js_traj)[i].shoulderjoint.velocity,
                (*js_traj)[i].elbowjoint.velocity,
                (*js_traj)[i].wrist1.velocity,
                (*js_traj)[i].wrist2.velocity,
                (*js_traj)[i].wrist3.velocity,
            };
            
            auto t = (*js_traj)[i].duration_from_start;
            rclcpp::Duration d = rclcpp::Duration::from_seconds(t);
            builtin_interfaces::msg::Duration msg;
            msg.sec = d.seconds();
            msg.nanosec = (d.nanoseconds() % 1000000000);
            
            res->trajectory.points[i].time_from_start=msg;
        }

        latest_joint_trajectory_ = std::make_shared<trajectory_msgs::msg::JointTrajectory>(res->trajectory);

        res->fraction = 100;
        res->success = true;
        res->message = "joint space trajectory generation succeeded";
    }

    
    //////// REPLACING THE SERVER WITH A TRIGGER BECAUSE ITS HARD TO TEST WITH JUST COMMAND LINE, need to change this before using it properly
    // user interface to execute trajectory
    bool execute_trajectory_server_callback_(
    // motion_planning_abstractions_msgs::srv::ExecuteTrajectory::Request::SharedPtr req,
    // motion_planning_abstractions_msgs::srv::ExecuteTrajectory::Response::SharedPtr res)
    )
    {
        using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
        using GoalHandleFollowJointTrajectory = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

        if (!sjtc_client_ptr_->wait_for_action_server(std::chrono::seconds(2))) {
            RCLCPP_ERROR(node_->get_logger(), "SJTC action server not available");
            //////// REPLACING THE SERVER WITH A TRIGGER BECAUSE ITS HARD TO TEST WITH JUST COMMAND LINE, need to change this before using it properly
            // res->success = false;
            // res->message = "SJTC action server not available";
            return false;
        }

        FollowJointTrajectory::Goal sjtc_goal;

        if(latest_joint_trajectory_ ==nullptr){
            RCLCPP_ERROR(node_->get_logger(),"No trajectory generated yet");
            return false;
        }
        //////// REPLACING THE SERVER WITH A TRIGGER BECAUSE ITS HARD TO TEST WITH JUST COMMAND LINE, need to change this before using it properly
        sjtc_goal.trajectory = *latest_joint_trajectory_;
        // sjtc_goal.trajectory = req->trajectory;

        RCLCPP_INFO(node_->get_logger(), "Sending trajectory to SJTC action");

        // INTERFACE WITH THE SJTC controller
        auto send_goal_options =
        rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
        [this](const GoalHandleFollowJointTrajectory::SharedPtr & goal_handle)
        {
            if (!goal_handle) {
                RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by the server");
            } else {
                RCLCPP_INFO(node_->get_logger(), "Goal was accepted by the server");
            }
        };

        sjtc_client_ptr_->async_send_goal(sjtc_goal, send_goal_options);

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

        // std::vector<TSCubicPolynomialTraj::trajPoint> trajectory =  generate_trajectory_(w_in,v_in,w_f,v_f,duration,dt);
        // if(this->latest_trajectory_ == nullptr){
        //     latest_trajectory_ = std::make_shared<std::vector<TSCubicPolynomialTraj::trajPoint>>();
        // }
        // *latest_trajectory_ = trajectory;

        geometry_msgs::msg::Pose wp1,wp2,wp3,wp4;
        // Waypoint 1
        wp1.position.x = 0.289;
        wp1.position.y = 0.766;
        wp1.position.z = 0.697;
        wp1.orientation.x = -0.500;
        wp1.orientation.y = -0.500;
        wp1.orientation.z =  0.500;
        wp1.orientation.w =  0.500;
            
        // Waypoint 2
        wp2.position.x = 0.558;
        wp2.position.y = 0.830;
        wp2.position.z = 0.823;
        wp2.orientation.x = -0.630;
        wp2.orientation.y = -0.321;
        wp2.orientation.z =  0.322;
        wp2.orientation.w =  0.629;
            
        // Waypoint 3
        wp3.position.x = 0.494;
        wp3.position.y = 1.155;
        wp3.position.z = 0.735;
        wp3.orientation.x = -0.627;
        wp3.orientation.y = -0.327;
        wp3.orientation.z =  0.328;
        wp3.orientation.w =  0.626;
            
        // Waypoint 4
        wp4.position.x = 0.316;
        wp4.position.y = 0.899;
        wp4.position.z = 0.862;
        wp4.orientation.x = -0.627;
        wp4.orientation.y = -0.327;
        wp4.orientation.z =  0.328;
        wp4.orientation.w =  0.626;

        std::shared_ptr<std::vector<TSCubicPolynomialTraj::trajPoint>> trajectory = waypointPlanning(std::vector<geometry_msgs::msg::Pose>{wp1,wp2,wp3,wp4},std::vector<double>{0.0,0.1,0.2,0.0},std::vector<double>{2,2,2,2});
        latest_trajectory_ = trajectory;
        std::shared_ptr<std::vector<TSCubicPolynomialTraj::jointSpaceTrajPoint>> js_traj = generate_js_traj(trajectory);

        // do_ik([](){
        //     geometry_msgs::msg::Pose pose;
        //     pose.position.x=0.1;
        //     pose.position.y=0.4;
        //     pose.position.z=0.1;
        //     pose.orientation.w=1.0;
        //     return pose;
        // }());

        return true;
    }

    bool print_latest_trajectory_server_callback_(){
        if (latest_trajectory_==nullptr)
            return false;

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

    bool print_latest_joint_space_trajectory_server_callback_(){
        if(latest_joint_space_trajectory_ ==nullptr)
            return false;

        for(TSCubicPolynomialTraj::jointSpaceTrajPoint point :*latest_joint_space_trajectory_){
            RCLCPP_INFO(
                node_->get_logger(),
                "t=%.3f | "
                "Joint 1 [%.4f %.4f] | "
                "Joint 2 [%.4f %.4f] | "
                "Joint 3 [%.4f %.4f] | "
                "Joint 4 [%.4f %.4f] | "
                "Joint 5 [%.4f %.4f] | "
                "Joint 6 [%.4f %.4f]",
                point.duration_from_start,
                point.basejoint.position, point.basejoint.velocity,
                point.shoulderjoint.position, point.shoulderjoint.velocity,
                point.elbowjoint.position, point.elbowjoint.velocity,
                point.wrist1.position, point.wrist1.velocity,
                point.wrist2.position, point.wrist2.velocity,
                point.wrist3.position, point.wrist3.velocity
            );
        }
        RCLCPP_INFO(node_->get_logger(),"Size of the trajectory message : %d",latest_trajectory_->size());
        return true;
    }

    // given a pose, does ik
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

        // We can also get the Jacobian from the :moveit_codedir:`RobotState<moveit_core/robot_state/include/moveit/robot_state/robot_state.h>`.
        Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
        Eigen::MatrixXd jacobian;
        current_robot_state_->getJacobian(joint_group_model_,
                                     current_robot_state_->getLinkModel(joint_group_model_->getLinkModelNames().back()),
                                     reference_point_position, jacobian);
        RCLCPP_INFO_STREAM(node_->get_logger(), "Jacobian: \n" << jacobian << "\n");
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
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_latest_trajectory_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr print_latest_joint_space_trajectory_server_;
    rclcpp::Service<motion_planning_abstractions_msgs::srv::GenerateTrajectory>::SharedPtr generate_trajectory_server_;
    
    //////// REPLACING THE SERVER WITH A TRIGGER BECAUSE ITS HARD TO TEST WITH JUST COMMAND LINE, need to change this before using it properly
    // rclcpp::Service<motion_planning_abstractions_msgs::srv::ExecuteTrajectory>::SharedPtr execute_trajectory_server_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr execute_trajectory_server_;

    // clients

    // publishers
    rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr jt_publisher_;

    // subscribers
    
    // action_clients
    rclcpp_action::Client<control_msgs::action::FollowJointTrajectory>::SharedPtr sjtc_client_ptr_;

    // timers
    rclcpp::TimerBase::SharedPtr latest_jt_publisher_timer_;

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
    std::shared_ptr<std::vector<TSCubicPolynomialTraj::jointSpaceTrajPoint>> latest_joint_space_trajectory_;
    std::shared_ptr<trajectory_msgs::msg::JointTrajectory> latest_joint_trajectory_;
    double average_velocity_=0.3; // change this to make pt to pt traj faster or slower by making this bigger or smaller
    double average_angular_velocity_=M_PI/6; // change this to make pt to pt traj faster or slower by making this bigger or smaller
    double max_average_velocity_=0.5; // use this to change behaviour of time deterministic planning
    double max_average_angular_velocity_=M_PI/3; // use this to change behaviour of time deterministic planning
    double waypoint_velocity_ = 0.05; // change this to make the robot slower or faster at waypoints
    double dt_=0.05; //trajectory interval
    
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto moveit_example = TSCubicPolynomialTraj();
    rclcpp::shutdown();
    return 0;
}
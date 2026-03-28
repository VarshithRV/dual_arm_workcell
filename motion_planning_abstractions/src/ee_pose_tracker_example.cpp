#include <memory>
#include <functional>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "motion_planning_abstractions/ee_pose_tracker.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("ee_pose_tracker_example");
    auto single_arm_control_interface = std::make_shared<BareBonesMoveit>(node);
    auto pose_tracker_interface = std::make_shared<PoseTracker>(node);

    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto clock = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);

    if(node==nullptr){
        std::cout<<"Bad Node initialization"<<std::endl;
    }
    if(single_arm_control_interface==nullptr){
        RCLCPP_INFO(node->get_logger(),"Bad single arm control interface initialization");
    }
    if(pose_tracker_interface==nullptr){
        RCLCPP_INFO(node->get_logger(),"Bad pose tracker interface initialization");
    }

    RCLCPP_INFO(node->get_logger(),"Created the pose tracker interface and the single arm control interface and the callback group");

    auto get_waypoint = [node,single_arm_control_interface](double t){
            auto current_pose = single_arm_control_interface->get_current_ee_pose();
            current_pose->position.x += 0.4*sin(0.5*t);
            return *current_pose;
    };

    auto execute_service = node->create_service<std_srvs::srv::Trigger>(
        "~/execute",
        [node,single_arm_control_interface,pose_tracker_interface,clock,get_waypoint](
            std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr res
        ){
            std::cout<<"clearing target"<<std::endl;
            pose_tracker_interface->clear_target_pose_();
            std::cout<<"preparing tracker"<<std::endl;
            pose_tracker_interface->prepare_tracker_();
            std::cout<<"setting target pose"<<std::endl;
            pose_tracker_interface->set_target_pose_(*(single_arm_control_interface->get_current_ee_pose()));
            std::cout<<"starting to track"<<std::endl;
            pose_tracker_interface->start_tracking_();
            std::chrono::duration current_time = std::chrono::duration<double,std::ratio<1>>(clock->now().seconds());
            auto duration = std::chrono::duration<double,std::ratio<1>>(0.0);
            while(rclcpp::ok() && duration<30s){
                duration = std::chrono::duration<double,std::ratio<1>>(clock->now().seconds())-current_time;
                pose_tracker_interface->set_target_pose_(get_waypoint(duration.count()));
            }
            std::cout<<"stopping tracking"<<std::endl;
            pose_tracker_interface->stop_tracking_();
            std::cout<<"unprepare tracker"<<std::endl;
            pose_tracker_interface->unprepare_tracker_();
            std::cout<<"done"<<std::endl;
        },
        rmw_qos_profile_services_default,
        callback_group
    );

    if(execute_service==nullptr){
        RCLCPP_INFO(node->get_logger(),"Bad server initialization");
    }
    else{
        RCLCPP_INFO(node->get_logger(),"All good");
    }

    RCLCPP_INFO(node->get_logger(),"Inializaing executor");
    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    RCLCPP_INFO(node->get_logger(),"Adding node");
    executor->add_node(node);
    RCLCPP_INFO(node->get_logger(),"Spinning now");
    executor->spin();

    rclcpp::shutdown();

}
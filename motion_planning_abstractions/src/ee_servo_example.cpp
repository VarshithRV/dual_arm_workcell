#include <functional>
#include <memory>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"

using namespace std::chrono_literals;

int main(int argc, const char** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("ee_servo_example");
    auto ee_servo_handle = std::make_shared<EEServo>(node);
    auto cb_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

    auto execute_server = node->create_service<std_srvs::srv::Trigger>(
        "~/execute",
        [node,ee_servo_handle](std_srvs::srv::Trigger::Request::SharedPtr req, std_srvs::srv::Trigger::Response::SharedPtr res){
            std::cout<<"preparing servo"<<std::endl;
            ee_servo_handle->prepare_servo_();
            std::cout<<"starting servo"<<std::endl;
            ee_servo_handle->start_servo_();
            std::cout<<"ready to move"<<std::endl;

            auto rate = rclcpp::Rate(50ms);
            
            int i = 0;
            while(rclcpp::ok()){
                i++;
                ee_servo_handle->set_vel_setpoint_(
                    [node](){
                        auto vel = geometry_msgs::msg::TwistStamped();
                        vel.header.frame_id="world";
                        vel.header.stamp = node->get_clock()->now();
                        vel.twist.linear.x = 0.1;
                        return vel;
                    }()
                );
                rate.sleep();
                if(i>100){
                    break;
                }
            }

            std::cout<<"Stopping servo now"<<std::endl;
            ee_servo_handle->stop_servo_();
            std::cout<<"Unpreparing now"<<std::endl;
            ee_servo_handle->unprepare_servo_();
            std::cout<<"Done"<<std::endl;
        },
        rmw_qos_profile_services_default,
        cb_group
    );

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
    
    rclcpp::shutdown();
}
#include <memory>
#include <functional>
#include <chrono>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "motion_planning_abstractions/bare_bones_moveit.hpp"
#include "motion_planning_abstractions/ee_servo.hpp"
#include "motion_planning_abstractions/ee_pose_tracker.hpp"

using namespace std::chrono_literals;

int main(int argc, char ** argv){
    rclcpp::init(argc,argv);
    auto node = std::make_shared<rclcpp::Node>("ee_pose_tracker_example");
    auto single_arm_control_interface = std::make_shared<BareBonesMoveit>(node);
    auto pose_tracker_interface = std::make_shared<PoseTracker>(node);

    auto callback_group = node->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    auto execute_service = node->create_service<std_srvs::srv::Trigger>(
        "~/execute",
        [node,callback_group,single_arm_control_interface,pose_tracker_interface](
            std_srvs::srv::Trigger::Request::SharedPtr,
            std_srvs::srv::Trigger::Response::SharedPtr res
        ){
            pose_tracker_interface->clear_target_pose_();
            pose_tracker_interface->prepare_tracker_();
            pose_tracker_interface->set_target_pose_(*single_arm_control_interface->get_current_ee_pose());
            pose_tracker_interface->start_tracking_();
            pose_tracker_interface->set_target_pose_(
                [single_arm_control_interface](){
                    auto current_pose = single_arm_control_interface->get_current_ee_pose();
                    current_pose->position.x  += 0.05;
                    return *current_pose;
                }()
            );
            std::this_thread::sleep_for(3s);
            pose_tracker_interface->stop_tracking_();
            pose_tracker_interface->unprepare_tracker_();
        }
    );

    auto executor = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
    executor->add_node(node);
    executor->spin();
}
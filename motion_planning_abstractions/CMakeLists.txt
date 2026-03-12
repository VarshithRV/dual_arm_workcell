cmake_minimum_required(VERSION 3.8)
project(motion_planning_abstractions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PACKAGE_INCLUDE_DEPENDS
  controller_manager_msgs
  control_msgs
  control_toolbox
  geometry_msgs
  motion_planning_abstractions_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_servo
  open_set_object_detection_msgs
  pluginlib
  rclcpp
  rclcpp_action
  rclcpp_components
  sensor_msgs
  std_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_geometry_msgs
  trajectory_msgs
  ur_msgs
  rosidl_runtime_cpp
  trajectory_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(motion_planning_abstractions_msgs REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_servo REQUIRED)
find_package(open_set_object_detection_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(rosidl_runtime_cpp REQUIRED)
find_package(trajectory_msgs REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)


install(
  DIRECTORY launch 
  DESTINATION share/${PROJECT_NAME}
)

add_executable(predefined_state_server src/predefined_state_server.cpp)
ament_target_dependencies(predefined_state_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
predefined_state_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_server src/pick_and_place_server.cpp)
ament_target_dependencies(pick_and_place_server  rclcpp geometry_msgs moveit_ros_planning_interface moveit_core 
motion_planning_abstractions_msgs ur_msgs open_set_object_detection_msgs)
install(TARGETS
pick_and_place_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_local_perception_server src/pick_and_place_local_perception_server.cpp)
ament_target_dependencies(pick_and_place_local_perception_server  rclcpp geometry_msgs moveit_ros_planning_interface moveit_core 
motion_planning_abstractions_msgs ur_msgs open_set_object_detection_msgs tf2 tf2_geometry_msgs)
install(TARGETS
pick_and_place_local_perception_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(task_space_cubic_polynomial_trajectory_server src/task_space_cubic_polynomial_trajectory_server.cpp)
ament_target_dependencies(task_space_cubic_polynomial_trajectory_server ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
task_space_cubic_polynomial_trajectory_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_ft_feedback_server src/pick_and_place_ft_feedback_server.cpp)
ament_target_dependencies(pick_and_place_ft_feedback_server  rclcpp geometry_msgs moveit_ros_planning_interface 
moveit_core motion_planning_abstractions_msgs ur_msgs controller_manager_msgs std_srvs open_set_object_detection_msgs tf2 tf2_geometry_msgs)
install(TARGETS
pick_and_place_ft_feedback_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pose_tracker src/pose_tracker.cpp)
ament_target_dependencies(pose_tracker ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
pose_tracker
DESTINATION lib/${PROJECT_NAME}
)

add_executable(bare_bones_moveit src/bare_bones_moveit.cpp)
ament_target_dependencies(bare_bones_moveit ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
bare_bones_moveit
DESTINATION lib/${PROJECT_NAME}
)

add_executable(dual_arm_control_template src/dual_arm_control_template.cpp)
ament_target_dependencies(dual_arm_control_template ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
dual_arm_control_template
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

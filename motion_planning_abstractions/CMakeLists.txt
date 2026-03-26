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

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
install(
  DIRECTORY include/
  DESTINATION include
)
include_directories(${CMAKE_CURRENT_SOURCE_DIR}/include)


add_library(bare_bones_moveit SHARED
  src/bare_bones_moveit.cpp
)
target_include_directories(bare_bones_moveit
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(bare_bones_moveit
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
install(
  TARGETS bare_bones_moveit
  EXPORT export_bare_bones_moveit
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_bare_bones_moveit HAS_LIBRARY_TARGET)


add_library(ee_servo SHARED
  src/ee_servo.cpp
)
target_include_directories(ee_servo
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(ee_servo
  ${THIS_PACKAGE_INCLUDE_DEPENDS}
)
install(
  TARGETS ee_servo
  EXPORT export_ee_servo
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
ament_export_targets(export_ee_servo HAS_LIBRARY_TARGET)

ament_export_include_directories(include)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})

add_executable(bare_bones_moveit_example src/bare_bones_moveit_example.cpp)
ament_target_dependencies(bare_bones_moveit_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(bare_bones_moveit_example bare_bones_moveit)
target_include_directories(bare_bones_moveit_example PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
bare_bones_moveit_example
DESTINATION lib/${PROJECT_NAME}
)

add_executable(ee_servo_example src/ee_servo_example.cpp)
ament_target_dependencies(ee_servo_example ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_link_libraries(ee_servo_example ee_servo)
target_include_directories(ee_servo_example PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
ee_servo_example
DESTINATION lib/${PROJECT_NAME}
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

add_executable(pose_tracker_node src/pose_tracker_node.cpp)
ament_target_dependencies(pose_tracker_node ${THIS_PACKAGE_INCLUDE_DEPENDS})
install(TARGETS
pose_tracker_node
DESTINATION lib/${PROJECT_NAME}
)

add_executable(dual_arm_control_template src/dual_arm_control_template.cpp)
ament_target_dependencies(dual_arm_control_template ${THIS_PACKAGE_INCLUDE_DEPENDS})
target_include_directories(dual_arm_control_template PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
install(TARGETS
dual_arm_control_template
DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

cmake_minimum_required(VERSION 3.8)
project(motion_planning_abstractions)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(motion_planning_abstractions_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ur_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(open_set_object_detection_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

add_executable(predefined_state_server src/predefined_state_server.cpp)
ament_target_dependencies(predefined_state_server example_interfaces rclcpp 
geometry_msgs moveit_ros_planning_interface moveit_core)
install(TARGETS
predefined_state_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_server src/pick_and_place_server.cpp)
ament_target_dependencies(pick_and_place_server example_interfaces rclcpp geometry_msgs moveit_ros_planning_interface moveit_core 
motion_planning_abstractions_msgs ur_msgs open_set_object_detection_msgs)
install(TARGETS
pick_and_place_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_local_perception_server src/pick_and_place_local_perception_server.cpp)
ament_target_dependencies(pick_and_place_local_perception_server example_interfaces rclcpp geometry_msgs moveit_ros_planning_interface moveit_core 
motion_planning_abstractions_msgs ur_msgs open_set_object_detection_msgs tf2 tf2_geometry_msgs)
install(TARGETS
pick_and_place_local_perception_server
DESTINATION lib/${PROJECT_NAME}
)

add_executable(pick_and_place_ft_feedback_server src/pick_and_place_ft_feedback_server.cpp)
ament_target_dependencies(pick_and_place_ft_feedback_server example_interfaces rclcpp geometry_msgs moveit_ros_planning_interface 
moveit_core motion_planning_abstractions_msgs ur_msgs controller_manager_msgs std_srvs open_set_object_detection_msgs tf2 tf2_geometry_msgs)
install(TARGETS
pick_and_place_ft_feedback_server
DESTINATION lib/${PROJECT_NAME}
)

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

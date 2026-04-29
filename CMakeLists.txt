cmake_minimum_required(VERSION 3.8)
project(fer_skills)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/GoHome.action"
  "action/MoveToPose.action"
  "action/PickObject.action"
  DEPENDENCIES geometry_msgs action_msgs
)

set(SKILL_SERVER_DEPS
  rclcpp
  rclcpp_action
  rclcpp_components
  geometry_msgs
  action_msgs
  control_msgs
  moveit_ros_planning_interface
)

add_executable(skill_server_node
  src/skill_server_node.cpp
  src/skills/go_home.cpp
  src/skills/move_to_pose.cpp
  src/skills/pick_object.cpp
)

target_include_directories(skill_server_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(skill_server_node ${SKILL_SERVER_DEPS})

# Link generated action interfaces (rosidl_target_interfaces is the rolling/humble-compatible API)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(skill_server_node "${cpp_typesupport_target}")

install(TARGETS skill_server_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

install(DIRECTORY include/
  DESTINATION include
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

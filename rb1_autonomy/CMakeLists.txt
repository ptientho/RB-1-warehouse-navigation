cmake_minimum_required(VERSION 3.8)
project(rb1_autonomy)

#if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#  add_compile_options(-Wall -Wextra -Wpedantic)
#endif()
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(behaviortree_ros2 REQUIRED)
find_package(btcpp_ros2_interfaces REQUIRED)
find_package(shelf_detect_msg REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(behaviortree_cpp REQUIRED)
find_package(shelf_attach_msg REQUIRED)
find_package(shelf_detach_msg REQUIRED)

# install directories
install(DIRECTORY

	config
	launch
	DESTINATION share/${PROJECT_NAME}
)

# set variable
set(NAV_SOURCES src/navigation_behavior.cpp)
set(SHELF_SOURCES src/shelf_detection_behavior.cpp src/shelf_attach_behavior.cpp src/shelf_detach_behavior.cpp)
set(AUTONOMY_SOURCE src/autonomy.cpp src/backup_behavior.cpp)

set(TARGET_DEPENDS

	rclcpp
	rclcpp_action
	nav2_msgs
	yaml-cpp
	tf2
	tf2_geometry_msgs
	shelf_detect_msg
	geometry_msgs
	behaviortree_ros2
	btcpp_ros2_interfaces
	shelf_attach_msg
	shelf_detach_msg
	behaviortree_cpp
)

# include headers
include_directories(include)

add_executable(autonomy_node src/main.cpp ${AUTONOMY_SOURCE} ${NAV_SOURCES} ${SHELF_SOURCES})
ament_target_dependencies(autonomy_node ${TARGET_DEPENDS})
target_link_libraries(autonomy_node ${YAML_CPP_LIBRARIES})

install(TARGETS
	autonomy_node
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

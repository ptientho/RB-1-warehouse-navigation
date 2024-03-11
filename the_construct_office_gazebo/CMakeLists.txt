cmake_minimum_required(VERSION 3.8)
project(the_construct_office_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(gazebo_ros REQUIRED)
# find_package(rmf_building_map_tools REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter, which checks for copyrights
  # uncomment the line when copyright and license are not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a Git repo)
  # uncomment the line when this package is not in a Git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

install(DIRECTORY
  launch
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS

  DESTINATION lib/${PROJECT_NAME}
)
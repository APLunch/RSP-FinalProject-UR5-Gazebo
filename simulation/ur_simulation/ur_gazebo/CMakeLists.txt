cmake_minimum_required(VERSION 3.8)
project(ur_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)


# install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
# install world files
install(DIRECTORY world
  DESTINATION share/${PROJECT_NAME}
)
# install urdf files
install(DIRECTORY urdf
  DESTINATION share/${PROJECT_NAME}
)
# install rviz files
install(DIRECTORY rviz
  DESTINATION share/${PROJECT_NAME}
)
# install meshes files
install(DIRECTORY meshes
  DESTINATION share/${PROJECT_NAME}
)
#install config files
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Declare executable
add_executable(joint_state_mirror src/joint_state_mirror.cpp)
ament_target_dependencies(joint_state_mirror rclcpp sensor_msgs)
#install executable
install(TARGETS joint_state_mirror
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()

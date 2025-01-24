cmake_minimum_required(VERSION 3.8)
project(connector)

# 设置 C++ 标准
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(robot_msg REQUIRED)
find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR} include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(connector_dependencies
  rclcpp
  robot_msg
)

add_executable(test_motor
  test/test_motor_ros2.cpp)
target_include_directories(test_motor PRIVATE include)
ament_target_dependencies(test_motor ${connector_dependencies})

add_executable(test_3508
  test/test_3508.cpp)
target_include_directories(test_3508 PRIVATE include)
ament_target_dependencies(test_3508 ${connector_dependencies})

add_executable(test_tty
  test/test_tty.cpp)
target_include_directories(test_tty PRIVATE include)
ament_target_dependencies(test_tty ${connector_dependencies})

add_executable(test_mpc
  test/test_mpc.cpp)
target_include_directories(test_mpc PRIVATE include)
ament_target_dependencies(test_mpc ${connector_dependencies})

add_executable(test_param_dynamic_config
  test/test_param_dynamic_config.cpp)
target_include_directories(test_param_dynamic_config PRIVATE include)
ament_target_dependencies(test_param_dynamic_config ${connector_dependencies})

add_executable(test_inter_pubsub
  test/test_inter_pubsub.cpp)
target_include_directories(test_inter_pubsub PRIVATE include)
ament_target_dependencies(test_inter_pubsub ${connector_dependencies})

add_executable(omni3
  src/apps/omni3.cpp)
target_include_directories(omni3 PRIVATE include ${CMAKE_CURRENT_SOURCE_DIR})
ament_target_dependencies(omni3 ${connector_dependencies})

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

install (TARGETS
  omni3
  test_motor
  test_3508
  test_tty
  test_mpc
  test_param_dynamic_config
  test_inter_pubsub
DESTINATION lib/${PROJECT_NAME})


ament_package()

cmake_minimum_required(VERSION 3.8)
project(embedded_interface)

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
find_package(geometry_msgs REQUIRED)
find_package(robot_msg REQUIRED)
find_package(Eigen3 REQUIRED)
# include_directories(${EIGEN3_INCLUDE_DIR} include)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

set(connector_dependencies
  rclcpp
  robot_msg
  geometry_msgs
)

set(EMBEDDED_INTERFACE_INCLUDE_DIRS include ${EIGEN3_INCLUDE_DIR})
# include_directories(${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_export_include_directories(${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_export_dependencies(rclcpp robot_msg Eigen3)

add_executable(test_motor
  test/test_motor_ros2.cpp)
target_include_directories(test_motor PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_motor ${connector_dependencies})

add_executable(test_3508
  test/test_3508.cpp)
target_include_directories(test_3508 PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_3508 ${connector_dependencies})

add_executable(test_tty
  test/test_tty.cpp)
target_include_directories(test_tty PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_tty ${connector_dependencies})

add_executable(test_mpc
  test/test_mpc.cpp)
target_include_directories(test_mpc PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_mpc ${connector_dependencies})

add_executable(test_param_dynamic_config
  test/test_param_dynamic_config.cpp)
target_include_directories(test_param_dynamic_config PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_param_dynamic_config ${connector_dependencies})

add_executable(test_inter_pubsub
  test/test_inter_pubsub.cpp)
target_include_directories(test_inter_pubsub PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_inter_pubsub ${connector_dependencies})

add_executable(test_any_pub
  test/test_any_pub.cpp)
target_include_directories(test_any_pub PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_any_pub ${connector_dependencies})

add_executable(test_param_vector
  test/test_param_vector.cpp)
target_include_directories(test_param_vector PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_param_vector ${connector_dependencies})

add_executable(lib_basic_test
  test/lib_basic_test.cpp)
target_include_directories(lib_basic_test PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(lib_basic_test ${connector_dependencies})

add_executable(test_observer
  test/test_observer.cpp)
target_include_directories(test_observer PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS})
ament_target_dependencies(test_observer ${connector_dependencies})

add_executable(test_c20
  test/test_c20.cpp)
target_include_directories(test_c20 PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS} )
ament_target_dependencies(test_c20 ${connector_dependencies})


add_executable(omni3
  src/apps/omni3.cpp)
target_include_directories(omni3 PRIVATE ${EMBEDDED_INTERFACE_INCLUDE_DIRS} )
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
  test_any_pub
  test_param_vector
  lib_basic_test
  test_observer
  test_c20
DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

ament_package()

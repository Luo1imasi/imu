cmake_minimum_required(VERSION 3.5)
project(hipnuc_imu)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hipnuc_lib_package REQUIRED)
find_package(rclcpp_components REQUIRED)

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)

  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  # set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# 包含目录
include_directories(include)

# 统一的 IMU 节点（支持 serialhipnuc_imu_node PROPERTIES
BUILD_WITH_INSTALL_RPATHTRUE

INSTALL_RPATH"${CMAKE_INSTALL_PREFIX}/lib"
)

# 订阅测试节点
add_executable(listener src/sub_spec.cpp)
ament_target_dependencies(listener rclcpp std_msgs sensor_msgs rclcpp_components)

install(TARGETS hipnuc_imu_node listener DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include

  # 添加 pthread 链接
  target_link_libraries(hipnuc_imu_node pthread)

  set_target_properties(imu_node PROPERTIES
  BUILD_WITH_INSTALL_RPATH TRUE
  INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
  )

  add_executable(listener src/sub_spec.cpp)
  ament_target_dependencies(listener rclcpp std_msgs sensor_msgs rclcpp_components)

  install(TARGETS imu_node listener DESTINATION lib/${PROJECT_NAME})
  install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
  install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  )

  ament_package()

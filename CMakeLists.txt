cmake_minimum_required(VERSION 3.5)
project(mpu9250driver)

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

# Add internal libs folder
add_subdirectory(lib)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(${PROJECT_NAME} 
  src/mpu9250driver.cpp
)

add_executable(mpu9250_custom_imu_publisher 
  src/mpu9250_custom_imu_publisher.cpp
)

add_executable(odom_tracker_node 
  src/odom_tracker_node.cpp
)

ament_target_dependencies(odom_tracker_node
  rclcpp
  nav_msgs
  std_srvs
)

ament_target_dependencies(${PROJECT_NAME} 
  rclcpp 
  sensor_msgs
)

ament_target_dependencies(mpu9250_custom_imu_publisher 
  rclcpp 
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME} mpu9250sensor linux_i2c_communicator)
target_include_directories(
  ${PROJECT_NAME} PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
                       $<INSTALL_INTERFACE:include>)

install(TARGETS ${PROJECT_NAME} mpu9250_custom_imu_publisher odom_tracker_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME}/)
ament_package()

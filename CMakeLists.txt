cmake_minimum_required(VERSION 3.5)
project(axolotl)

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
find_package(rmf_fleet_adapter REQUIRED)
# find_package(rmf_task_msgs REQUIRED)
# find_package(rmf_traffic_ros2 REQUIRED)
# find_package(rmf_traffic REQUIRED)
# find_package(rmf_battery REQUIRED)
# find_package(Eigen3 REQUIRED)
# find_package(unordered_set)

add_executable(barkin src/main.cpp)
# target_include_directories(barkin PUBLIC /home/jh/rmf_ws/install/rmf_fleet_adapter/include)

# Apparently CMake has namespace, so will have to check what the actual name is used in the proect() CMakeLists is. 
# I.e. find_package(Eigen3) uses Eigen3::Eigen namespace, where in the root folder's CMakeLists uses project(Eigen)
target_link_libraries(barkin 
  rclcpp::rclcpp  
  rmf_fleet_adapter::rmf_fleet_adapter
  # rmf_task_msgs::rmf_task_msgs
  # rmf_traffic_ros2::rmf_traffic_ros2
  # rmf_traffic::rmf_traffic
  # rmf_battery::rmf_battery
  # Eigen3::Eigen
  # unordered_set::unordered_set
  )

# ament_target_dependencies(barkin
#   rclcpp
#   rmf_fleet_adapter
# )

install(TARGETS
  barkin
  DESTINATION lib/${PROJECT_NAME})
ament_package()

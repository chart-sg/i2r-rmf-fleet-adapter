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
find_package(Threads REQUIRED)
# find_package(rclcpp REQUIRED)
find_package(rmf_fleet_adapter REQUIRED)
# find_package(geometry_msgs REQUIRED)

########################
# main.cpp
add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/i2r_driver/i2r_driver.cpp)

# Apparently CMake has namespace, so will have to check what the actual name is used in the proect() CMakeLists is. 
# I.e. find_package(Eigen3) uses Eigen3::Eigen namespace, where in the root folder's CMakeLists uses project(Eigen)

target_link_libraries(${CMAKE_PROJECT_NAME}   
  rmf_fleet_adapter::rmf_fleet_adapter)

target_include_directories(${CMAKE_PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
########################


########################
# client (TODO: Make it into a header libary. But for now just keep as is for testing)
add_executable(${CMAKE_PROJECT_NAME}_client src/client.cpp)
  
# This part shouldnt be neccessary if apt install asio worked properly
# target_include_directories(${CMAKE_PROJECT_NAME}_server 
#   PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include/websocketpp
#   PUBLIC /usr/include/asio
#   )

target_link_libraries(${CMAKE_PROJECT_NAME}_client PUBLIC 
  Threads::Threads
  crypto
  ssl
  )

########################

install(TARGETS
  ${CMAKE_PROJECT_NAME}_client
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  ${CMAKE_PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  
ament_package()

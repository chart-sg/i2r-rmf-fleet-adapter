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
find_library(jsoncpp /lib/x86_64-linux-gnu)
find_package(rmf_fleet_adapter REQUIRED)

########################
# main.cpp
add_executable(${CMAKE_PROJECT_NAME} src/main.cpp src/i2r_driver/mission_gen.cpp src/i2r_driver/i2r_driver.cpp)

target_link_libraries(${CMAKE_PROJECT_NAME}   
  rmf_fleet_adapter::rmf_fleet_adapter
  jsoncpp
  Threads::Threads
  crypto
  ssl
  )

target_include_directories(${CMAKE_PROJECT_NAME}
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

########################
# client (TODO: Make it into a header libary. But for now just keep as is for testing)
add_executable(${CMAKE_PROJECT_NAME}_client src/client.cpp src/i2r_driver/mission_gen.cpp src/i2r_driver/feedback_parser.cpp)
  
target_link_libraries(${CMAKE_PROJECT_NAME}_client PUBLIC 
  rmf_fleet_adapter::rmf_fleet_adapter
  jsoncpp
  Threads::Threads
  crypto
  ssl
  )

target_include_directories(${CMAKE_PROJECT_NAME}_client
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)

#######################
install(TARGETS
  ${CMAKE_PROJECT_NAME}_client
  DESTINATION lib/${PROJECT_NAME})

install(
  TARGETS
  ${CMAKE_PROJECT_NAME}
  DESTINATION lib/${PROJECT_NAME})

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
  )
  
ament_package()

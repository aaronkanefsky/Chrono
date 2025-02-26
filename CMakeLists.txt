cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

project(Chrono_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(Chrono_DIR "/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/cmake")
set(PROJ_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/inc)

find_package(Chrono REQUIRED COMPONENTS Irrlicht ROS OPTIONAL_COMPONENTS CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

if (NOT Chrono_FOUND)
  message(FATAL_ERROR "Could not find Chrono or one of its required modules")
endif()

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

link_directories(/opt/ros/galactic/lib)

include_directories(
  ${CHRONO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  /home/aaron/Documents/acsl-chrono-simulator/libraries/chrono/src
  /opt/ros/galactic/include
  ${PROJ_INCLUDE_DIR}  # Include the 'inc' directory
)

add_executable(custom_handler main.cpp src/CustomHandler.cpp)  # Add your custom cpp file here

ament_target_dependencies(custom_handler rclcpp std_msgs)

target_compile_definitions(custom_handler PRIVATE "CHRONO_DATA_DIR=\"${Chrono_DIR}\"")

# Windows
if(MSVC)
    set_target_properties(custom_handler PROPERTIES MSVC_RUNTIME_LIBRARY ${CHRONO_MSVC_RUNTIME_LIBRARY})
endif()

link_directories(/opt/ros/galactic/lib)

target_link_libraries(custom_handler
  rclcpp
  rcl
  tracetools
  rmw
  rcutils
)

target_link_libraries(custom_handler
    /home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/lib/libChronoEngine.so
    /home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/lib/libChronoEngine_ros.so
)
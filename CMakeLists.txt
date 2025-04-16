cmake_minimum_required(VERSION 3.18)
cmake_policy(SET CMP0091 NEW)

project(Chrono_Project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

set(Chrono_DIR "/home/aaron/Documents/acsl-chrono-simulator/libraries/chrono-build/cmake")
set(PROJ_INCLUDE_DIR ${CMAKE_SOURCE_DIR}/inc)
set(MESSAGE_INCLUDE_DIR "/home/aaron/px4_msgs-build/install/px4_msgs")
# set(CUSTOM_MESSAGE_INCLUDE_DIR "/home/aaron/CustomMessage/tutorial_interfaces/install")

# Set the CMake prefix path to include the directory where px4_msgsConfig.cmake is located
set(CMAKE_PREFIX_PATH "/home/aaron/px4_msgs-build/install/px4_msgs/share:$ENV{CMAKE_PREFIX_PATH}")
set(px4_msgs_DIR "/home/aaron/px4_msgs-build/install/px4_msgs/share/px4_msgs/cmake")
# set(tutorial_interfaces_DIR "/home/aaron/CustomMessage/tutorial_interfaces/install/tutorial_interfaces/share/tutorial_interfaces/cmake")

# Ensure px4_msgs is found by finding its config
find_package(px4_msgs REQUIRED)

find_package(Chrono REQUIRED COMPONENTS Irrlicht ROS OPTIONAL_COMPONENTS CONFIG REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
# find_package(tutorial_interfaces REQUIRED)

if (NOT Chrono_FOUND)
  message(FATAL_ERROR "Could not find Chrono or one of its required modules")
endif()

link_directories(/opt/ros/galactic/lib)

include_directories(
  ${CHRONO_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  /home/aaron/Documents/acsl-chrono-simulator/libraries/chrono/src
  /opt/ros/galactic/include
  ${PROJ_INCLUDE_DIR}  # Include the 'inc' directory
  ${MESSAGE_INCLUDE_DIR}/include/px4_msgs/msg  # Ensure px4_msgs is correctly included
  # /home/aaron/CustomMessage/tutorial_interfaces/install/tutorial_interfaces/include  # Include the tutorial_interfaces package
  # ${tutorial_interfaces_INCLUDE_DIRS}  # Ensure this is set)
)
add_executable(custom_handler main.cpp src/CustomHandler.cpp)  # Add your custom cpp file here

ament_target_dependencies(custom_handler rclcpp std_msgs px4_msgs)

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

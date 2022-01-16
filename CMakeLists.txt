cmake_minimum_required(VERSION 2.8.3)
project(atreus)

# Set target C++ standard and required compiler features
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
    roscpp
    gazebo_ros
    sensor_msgs
    pcl_conversions
    pcl_ros

)
find_package(Boost REQUIRED COMPONENTS thread)
find_package(Threads REQUIRED)

add_service_files(
    FILES
    Path.srv
)

generate_messages(
    DEPENDENCIES
    std_msgs
)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")

catkin_package(
    DEPENDS
        roscpp
        gazebo_ros
)

include_directories(
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIR}
)

add_executable(cloudy src/cloudy.cpp)
target_link_libraries(cloudy ${catkin_LIBRARIES})

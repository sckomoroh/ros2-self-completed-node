cmake_minimum_required(VERSION 3.10)

project(simple-sybc-node)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(${PROJECT_NAME} 
    src/main.cpp
    src/SimpleNode.cpp
)

ament_target_dependencies(${PROJECT_NAME}
    rclcpp
    message_filters
    sensor_msgs
)
install(TARGETS
    ${PROJECT_NAME}
    DESTINATION lib/${PROJECT_NAME}
)


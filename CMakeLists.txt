cmake_minimum_required(VERSION 2.8.3)
project(robomaster_ep)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    nav_msgs
    sensor_msgs
    message_generation
)

add_message_files(
    FILES
    PositionCommand.msg
)
  
generate_messages(
    DEPENDENCIES
    std_msgs
    nav_msgs
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

catkin_package(
    INCLUDE_DIRS 
        inc
        inc/modules
    CATKIN_DEPENDS
        std_msgs
        sensor_msgs
        nav_msgs
        message_runtime
    DEPENDS
        ${LIBS}
)

include_directories(
    inc
    inc/modules
    ${catkin_INCLUDE_DIRS}
)

add_executable(robomaster_ep 
    src/modules/Controller.cc
    src/modules/EventHandler.cc
    src/modules/PushReceiver.cc
    src/Robot.cc
    src/RobotNode.cc
    src/RobotSubscriber.cc
    src/main.cc)

target_link_libraries(robomaster_ep 
    ${catkin_LIBRARIES}
)

add_dependencies(robomaster_ep robomaster_ep_generate_messages_cpp)

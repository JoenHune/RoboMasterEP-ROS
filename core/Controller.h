#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <string>

#pragma once

#include "Types.h"

namespace RoboMasterEP
{

class Robot;

class Controller
{
private:
    in_addr_t ip;
    int port;
    int _tcp_socket = -1;

    const int BUFFER_LENGTH = 1024;
    
    char *receive_buffer;

    // create a tcp link with robot
    int connect_via_tcp(in_addr_t ip, int port);

public:
    Robot *robot;

    Controller(Robot *robot, in_addr_t ip, int port);
    ~Controller();

    // basic

    // basic: send a command to robot
    // @brief: 
    //      send a command to robot and receive response from it
    // @params;
    //      command(std::string): command string
    // @return:
    //      std::string: robot's response
    std::string send_command(std::string command);

    // 2.2.2 robot control
    
    // 2.2.2.1 robot movement mode control
    // @brief: 
    //      control robot movement mode
    // @params:
    //      mode(enum ControlMode): mode of robot's movement
    //          - chassis_lead: gimbal follow chassis
    //          - gimbal_lead: chassis follow gimbal
    //          - free: control gimbal and chassis separately
    // @return:
    //      bool: operation succeeded or failed
    bool set_robot_mode(ControlMode mode);

    // 2.2.2.2 robot movement mode acquire
    // @brief: 
    //      acquire robot movement mode
    // @params:
    //      none
    // @return:
    //      enum ControlMode: current mode of robot
    //          - chassis_lead: gimbal follow chassis
    //          - gimbal_lead: chassis follow gimbal
    //          - free: control gimbal and chassis separately
    ControlMode get_robot_mode();

    // 2.2.3 chassis control

    // 2.2.3.1 chassis speed control
    // @brief: 
    //      control chassis' speed
    // @params:
    //      vx(float:[-3.5, 3.5]): x axis velocity of chassis, m/s
    //      vy(float:[-3.5, 3.5]): y axis velocity of chassis, m/s
    //      vz(float:[-600, 600]): z axis rotation velocity of chassis, °/s
    // @return:
    //      bool: operation succeeded or failed
    bool set_chassis_speed(float vx, float vy, float vz);
    
    // 2.2.3.2 chassis wheels' speed control
    // @brief: 
    //      control chassis wheels seperately
    // @params:
    //      w1(int:[-1000, 1000]): velocity of right-front wheel, rpm
    //      w2(int:[-1000, 1000]): velocity of left-front wheel, rpm
    //      w3(int:[-1000, 1000]): velocity of right-rear wheel, rpm
    //      w4(int:[-1000, 1000]): velocity of left-rear wheel, rpm
    // @return:
    //      bool: operation succeeded or failed
    bool set_wheels_rpm(float w1, float w2, float w3, float w4);

    // 2.2.3.3 chassis relative position control
    // @brief: 
    //      control chassis to a specified position relative to current position with appointed speed
    // @params:
    //      dx(int:[-5, 5]): x axis movement of chassis, m
    //      dy(int:[-5, 5]): y axis movement of chassis, m
    //      dz(int:[-1800, 1800]): z axis rotation of chassis, °
    //      speed_xy(int:(0, 3.5]): speed of x and y axes movement, m/s
    //      speed_z(int:(0, 600}): rotate velocity of z axis rotation, °/s
    // @return:
    //      bool: operation succeeded or failed
    bool set_chassis_position_relative(int dx, int dy, int dz, int speed_xy, int speed_z);

    // 2.2.3.4 acquire chassis' speed
    // @brief: 
    //      acquire chassis' speed
    // @params:
    //      none
    // @return:
    //      struct ChassisSpeed: chassis' speed
    ChassisSpeed get_chassis_speed();

    // 2.2.3.5 acquire chassis' position
    // @brief: 
    //      acquire chassis' position
    // @params:
    //      none
    // @return:
    //      struct ChassisPosition: chassis' position
    ChassisPosition get_chassis_position();

    // 2.2.3.6 acquire chassis' attitude
    // @brief: acquire chassis' attitude
    // @params:
    //      none
    // @return:
    //      struct ChassisAttitude: chassis' attitude
    ChassisAttitude get_chassis_attitude();

    // 2.2.3.7 acquire chassis' status
    // @brief: 
    //      acquire chassis' status
    // @params:
    //      none
    // @return:
    //      struct ChassisStatus: chassis' status
    ChassisStatus get_chassis_status();

    // 2.2.3.8 chassis push switch
    // @brief: 
    //      turn on/off specified information push of chassis
    // @params:
    //      s(enum PushSwitch): off/on status
    //      attr(enum ChassisPushAttr): different information push of chassis
    //      freq(enum ChassisPushFrequence): different frequence of information push
    // @return:
    //      bool: operation succeeded or failed
    bool switch_chassis_push_info(PushSwitch s, ChassisPushAttr attr, ChassisPushFrequence freq);
};

};

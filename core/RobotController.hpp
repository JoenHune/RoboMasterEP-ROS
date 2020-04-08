#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <iostream>
#include <string>
#include <iomanip>

struct ChassisSpeed
{
    float vx;
    float vy;
    float vz;
    int w1;
    int w2;
    int w3;
    int w4;
};

struct ChassisPosition
{
    float x;
    float y;
    float z;
};

struct ChassisAttitude
{
    float pitch;
    float roll;
    float yaw;
};

union ChassisStatus
{
    uint16_t code;
    struct
    {
        uint16_t stop:1;
        uint16_t uphill:1;
        uint16_t downhill:1;
        uint16_t on_slope:1;
        uint16_t pick_up:1;
        uint16_t slip:1;
        uint16_t impact_x:1;
        uint16_t impact_y:1;
        uint16_t impact_z:1;
        uint16_t roll_over:1;
        uint16_t hill_stop:1;

        uint16_t reserve:5;
    } bit;
};

class RobotController
{
private:
    const int PORT_UDP_BROADCAST = 40926;
    const int PORT_TCP_CONTROL = 40923;
    const int BUFFER_LENGTH = 1024;

    char *receive_buffer;
    
    in_addr_t IP = INADDR_NONE;
    int _tcp_socket = -1;

    // scan and require robot's IP address
    in_addr_t _get_ip(int port);
    // create a tcp link with robot
    int _connect(in_addr_t IP, int port);

public:
    enum ControlMode
    {
        chassis_lead = 0,
        gimbal_lead,
        free
    };

    enum PushSwitch
    {
        off = 0,
        on
    };

    enum ChassisPushAttr
    {
        position = 0,
        attitude,
        status,
        all
    };

    enum ChassisPushFrequence
    {
        f1 = 1,
        f2 = 5,
        f3 = 10,
        f4 = 20,
        f5 = 30,
        f6 = 60
    };

    RobotController();
    ~RobotController();

    // basic

    // @brief: send a control command to robot
    // @params;
    //      command(std::string): command string
    // @return:
    //      std::string: robot's response
    std::string send_command(std::string command);

    // 2.2.2 robot control
    
    // 2.2.2.1 robot movement mode control
    // @brief: control robot movement mode
    // @params:
    //      mode(enum ControlMode): mode of robot's movement
    //          - chassis_lead: gimbal follow chassis
    //          - gimbal_lead: chassis follow gimbal
    //          - free: control gimbal and chassis separately
    // @return:
    //      bool: operation succeeded or failed
    bool set_robot_mode(ControlMode mode);

    // 2.2.2.2 robot movement mode acquire
    // @brief: acquire robot movement mode
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
    // @brief: control chassis' speed
    // @params:
    //      vx(float:[-3.5, 3.5]): x axis velocity of chassis, m/s
    //      vy(float:[-3.5, 3.5]): y axis velocity of chassis, m/s
    //      vz(float:[-600, 600]): z axis rotation velocity of chassis, °/s
    // @return:
    //      bool: operation succeeded or failed
    bool set_chassis_speed(float vx, float vy, float vz);
    
    // 2.2.3.2 chassis wheels' speed control
    // @brief: control chassis wheels seperately
    // @params:
    //      w1(int:[-1000, 1000]): velocity of right-front wheel, rpm
    //      w2(int:[-1000, 1000]): velocity of left-front wheel, rpm
    //      w3(int:[-1000, 1000]): velocity of right-rear wheel, rpm
    //      w4(int:[-1000, 1000]): velocity of left-rear wheel, rpm
    // @return:
    //      bool: operation succeeded or failed
    bool set_wheels_rpm(float w1, float w2, float w3, float w4);

    // 2.2.3.3 chassis relative position control
    // @brief: control chassis to a specified position relative to current position with appointed speed
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
    // @brief: acquire chassis' speed
    // @params:
    //      none
    // @return:
    //      struct ChassisSpeed: chassis' speed
    ChassisSpeed get_chassis_speed();

    // 2.2.3.5 acquire chassis' position
    // @brief: acquire chassis' position
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
    // @brief: acquire chassis' status
    // @params:
    //      none
    // @return:
    //      struct ChassisStatus: chassis' status
    ChassisStatus get_chassis_status();

    // 2.2.3.8 chassis push switch
    // @brief: turn on/off specified information push of chassis
    // @params:
    //      s(enum PushSwitch): off/on status
    //      attr(enum ChassisPushAttr): different information push of chassis
    //      freq(enum ChassisPushFrequence): different frequence of information push
    // @return:
    //      bool: operation succeeded or failed
    bool switch_chassis_push_info(PushSwitch s, ChassisPushAttr attr, ChassisPushFrequence freq);
};

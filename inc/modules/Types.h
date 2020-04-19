#pragma once

#include <stdlib.h>
#include <cstdint>
#include <string>

namespace RoboMasterEP
{
    
enum class ThreadType
{
    PUSHRECEIVER = 0,
    EVENTDETECTOR,
    NUMBER_OF_TYPES
};

enum class ControlMode
{
    FREE = 0,
    CHASSIS_LEAD,
    GIMBAL_LEAD,
};

enum class PushSwitch
{
    OFF = 0,
    ON
};

enum class ChassisPushAttr
{
    POSITION = 0,
    ATTITUDE,
    STATUS,
    ALL
};

enum class ChassisPushFrequence
{
    FREQ_OFF = 0,
    FREQ_1Hz = 1,
    FREQ_5Hz = 5,
    FREQ_10Hz = 10,
    FREQ_20Hz = 20,
    FREQ_30Hz = 30,
    FREQ_50Hz = 50
};

enum class LEDComp
{
    ALL = 0,
    TOP_ALL,
    TOP_RIGHT,
    TOP_LEFT,
    BOTTOM_ALL,
    BOTTOM_FRONT,
    BOTTOM_BACK,
    BOTTOM_LEFT,
    BOTTOM_RIGHT
};
std::string LEDCompToString(LEDComp comp);

enum class LEDEffect
{
    OFF = 0,
    SOLID,
    PULSE,
    BLINK,
    SCROLLING // only for gimbal's leds
};
std::string LEDEffectToString(LEDEffect effect);

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

};
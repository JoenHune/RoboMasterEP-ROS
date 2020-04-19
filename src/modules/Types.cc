#include "Types.h"

using namespace RoboMasterEP;

std::string RoboMasterEP::LEDCompToString(LEDComp comp)
{
    switch (comp)
    {
    case LEDComp::ALL:
        return std::string("all");
    case LEDComp::TOP_ALL:
        return std::string("top_all");
    case LEDComp::TOP_RIGHT:
        return std::string("top_right");
    case LEDComp::TOP_LEFT:
        return std::string("top_left");
    case LEDComp::BOTTOM_ALL:
        return std::string("bottom_all");
    case LEDComp::BOTTOM_FRONT:
        return std::string("bottom_front");
    case LEDComp::BOTTOM_BACK:
        return std::string("bottom_back");
    case LEDComp::BOTTOM_LEFT:
        return std::string("bottom_left");
    case LEDComp::BOTTOM_RIGHT:
        return std::string("bottom_right");
    }
    return std::string("");
};

std::string RoboMasterEP::LEDEffectToString(LEDEffect effect)
{
    switch (effect)
    {
    case LEDEffect::OFF:
        return std::string("off");
    case LEDEffect::SOLID:
        return std::string("solid");
    case LEDEffect::PULSE:
        return std::string("pulse");
    case LEDEffect::BLINK:
        return std::string("blink");
    case LEDEffect::SCROLLING:
        return std::string("scrolling");
    }
    return std::string("");
};

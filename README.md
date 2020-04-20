# RoboMasterEP-ROS

C++ version RoboMaster SDK and ROS Interfaces for RoboMaster EP

Working in progress...

## Current to-do list

- [x] A basic framework consists an independent C++ SDK and corresponding ROS package
- [ ] More optimization maybe

### Connection

- [x] Connection via WiFi LAN including IP scanning and TCP transmitting
- [ ] Connection via USB (based on [RNDIS](https://en.wikipedia.org/wiki/RNDIS))

### Controller

- [x] Control of chassis with `string` command line
- [x] Control of chassis with Joystick (work with ROS and `ros-kinetic-joy` package)
- [ ] Control of chassis with expected state(s) which probably consists expected position and expected velocity
- [ ] LEDs's control

### Push(Messages) Receiver

- [x] A basic receiver of messages pushed by chassis in an independent thread
- [ ] Create feedback(s) to Joystick while messages impact that something is peculiar

### Events Handler

- [ ] Handler of Sensor Adapter's events

## Existing Problem(s)

1. `controller->set_chassis_speed()` needs a short break-time (set moving speed back to 0 or another direction) to change robot's direction.

2. LEDs seems out-of-control regardless of [official python sample](https://robomaster-dev.readthedocs.io/zh_CN/latest/sdk/connection.html#id5) nor this self-develop c++ sdk with a chassis-only setup

## Reference

1. [Official Website](https://www.dji.com/robomaster-ep)
2. [RoboMaster Developer Guide](https://robomaster-dev.readthedocs.io/zh_CN/latest/quick_start.html)

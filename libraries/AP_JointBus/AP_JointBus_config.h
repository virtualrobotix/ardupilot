/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Position-controlled joint actuators on a field bus (serial servo bus today, CAN later).
// Enabled on SITL by default; boards opt in from hwdef.dat.
#ifndef AP_JOINTBUS_ENABLED
#define AP_JOINTBUS_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// Backend: the XDuck sketch on the Arduino Uno Q MCU (15 SCS servos and the IMU at 100 Hz)
#ifndef AP_JOINTBUS_XDUCK_ENABLED
#define AP_JOINTBUS_XDUCK_ENABLED AP_JOINTBUS_ENABLED
#endif

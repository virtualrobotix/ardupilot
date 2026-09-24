#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// MicroDuck PPO policy task. Enabled on SITL by default; boards must opt in
// (the MLP weights are ~770 KB of flash as float32).
#ifndef AP_MICRODUCK_ENABLED
#define AP_MICRODUCK_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// Second network (Cartan + DiLU, ~500 KB float32) selectable with MDK_POLICY 1.
// Boards short on flash compile only the MLP.
#ifndef AP_MICRODUCK_CARTAN_ENABLED
#define AP_MICRODUCK_CARTAN_ENABLED AP_MICRODUCK_ENABLED
#endif

// Joint feedback source: SITL reads the plant's joints from the SIM_JSON backend
#ifndef AP_MICRODUCK_JOINT_FEEDBACK_SITL_ENABLED
#define AP_MICRODUCK_JOINT_FEEDBACK_SITL_ENABLED (AP_MICRODUCK_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

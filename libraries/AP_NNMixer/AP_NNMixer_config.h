/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
 */
#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// NNMixer PPO policy task. Enabled on SITL by default; boards must opt in
// (the MLP weights are ~770 KB of flash as float32).
#ifndef AP_NNMIXER_ENABLED
#define AP_NNMIXER_ENABLED (CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

// Second network (Cartan + DiLU, ~500 KB float32) selectable with NNM_POLICY 1.
// Boards short on flash compile only the MLP.
#ifndef AP_NNMIXER_CARTAN_ENABLED
#define AP_NNMIXER_CARTAN_ENABLED AP_NNMIXER_ENABLED
#endif

// Joint feedback source: SITL reads the plant's joints from the SIM_JSON backend
#ifndef AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED
#define AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED (AP_NNMIXER_ENABLED && CONFIG_HAL_BOARD == HAL_BOARD_SITL)
#endif

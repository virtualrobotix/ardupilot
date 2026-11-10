/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include "AP_JointBus.h"

#if AP_JOINTBUS_ENABLED

class AP_JointBus_Backend {
public:
    virtual ~AP_JointBus_Backend() {}

    // start the link; false when it cannot run (the frontend then deletes the backend)
    virtual bool init() = 0;

    virtual bool get_feedback(AP_JointBus::Feedback &fb) = 0;
    virtual bool torque_enabled() = 0;
    virtual void set_velocity_alpha(float alpha) = 0;
    virtual void write(const float *q_rad, uint8_t n, bool enable) = 0;
    virtual void set_aux_deg(float deg) = 0;
    virtual void request(const AP_JointBus::Request &req) = 0;
    virtual bool pop_reply(AP_JointBus::Reply &reply) = 0;
    virtual bool exclusive() = 0;
    virtual void end_exclusive() = 0;
    virtual void get_telemetry(AP_JointBus::Telemetry &t) = 0;
};

#endif  // AP_JOINTBUS_ENABLED

/*
  AP_JointBus: joint actuators that take a position target and report position and velocity over
  a field bus. The frontend is what a joint controller (AP_NNMixer) talks to; a backend owns one
  kind of link. Joints are indexed in the controller's order, the backend maps them to its
  actuators. One extra "aux" actuator (a gripper, a mouth) rides along outside the joint vector.

  Backends: XDuck (the Arduino Uno Q sketch driving SCS servos). A direct SCS UART backend and a
  CAN backend fit behind the same interface.

  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include "AP_JointBus_config.h"

#if AP_JOINTBUS_ENABLED

#include <AP_HAL/AP_HAL.h>

class AP_JointBus_Backend;

class AP_JointBus {
public:
    static const uint8_t MAX_JOINTS = 20;
    static const uint8_t MAX_SLOTS = 16;    // actuators on the bus, in the backend's own order

    AP_JointBus();

    CLASS_NO_COPY(AP_JointBus);

    static AP_JointBus *get_singleton() { return _singleton; }

    struct Feedback {
        uint32_t ms;                // AP_HAL::millis() when the sample arrived
        uint8_t count;
        float pos[MAX_JOINTS];      // rad
        float vel[MAX_JOINTS];      // rad/s
    };

    // Telemetry::status bits
    enum Status : uint16_t {
        STATUS_LINK        = 1U << 0,   // state packets arriving
        STATUS_IMU_OK      = 1U << 1,
        STATUS_TORQUE      = 1U << 2,   // actuators powered and tracking targets
        STATUS_TIMED_OUT   = 1U << 3,   // the bus controller dropped torque on its own
        STATUS_CAL         = 1U << 4,   // exclusive mode: joint centering
        STATUS_SERVO_SETUP = 1U << 5,   // exclusive mode: single-servo setup
        STATUS_RAW_VALID   = 1U << 7,   // raw_pos is current
    };

    struct Telemetry {
        uint16_t status;
        uint8_t n_slots;
        uint16_t present_mask;      // actuators that answered the last poll
        uint16_t held_mask;         // actuators holding their centre in calibration
        float angle_deg[MAX_SLOTS];
        float target_deg[MAX_SLOTS];
        uint16_t raw_pos[MAX_SLOTS];
        uint16_t volt_mask;
        uint8_t deci_volt[MAX_SLOTS];
        uint32_t volt_ms;           // 0 = never received
        float rx_hz;
        uint32_t crc_errors;
        uint32_t rejected_commands;
    };

    // setup operations, carried by the NNM_SRV MAVLink message (data[1])
    enum class Op : uint8_t {
        CAL_ENTER = 10,
        CAL_EXIT = 11,
        CAL_CENTER = 12,                // id = actuator slot
        HEARTBEAT = 13,                 // keeps an exclusive mode open
        SERVO_ENTER = 20,
        SERVO_EXIT = 21,
        SERVO_UNLOCK = 22,              // id = bus id
        SERVO_SET_ID = 23,              // arg0 = new id
        SERVO_SET_GAINS = 24,           // arg0 = kp, arg1 = kd
        SERVO_READ = 25,
        SERVO_PING = 26,
        SERVO_RESET_MID = 27,
        SERVO_SET_PROTECTION = 28,      // arg0 = 0 off, 1 on (all servos)
        SERVO_SET_PROTECTION_CURRENT = 29,  // arg0 = limit in bus units (all servos)
        SERVO_SET_RUN_GAINS = 30,       // sends the table set with set_run_gains(): one frame, all servos
        SERVO_GOTO = 31,                // id = bus id, arg0 = goal 0..4095: torque on, soft gains, slow
    };

    struct Request {
        uint16_t seq;
        uint8_t op;
        uint8_t id;
        float arg[2];
    };

    struct Reply {
        uint16_t seq;
        uint8_t op;
        uint8_t ok;
        uint8_t id;
        uint16_t raw_pos;
        uint16_t center_pos;
        uint8_t kp;
        uint8_t kd;
        uint8_t protection;
        uint16_t current;
        uint8_t source;             // 0 = answered by the bus controller, 1 = answered here
    };

    // look for a port with SERIALn_PROTOCOL = 51 and start the backend. False when there is none.
    bool init(uint8_t n_joints);
    bool active() const { return _backend != nullptr; }

    // latest joint sample; false when the link is down or a joint never answered
    bool get_feedback(Feedback &fb);

    // true once the actuators are powered and following targets
    bool torque_enabled();

    // first-order smoothing of the reported velocity, referenced to 100 Hz samples. 0 = raw.
    void set_velocity_alpha(float alpha);

    // targets for the next command. enable = false releases torque.
    void write(const float *q_rad, uint8_t n, bool enable);

    void set_aux_deg(float deg);

    // setup operation. An immediate answer (refusal, or an operation with no reply from the bus)
    // is queued like the ones from the bus; collect all of them with pop_reply().
    void request(const Request &req);
    bool pop_reply(Reply &reply);

    // per-servo KP/KD table for the running servos (bit i of mask = slot i). It is stored, not sent:
    // Op SERVO_SET_RUN_GAINS sends it. The MCU applies it at ARM.
    void set_run_gains(uint16_t mask, const uint8_t *kp, const uint8_t *kd, uint8_t n);

    // true while a calibration or servo-setup session owns the bus
    bool exclusive();
    void end_exclusive();

    void get_telemetry(Telemetry &t);

private:
    static AP_JointBus *_singleton;
    AP_JointBus_Backend *_backend;
};

namespace AP {
    AP_JointBus *jointbus();
};

#endif  // AP_JOINTBUS_ENABLED

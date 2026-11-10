/*
  AP_JointBus backend for the XDuck sketch on the Arduino Uno Q microcontroller.

  The STM32 polls 15 SCS servos and the IMU at 100 Hz and publishes one state packet per tick;
  it takes one command packet with 15 targets in degrees. On the Uno Q the packets travel through
  the Arduino Bridge RPC; a relay in the App forwards them over UDP to a serial port of the
  autopilot (SERIALn_PROTOCOL 51). On that hop every packet is followed by a CRC16: the packets
  carry none, and a UDP "serial" port does not keep datagram boundaries.

  What the original Python controller did towards the MCU is done here: echo the state's sample
  time in every command, enable with ARM on the measured angles, keep DISARM flowing while a
  setup session owns the bus, close a setup session when the ground station stops calling.

  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#pragma once

#include "AP_JointBus_config.h"

#if AP_JOINTBUS_XDUCK_ENABLED

#include "AP_JointBus_Backend.h"
#include <AP_HAL/utility/RingBuffer.h>

class AP_JointBus_XDuck : public AP_JointBus_Backend {
public:
    AP_JointBus_XDuck(AP_HAL::UARTDriver *uart, uint32_t baud, uint8_t n_joints);

    bool init() override;
    bool get_feedback(AP_JointBus::Feedback &fb) override;
    bool torque_enabled() override;
    void set_velocity_alpha(float alpha) override;
    void write(const float *q_rad, uint8_t n, bool enable) override;
    void set_aux_deg(float deg) override;
    void request(const AP_JointBus::Request &req) override;
    bool pop_reply(AP_JointBus::Reply &reply) override;
    bool exclusive() override;
    void end_exclusive() override;
    void get_telemetry(AP_JointBus::Telemetry &t) override;

private:
    static const uint8_t NUM_SLOTS = 15;
    static const uint8_t NUM_JOINTS = 14;
    static const uint8_t AUX_SLOT = 14;

    enum class Exclusive : uint8_t { NONE = 0, CAL = 1, SERVO = 2 };

    struct OutFrame {
        uint8_t len;
        uint8_t data[16];
    };

    AP_HAL::UARTDriver *_uart;
    const uint32_t _baud;
    const uint8_t _n_joints;
    uint16_t _joint_mask;

    HAL_Semaphore _sem;

    // receive
    uint8_t _rx[1024];
    uint16_t _rx_len;
    uint32_t _crc_errors;
    uint32_t _rx_count;
    uint32_t _rx_window_ms;
    uint32_t _rx_window_count;
    float _rx_hz;

    // last state packet
    bool _have_state;
    uint32_t _state_ms;
    uint64_t _state_us;
    uint8_t _flags;
    uint16_t _mask;
    uint16_t _seen_mask;
    uint32_t _sample_us;
    uint32_t _imu_us;
    bool _have_imu;
    uint32_t _rejected;
    float _slot_angle[NUM_SLOTS];       // deg
    float _slot_target[NUM_SLOTS];      // deg, as echoed by the MCU
    float _vel[NUM_JOINTS];             // rad/s, joint order, filtered
    bool _vel_valid;
    float _vel_alpha;

    // raw encoders and voltages (setup pages)
    uint16_t _raw_mask;
    uint16_t _held_mask;
    uint16_t _raw_pos[NUM_SLOTS];
    uint32_t _raw_ms;
    uint16_t _volt_mask;
    uint8_t _deci_volt[NUM_SLOTS];
    uint32_t _volt_ms;

    // command
    float _target_rad[NUM_JOINTS];
    float _aux_deg;
    bool _want_enable;
    bool _cmd_pending;
    uint32_t _write_ms;
    uint32_t _seq;
    uint32_t _last_tx_ms;

    // setup sessions
    Exclusive _exclusive;
    uint32_t _exclusive_ms;         // last ground-station call while in a session
    uint16_t _servo_seq;            // request waiting for a servo reply
    uint8_t _servo_op;              // its AP_JointBus::Op, 0 = none
    ObjectBuffer<OutFrame> _out{8};
    ObjectBuffer<AP_JointBus::Reply> _replies{8};

    void thread_main();
    void read_port();
    void dispatch(const uint8_t *frame, uint16_t len);
    void handle_state(const uint8_t *frame);
    void send_command(uint32_t now_ms);
    void send_frame(const uint8_t *data, uint16_t len);
    void queue_cal(uint8_t op, uint8_t index, uint32_t seq);
    void queue_servo(uint8_t op, uint8_t target_id, uint8_t new_id, uint8_t kp, uint8_t kd,
                     uint8_t protection, uint16_t current);
    void local_reply(const AP_JointBus::Request &req, bool ok);
    void close_session_locked();
};

#endif  // AP_JOINTBUS_XDUCK_ENABLED

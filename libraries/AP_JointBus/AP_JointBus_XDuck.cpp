/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
*/
#include "AP_JointBus_XDuck.h"

#if AP_JOINTBUS_XDUCK_ENABLED

#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_InertialSensor/AP_InertialSensor_JointBus.h>
#include <string.h>

extern const AP_HAL::HAL& hal;

// ---------------------------------------------------------------------------
// wire format of the sketch (sketch/protocol.h), little endian

namespace {

enum CommandMode : uint8_t { MODE_DISARM = 0, MODE_ARM = 1, MODE_POLICY = 2 };

enum StateFlags : uint8_t {
    FLAG_IMU_OK = 1,
    FLAG_ENABLED = 2,
    FLAG_TIMED_OUT = 4,
    FLAG_CAL = 8,
    FLAG_SERVO_DEBUG = 16,
};

enum CalOp : uint8_t { MCU_CAL_ENTER = 0, MCU_CAL_EXIT = 1, MCU_CAL_CENTER_ONE = 2 };

enum ServoOp : uint8_t {
    MCU_SERVO_ENTER = 0,
    MCU_SERVO_EXIT = 1,
    MCU_SERVO_UNLOCK = 2,
    MCU_SERVO_SET_ID = 3,
    MCU_SERVO_SET_GAINS = 5,
    MCU_SERVO_READ = 6,
    MCU_SERVO_PING = 7,
    MCU_SERVO_RESET_MID = 8,
    MCU_SERVO_SET_PROTECTION = 9,
    MCU_SERVO_SET_PROTECTION_CURRENT = 10,
};

struct PACKED CommandPacket {
    char magic[4];
    uint32_t seq;
    uint32_t source_us;
    uint32_t ttl_us;
    uint8_t mode;
    uint8_t reserved;
    uint16_t reserved2;
    float target[15];
};

struct PACKED StatePacket {
    char magic[4];
    uint8_t version;
    uint8_t flags;
    uint16_t servo_mask;
    uint32_t seq;
    uint32_t sample_us;
    uint32_t imu_us;
    uint32_t command_seq;
    uint32_t command_age_us;
    uint32_t period_us;
    uint32_t work_us;
    uint32_t overruns;
    uint32_t report_drops;
    uint32_t invalid_commands;
    int16_t acc[3];
    int16_t gyro[3];
    float angle[15];
    float velocity[15];
    float target[15];
};

struct PACKED VoltagePacket {
    char magic[4];
    uint32_t seq;
    uint32_t sample_us;
    uint16_t servo_mask;
    uint8_t deci_volt[15];
};

struct PACKED CalPacket {
    char magic[4];
    uint32_t seq;
    uint8_t op;
    uint8_t index;
};

struct PACKED CalReplyPacket {
    char magic[4];
    uint32_t seq;
    uint8_t index;
    uint8_t ok;
};

struct PACKED ServoPacket {
    char magic[4];
    uint8_t op;
    uint8_t target_id;
    uint8_t new_id;
    uint8_t kp;
    uint8_t kd;
    uint8_t protection_flags;
    uint16_t protection_current;
};

struct PACKED RawPacket {
    char magic[4];
    uint16_t mask;
    uint16_t held_mask;
    uint16_t pos[15];
};

struct PACKED ServoReplyPacket {
    char magic[4];
    uint8_t op;
    uint8_t target_id;
    uint8_t ok;
    uint8_t protection_flags;
    uint16_t raw_pos;           // protection current for MCU_SERVO_SET_PROTECTION_CURRENT
    uint16_t center_pos;
    uint8_t kp;
    uint8_t kd;
};

static_assert(sizeof(CommandPacket) == 80, "command ABI");
static_assert(sizeof(StatePacket) == 240, "state ABI");
static_assert(sizeof(VoltagePacket) == 29, "voltage ABI");
static_assert(sizeof(CalPacket) == 10, "cal ABI");
static_assert(sizeof(CalReplyPacket) == 10, "cal reply ABI");
static_assert(sizeof(ServoPacket) == 12, "servo ABI");
static_assert(sizeof(RawPacket) == 38, "raw ABI");
static_assert(sizeof(ServoReplyPacket) == 14, "servo reply ABI");

// packets from the MCU, by magic. DQD1 (diagnostics) is recognised so it is skipped whole.
const struct {
    char magic[5];
    uint16_t len;
} kFrames[] = {
    { "DQS1", 240 },
    { "DQD1", 68 },
    { "DQV1", 29 },
    { "DQA1", 10 },
    { "DQR2", 38 },
    { "DQS3", 14 },
};

uint16_t frame_length(const uint8_t *magic)
{
    for (const auto &f : kFrames) {
        if (memcmp(magic, f.magic, 4) == 0) {
            return f.len;
        }
    }
    return 0;
}

// joint (policy) order -> actuator slot. Slots are servo ids 10..14, 20..24, 30..34; the policy
// order is left leg, neck and head (30..33), right leg. Slot 14 (id 34) is the mouth.
const uint8_t kJointSlot[14] = { 0, 1, 2, 3, 4, 10, 11, 12, 13, 5, 6, 7, 8, 9 };

const uint8_t BROADCAST_ID = 0xFE;
const uint8_t PROTECTION_OFF = 0x04;
const uint8_t PROTECTION_ON = 0x0D;

const uint32_t LINK_TIMEOUT_MS = 150;       // state packets
const uint32_t HOST_TIMEOUT_MS = 150;       // write() calls from the joint controller
const uint32_t SESSION_TIMEOUT_MS = 2000;   // ground-station calls during a setup session
const uint32_t IDLE_TX_MS = 100;            // DISARM keep-alive when nobody calls write()

}  // namespace

AP_JointBus_XDuck::AP_JointBus_XDuck(AP_HAL::UARTDriver *uart, uint32_t baud, uint8_t n_joints) :
    _uart(uart),
    _baud(baud),
    _n_joints(n_joints)
{
}

bool AP_JointBus_XDuck::init()
{
    if (_uart == nullptr || _n_joints == 0 || _n_joints > NUM_JOINTS) {
        return false;
    }
    _joint_mask = 0;
    for (uint8_t j = 0; j < _n_joints; j++) {
        _joint_mask |= uint16_t(1U << kJointSlot[j]);
    }
    return hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_JointBus_XDuck::thread_main, void),
                                        "jointbus", 4096, AP_HAL::Scheduler::PRIORITY_SPI, 0);
}

void AP_JointBus_XDuck::thread_main()
{
    // the port is opened by the thread that uses it
    _uart->begin(_baud, 4096, 2048);
    // a frame per write, sent at once: on a UDP port the buffered path would hold frames back
    // looking for MAVLink packet boundaries in them
    _uart->set_unbuffered_writes(true);
    while (true) {
        hal.scheduler->delay_microseconds(1000);
        read_port();

        const uint32_t now = AP_HAL::millis();
        bool send = false;
        {
            WITH_SEMAPHORE(_sem);
            if (_exclusive != Exclusive::NONE && now - _exclusive_ms > SESSION_TIMEOUT_MS) {
                // the ground station went away with the bus still taken
                close_session_locked();
            }
            send = _cmd_pending || now - _last_tx_ms >= IDLE_TX_MS;
        }
        OutFrame f;
        while (_out.pop(f)) {
            send_frame(f.data, f.len);
        }
        if (send) {
            send_command(now);
        }
    }
}

void AP_JointBus_XDuck::read_port()
{
    const uint32_t avail = _uart->available();
    if (avail > 0) {
        const uint16_t space = uint16_t(sizeof(_rx) - _rx_len);
        const ssize_t n = _uart->read(&_rx[_rx_len], uint16_t(MIN(avail, uint32_t(space))));
        if (n > 0) {
            _rx_len += uint16_t(n);
        }
    }
    uint16_t pos = 0;
    while (_rx_len - pos >= 4) {
        if (_rx[pos] != 'D' || _rx[pos + 1] != 'Q') {
            pos++;
            continue;
        }
        const uint16_t len = frame_length(&_rx[pos]);
        if (len == 0) {
            pos++;
            continue;
        }
        if (_rx_len - pos < len + 2) {
            break;
        }
        const uint16_t crc = crc16_ccitt(&_rx[pos], len, 0);
        if (crc != uint16_t(_rx[pos + len] | (_rx[pos + len + 1] << 8))) {
            _crc_errors++;
            pos++;
            continue;
        }
        dispatch(&_rx[pos], len);
        pos += len + 2;
    }
    if (pos > 0) {
        memmove(_rx, &_rx[pos], _rx_len - pos);
        _rx_len -= pos;
    }
}

void AP_JointBus_XDuck::dispatch(const uint8_t *frame, uint16_t len)
{
    if (memcmp(frame, "DQS1", 4) == 0) {
        handle_state(frame);
        return;
    }
    if (memcmp(frame, "DQR2", 4) == 0) {
        RawPacket r;
        memcpy(&r, frame, sizeof(r));
        WITH_SEMAPHORE(_sem);
        _raw_mask = r.mask;
        _held_mask = r.held_mask;
        memcpy(_raw_pos, r.pos, sizeof(_raw_pos));
        _raw_ms = AP_HAL::millis();
        return;
    }
    if (memcmp(frame, "DQV1", 4) == 0) {
        VoltagePacket v;
        memcpy(&v, frame, sizeof(v));
        WITH_SEMAPHORE(_sem);
        _volt_mask = v.servo_mask;
        memcpy(_deci_volt, v.deci_volt, sizeof(_deci_volt));
        _volt_ms = AP_HAL::millis();
        return;
    }
    if (memcmp(frame, "DQA1", 4) == 0) {
        CalReplyPacket c;
        memcpy(&c, frame, sizeof(c));
        AP_JointBus::Reply reply {};
        reply.seq = uint16_t(c.seq);
        reply.op = uint8_t(AP_JointBus::Op::CAL_CENTER);
        reply.ok = c.ok ? 1 : 0;
        reply.id = c.index;
        WITH_SEMAPHORE(_sem);
        _replies.push(reply);
        return;
    }
    if (memcmp(frame, "DQS3", 4) == 0) {
        ServoReplyPacket s;
        memcpy(&s, frame, sizeof(s));
        static const struct {
            uint8_t mcu;
            AP_JointBus::Op op;
        } map[] = {
            { MCU_SERVO_ENTER, AP_JointBus::Op::SERVO_ENTER },
            { MCU_SERVO_EXIT, AP_JointBus::Op::SERVO_EXIT },
            { MCU_SERVO_UNLOCK, AP_JointBus::Op::SERVO_UNLOCK },
            { MCU_SERVO_SET_ID, AP_JointBus::Op::SERVO_SET_ID },
            { MCU_SERVO_SET_GAINS, AP_JointBus::Op::SERVO_SET_GAINS },
            { MCU_SERVO_READ, AP_JointBus::Op::SERVO_READ },
            { MCU_SERVO_PING, AP_JointBus::Op::SERVO_PING },
            { MCU_SERVO_RESET_MID, AP_JointBus::Op::SERVO_RESET_MID },
            { MCU_SERVO_SET_PROTECTION, AP_JointBus::Op::SERVO_SET_PROTECTION },
            { MCU_SERVO_SET_PROTECTION_CURRENT, AP_JointBus::Op::SERVO_SET_PROTECTION_CURRENT },
        };
        WITH_SEMAPHORE(_sem);
        for (const auto &m : map) {
            if (m.mcu != s.op || _servo_op != uint8_t(m.op)) {
                continue;
            }
            // the reply carries no sequence number: it answers the one request in flight
            AP_JointBus::Reply reply {};
            reply.seq = _servo_seq;
            reply.op = _servo_op;
            reply.ok = s.ok ? 1 : 0;
            reply.id = s.target_id;
            reply.center_pos = s.center_pos;
            reply.kp = s.kp;
            reply.kd = s.kd;
            if (s.op == MCU_SERVO_SET_PROTECTION) {
                reply.protection = s.protection_flags == PROTECTION_ON ? 1 : 0;
            } else if (s.op == MCU_SERVO_SET_PROTECTION_CURRENT) {
                reply.current = s.raw_pos;
            } else {
                reply.raw_pos = s.raw_pos;
            }
            _servo_op = 0;
            _replies.push(reply);
            break;
        }
        return;
    }
    // DQD1: bridge diagnostics, not used
}

void AP_JointBus_XDuck::handle_state(const uint8_t *frame)
{
    StatePacket s;
    memcpy(&s, frame, sizeof(s));
    if (s.version != 1) {
        return;
    }
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        if (!isfinite(s.angle[i]) || !isfinite(s.velocity[i]) || !isfinite(s.target[i])) {
            return;
        }
    }
    const uint64_t now_us = AP_HAL::micros64();
    const uint32_t now_ms = AP_HAL::millis();
    bool new_imu = false;
    {
        WITH_SEMAPHORE(_sem);
        const float dt = _have_state ? float(now_us - _state_us) * 1.0e-6f : 0.01f;
        // velocity smoothing as in the XDuck receive filter: alpha is per 10 ms sample
        const bool restart = !_vel_valid || dt > 0.1f;
        const float keep = (_vel_alpha > 0.0f) ? powf(_vel_alpha, constrain_float(dt, 0.001f, 0.1f) / 0.01f) : 0.0f;
        for (uint8_t j = 0; j < _n_joints; j++) {
            const float v = radians(s.velocity[kJointSlot[j]]);
            _vel[j] = restart ? v : keep * _vel[j] + (1.0f - keep) * v;
        }
        _vel_valid = true;
        memcpy(_slot_angle, s.angle, sizeof(_slot_angle));
        memcpy(_slot_target, s.target, sizeof(_slot_target));
        _flags = s.flags;
        _mask = s.servo_mask;
        _seen_mask |= s.servo_mask;
        _sample_us = s.sample_us;
        _rejected = s.invalid_commands;
        new_imu = (s.flags & FLAG_IMU_OK) && (!_have_imu || s.imu_us != _imu_us);
        if (new_imu) {
            _imu_us = s.imu_us;
            _have_imu = true;
        }
        _have_state = true;
        _state_us = now_us;
        _state_ms = now_ms;
        _rx_count++;
        if (_rx_window_ms == 0) {
            _rx_window_ms = now_ms;
            _rx_window_count = _rx_count;
        } else if (now_ms - _rx_window_ms >= 1000) {
            _rx_hz = float(_rx_count - _rx_window_count) * 1000.0f / float(now_ms - _rx_window_ms);
            _rx_window_ms = now_ms;
            _rx_window_count = _rx_count;
        }
    }
#if AP_INERTIALSENSOR_JOINTBUS_ENABLED
    if (new_imu) {
        // The sketch reports the gravity vector (minus specific force) and the angular rate in the
        // robot frame, x forward, y left, z up. The autopilot wants specific force and rate with
        // x forward, y right, z down.
        const float a = GRAVITY_MSS / 16384.0f;
        const float g = radians(1.0f / 32.0f);
        const Vector3f accel(-s.acc[0] * a, s.acc[1] * a, s.acc[2] * a);
        const Vector3f gyro(s.gyro[0] * g, -s.gyro[1] * g, -s.gyro[2] * g);
        AP_InertialSensor_JointBus::handle_sample(accel, gyro, s.imu_us);
    }
#else
    (void)new_imu;
#endif
}

void AP_JointBus_XDuck::send_frame(const uint8_t *data, uint16_t len)
{
    // one write per frame so the frame leaves in one datagram
    uint8_t buf[sizeof(CommandPacket) + 2];
    if (len + 2U > sizeof(buf)) {
        return;
    }
    memcpy(buf, data, len);
    const uint16_t crc = crc16_ccitt(data, len, 0);
    buf[len] = uint8_t(crc & 0xFF);
    buf[len + 1] = uint8_t(crc >> 8);
    _uart->write(buf, len + 2U);
}

void AP_JointBus_XDuck::send_command(uint32_t now_ms)
{
    CommandPacket c {};
    memcpy(c.magic, "DQC1", 4);
    c.mode = MODE_DISARM;
    {
        WITH_SEMAPHORE(_sem);
        const bool link = _have_state && now_ms - _state_ms < LINK_TIMEOUT_MS;
        const bool host = now_ms - _write_ms < HOST_TIMEOUT_MS;
        // DISARM and ARM carry the measured angles: enabling never commands a jump
        memcpy(c.target, _slot_angle, sizeof(c.target));
        if (_want_enable && host && link && _exclusive == Exclusive::NONE) {
            if (_flags & FLAG_ENABLED) {
                c.mode = MODE_POLICY;
                for (uint8_t j = 0; j < _n_joints; j++) {
                    c.target[kJointSlot[j]] = degrees(_target_rad[j]);
                }
                c.target[AUX_SLOT] = _aux_deg;
            } else if ((_flags & FLAG_IMU_OK) && (_mask & _joint_mask) == _joint_mask) {
                c.mode = MODE_ARM;
            }
        }
        for (uint8_t i = 0; i < NUM_SLOTS; i++) {
            if (!isfinite(c.target[i])) {
                c.mode = MODE_DISARM;
                c.target[i] = 0.0f;
            }
        }
        c.seq = ++_seq;
        c.source_us = _sample_us;
        _cmd_pending = false;
        _last_tx_ms = now_ms;
    }
    send_frame((const uint8_t *)&c, sizeof(c));
}

bool AP_JointBus_XDuck::get_feedback(AP_JointBus::Feedback &fb)
{
    WITH_SEMAPHORE(_sem);
    if (!_have_state || (_seen_mask & _joint_mask) != _joint_mask ||
        AP_HAL::millis() - _state_ms > LINK_TIMEOUT_MS) {
        return false;
    }
    fb.ms = _state_ms;
    fb.count = _n_joints;
    for (uint8_t j = 0; j < _n_joints; j++) {
        fb.pos[j] = radians(_slot_angle[kJointSlot[j]]);
        fb.vel[j] = _vel[j];
    }
    return true;
}

bool AP_JointBus_XDuck::torque_enabled()
{
    WITH_SEMAPHORE(_sem);
    return _have_state && (_flags & FLAG_ENABLED) && AP_HAL::millis() - _state_ms < LINK_TIMEOUT_MS;
}

void AP_JointBus_XDuck::set_velocity_alpha(float alpha)
{
    WITH_SEMAPHORE(_sem);
    _vel_alpha = constrain_float(alpha, 0.0f, 0.95f);
}

void AP_JointBus_XDuck::write(const float *q_rad, uint8_t n, bool enable)
{
    WITH_SEMAPHORE(_sem);
    if (q_rad != nullptr) {
        memcpy(_target_rad, q_rad, sizeof(float) * MIN(n, _n_joints));
    }
    _want_enable = enable && q_rad != nullptr && n >= _n_joints;
    _write_ms = AP_HAL::millis();
    _cmd_pending = true;
}

void AP_JointBus_XDuck::set_aux_deg(float deg)
{
    WITH_SEMAPHORE(_sem);
    _aux_deg = isfinite(deg) ? deg : 0.0f;
}

void AP_JointBus_XDuck::queue_cal(uint8_t op, uint8_t index, uint32_t seq)
{
    CalPacket p {};
    memcpy(p.magic, "DQL2", 4);
    p.seq = seq;
    p.op = op;
    p.index = index;
    OutFrame f {};
    f.len = sizeof(p);
    memcpy(f.data, &p, sizeof(p));
    _out.push(f);
}

void AP_JointBus_XDuck::queue_servo(uint8_t op, uint8_t target_id, uint8_t new_id, uint8_t kp, uint8_t kd,
                                    uint8_t protection, uint16_t current)
{
    ServoPacket p {};
    memcpy(p.magic, "DQV2", 4);
    p.op = op;
    p.target_id = target_id;
    p.new_id = new_id;
    p.kp = kp;
    p.kd = kd;
    p.protection_flags = protection;
    p.protection_current = current;
    OutFrame f {};
    f.len = sizeof(p);
    memcpy(f.data, &p, sizeof(p));
    _out.push(f);
}

void AP_JointBus_XDuck::local_reply(const AP_JointBus::Request &req, bool ok)
{
    AP_JointBus::Reply reply {};
    reply.seq = req.seq;
    reply.op = req.op;
    reply.ok = ok ? 1 : 0;
    reply.id = req.id;
    reply.source = 1;
    _replies.push(reply);
}

void AP_JointBus_XDuck::close_session_locked()
{
    if (_exclusive == Exclusive::CAL) {
        queue_cal(MCU_CAL_EXIT, 0, 0);
    } else if (_exclusive == Exclusive::SERVO) {
        queue_servo(MCU_SERVO_EXIT, 10, 0, 0, 0, 0, 0);
    }
    _exclusive = Exclusive::NONE;
    _servo_op = 0;
}

void AP_JointBus_XDuck::request(const AP_JointBus::Request &req)
{
    using Op = AP_JointBus::Op;
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    const bool link = _have_state && now - _state_ms < LINK_TIMEOUT_MS;
    const Op op = Op(req.op);
    const int32_t a0 = isfinite(req.arg[0]) ? int32_t(lrintf(req.arg[0])) : -1;
    const int32_t a1 = isfinite(req.arg[1]) ? int32_t(lrintf(req.arg[1])) : -1;

    switch (op) {
    case Op::HEARTBEAT:
        if (_exclusive != Exclusive::NONE) {
            _exclusive_ms = now;
        }
        local_reply(req, _exclusive != Exclusive::NONE);
        return;

    case Op::CAL_ENTER:
    case Op::SERVO_ENTER:
        if (!link) {
            local_reply(req, false);
            return;
        }
        close_session_locked();
        _exclusive_ms = now;
        if (op == Op::CAL_ENTER) {
            _exclusive = Exclusive::CAL;
            queue_cal(MCU_CAL_ENTER, 0, 0);
        } else {
            _exclusive = Exclusive::SERVO;
            queue_servo(MCU_SERVO_ENTER, 10, 0, 0, 0, 0, 0);
        }
        local_reply(req, true);
        return;

    case Op::CAL_EXIT:
    case Op::SERVO_EXIT:
        close_session_locked();
        local_reply(req, true);
        return;

    case Op::CAL_CENTER:
        if (_exclusive != Exclusive::CAL || !link || req.id >= NUM_SLOTS) {
            local_reply(req, false);
            return;
        }
        _exclusive_ms = now;
        queue_cal(MCU_CAL_CENTER_ONE, req.id, req.seq);
        return;

    default:
        break;
    }

    // single-servo operations: one in flight, answered by the MCU
    if (_exclusive != Exclusive::SERVO || !link) {
        local_reply(req, false);
        return;
    }
    _exclusive_ms = now;
    bool ok = req.id <= 253;
    switch (op) {
    case Op::SERVO_UNLOCK:
        if (ok) {
            queue_servo(MCU_SERVO_UNLOCK, req.id, 0, 0, 0, 0, 0);
        }
        break;
    case Op::SERVO_SET_ID:
        ok = ok && a0 >= 0 && a0 <= 253;
        if (ok) {
            queue_servo(MCU_SERVO_SET_ID, req.id, uint8_t(a0), 0, 0, 0, 0);
        }
        break;
    case Op::SERVO_SET_GAINS:
        ok = ok && a0 >= 0 && a0 <= 255 && a1 >= 0 && a1 <= 255;
        if (ok) {
            queue_servo(MCU_SERVO_SET_GAINS, req.id, 0, uint8_t(a0), uint8_t(a1), 0, 0);
        }
        break;
    case Op::SERVO_READ:
        if (ok) {
            queue_servo(MCU_SERVO_READ, req.id, 0, 0, 0, 0, 0);
        }
        break;
    case Op::SERVO_PING:
        if (ok) {
            queue_servo(MCU_SERVO_PING, req.id, 0, 0, 0, 0, 0);
        }
        break;
    case Op::SERVO_RESET_MID:
        if (ok) {
            queue_servo(MCU_SERVO_RESET_MID, req.id, 0, 0, 0, 0, 0);
        }
        break;
    case Op::SERVO_SET_PROTECTION:
        // applies to every servo on the bus
        ok = a0 == 0 || a0 == 1;
        if (ok) {
            queue_servo(MCU_SERVO_SET_PROTECTION, BROADCAST_ID, 0, 0, 0, a0 ? PROTECTION_ON : PROTECTION_OFF, 0);
        }
        break;
    case Op::SERVO_SET_PROTECTION_CURRENT:
        ok = a0 >= 0 && a0 <= 65535;
        if (ok) {
            queue_servo(MCU_SERVO_SET_PROTECTION_CURRENT, BROADCAST_ID, 0, 0, 0, 0, uint16_t(a0));
        }
        break;
    default:
        ok = false;
        break;
    }
    if (!ok) {
        local_reply(req, false);
        return;
    }
    _servo_seq = req.seq;
    _servo_op = req.op;
}

bool AP_JointBus_XDuck::pop_reply(AP_JointBus::Reply &reply)
{
    WITH_SEMAPHORE(_sem);
    return _replies.pop(reply);
}

bool AP_JointBus_XDuck::exclusive()
{
    WITH_SEMAPHORE(_sem);
    return _exclusive != Exclusive::NONE;
}

void AP_JointBus_XDuck::end_exclusive()
{
    WITH_SEMAPHORE(_sem);
    close_session_locked();
}

void AP_JointBus_XDuck::get_telemetry(AP_JointBus::Telemetry &t)
{
    WITH_SEMAPHORE(_sem);
    const uint32_t now = AP_HAL::millis();
    const bool link = _have_state && now - _state_ms < LINK_TIMEOUT_MS;
    t.n_slots = NUM_SLOTS;
    t.status = 0;
    if (link) {
        t.status |= AP_JointBus::STATUS_LINK;
        if (_flags & FLAG_IMU_OK) {
            t.status |= AP_JointBus::STATUS_IMU_OK;
        }
        if (_flags & FLAG_ENABLED) {
            t.status |= AP_JointBus::STATUS_TORQUE;
        }
        if (_flags & FLAG_TIMED_OUT) {
            t.status |= AP_JointBus::STATUS_TIMED_OUT;
        }
    }
    if (_exclusive == Exclusive::CAL) {
        t.status |= AP_JointBus::STATUS_CAL;
    } else if (_exclusive == Exclusive::SERVO) {
        t.status |= AP_JointBus::STATUS_SERVO_SETUP;
    }
    const bool raw = _raw_ms != 0 && now - _raw_ms < 500;
    if (raw) {
        t.status |= AP_JointBus::STATUS_RAW_VALID;
    }
    t.present_mask = link ? _mask : 0;
    t.held_mask = raw ? _held_mask : 0;
    for (uint8_t i = 0; i < NUM_SLOTS; i++) {
        t.angle_deg[i] = _slot_angle[i];
        t.target_deg[i] = _slot_target[i];
        t.raw_pos[i] = _raw_pos[i];
        t.deci_volt[i] = _deci_volt[i];
    }
    t.volt_mask = _volt_mask;
    t.volt_ms = _volt_ms;
    t.rx_hz = link ? _rx_hz : 0.0f;
    t.crc_errors = _crc_errors;
    t.rejected_commands = _rejected;
}

#endif  // AP_JOINTBUS_XDUCK_ENABLED

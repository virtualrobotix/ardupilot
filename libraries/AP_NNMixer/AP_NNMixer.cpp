/*
  Author: Roberto Navoni, member of the ArduPilot Dev Team
  Contact: r.navoni74@gmail.com
  Developed by Roberto Navoni — DelphyAI LAB
  For information: r.navoni74@gmail.com
 */
#include "AP_NNMixer_config.h"

#if AP_NNMIXER_ENABLED

#include "AP_NNMixer.h"
#include "nnmixer_infer.h"
#if AP_NNMIXER_BAKED_MLP_ENABLED
#include "policy_mlp.h"
#endif
#if AP_NNMIXER_CARTAN_ENABLED
#include "policy_cartan.h"
#endif

#include <AP_HAL/AP_HAL.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <AP_InertialSensor/AP_InertialSensor_JointBus.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Vehicle/AP_Vehicle.h>
#include <GCS_MAVLink/GCS.h>
#include <RC_Channel/RC_Channel.h>
#include <SRV_Channel/SRV_Channel.h>
#include <AP_Filesystem/AP_Filesystem.h>

#include <stdlib.h>
#include <string.h>

#if AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED
#include <SITL/SITL.h>
#endif
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
#include <time.h>
#endif


extern const AP_HAL::HAL& hal;

#define NNM_PWM_CENTER 1500
#define NNM_RAD_PER_US 0.003f
#define NNM_PWM_MIN 800
#define NNM_PWM_MAX 2200

// Rover mode numbers (Mode::Number) — keep in sync with Rover/mode.h
static const uint8_t kModeManual = 0;
static const uint8_t kModeHold = 4;
static const uint8_t kModeAuto = 10;
static const uint8_t kModeRtl = 11;
static const uint8_t kModeSmartRtl = 12;
static const uint8_t kModeGuided = 15;

const AP_Param::GroupInfo AP_NNMixer::var_info[] = {
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_NNMixer, _enable, 0, AP_PARAM_FLAG_ENABLE),

    // @Param: ROBOT
    // @DisplayName: Robot topology (boot)
    // @Description: Selects /APM/nnm/<name>/robot.bin. Requires reboot. 0 MicroDuck, 1 Microban, 2 Zeroth, 3 Bimo, 4 Legolas, 5 Upkie, 6 Rex, 7 Yertle, 8 AlbertPro, 9 Open Duck Mini v2, 10 Freenove Robot Dog, 11 Petoi Bittle, 12 Booster T1, 13 flybody (Drosophila, simulation only), 14 UBTECH Yanshee, 15 UBTECH TienKung (Walker), 16 ToddlerBot, 17 Jumper, 18 PhoneBot, 19 PhoneBot reversed (pelvis and legs turned 180 deg), 20 Sprite (31 joints, over NNM_MAX_JOINTS), 21 XGO Duck (XDuck policies, servos on the Arduino Uno Q joint bus), 22 Q8bot (8 Dynamixel, closed-chain legs)
    // @Values: 0:MicroDuck,1:Microban,2:Zeroth,3:Bimo,4:Legolas,5:Upkie,6:Rex,7:Yertle,8:AlbertPro,9:OpenDuckMini,10:FreenoveDog,11:Bittle,12:BoosterT1,13:Flybody,14:Yanshee,15:TienKung,16:ToddlerBot,17:Jumper,18:PhoneBot,19:PhoneBotReversed,20:Sprite,21:XGODuck,22:Q8bot
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("ROBOT", 17, AP_NNMixer, _robot, 0),

    // @Param: POLICY
    // @DisplayName: Policy file index for the boot robot
    // @Description: Index into /APM/nnm/<robot>/policies/*.nnm (alphabetical). Switch copies SD→RAM into the free int8 slot. Rejects files whose robot_id does not match the boot robot.
    // @Range: 0 7
    // @User: Advanced
    AP_GROUPINFO("POLICY", 2, AP_NNMixer, _policy, 0),

    AP_GROUPINFO("VX_MAX", 3, AP_NNMixer, _vx_max, 0.4f),
    AP_GROUPINFO("VY_MAX", 4, AP_NNMixer, _vy_max, 0.3f),
    AP_GROUPINFO("WZ_MAX", 5, AP_NNMixer, _wz_max, 1.0f),
    AP_GROUPINFO("WD_MS", 6, AP_NNMixer, _wd_ms, 40),
    AP_GROUPINFO("ATT_SRC", 7, AP_NNMixer, _att_src, 0),
    AP_GROUPINFO("ATT_TAU", 8, AP_NNMixer, _att_tau, 0.5f),
    AP_GROUPINFO("RC_VX", 9, AP_NNMixer, _rc_vx, 2),
    AP_GROUPINFO("RC_VY", 10, AP_NNMixer, _rc_vy, 1),
    AP_GROUPINFO("RC_WZ", 11, AP_NNMixer, _rc_wz, 4),
    AP_GROUPINFO("ACT_MAX", 12, AP_NNMixer, _act_max, 2.0f),
    AP_GROUPINFO("LOG", 13, AP_NNMixer, _log, 1),
    AP_GROUPINFO("HOLD_MODE", 14, AP_NNMixer, _hold_mode, 4),
    AP_GROUPINFO("SRV_FN0", 15, AP_NNMixer, _servo_fn0, 94),
    AP_GROUPINFO("HIL_ATT", 16, AP_NNMixer, _hil_att, 0),

    // @Param: BLEND_MS
    // @DisplayName: Policy switch blend time
    // @Units: ms
    // @User: Advanced
    AP_GROUPINFO("BLEND_MS", 18, AP_NNMixer, _blend_ms, 500),

    // @Param: CLOCK_HZ
    // @DisplayName: Gesture clock frequency
    // @Description: Fills the two observation channels after the twist with sin and cos of a phase advancing at this rate, for policies trained on a timed motion (a wave). 0 leaves the channels at zero. Only used when the robot observation has extra channels.
    // @Units: Hz
    // @Range: 0 2
    // @User: Advanced
    AP_GROUPINFO("CLOCK_HZ", 19, AP_NNMixer, _clock_hz, 0.0f),

    // @Param: POSE_WD
    // @DisplayName: Pose-command watchdog
    // @Description: Age limit for a MAVLink NNM_POSE stream (DEBUG_FLOAT_ARRAY). While fresh, the eight channels after the clock are low-pass filtered into the observation. When stale, they ease to zero (rest pose) with the same filter — never a hard cut.
    // @Units: ms
    // @Range: 50 5000
    // @User: Advanced
    AP_GROUPINFO("POSE_WD", 20, AP_NNMixer, _pose_wd_ms, 500),

    // @Param: POSE_TAU
    // @DisplayName: Pose-command low-pass time constant
    // @Description: First-order filter on the NNM_POSE channels before they enter the observation. Matches the pose_cmd training domain.
    // @Units: s
    // @Range: 0.02 1.0
    // @User: Advanced
    AP_GROUPINFO("POSE_TAU", 21, AP_NNMixer, _pose_tau, 0.15f),

    // @Param: CLOCK_AUTO
    // @DisplayName: Gait clock only while commanded
    // @Description: 1: the NNM_CLOCK_HZ phase runs only while the commanded twist is non-zero and the clock channels are zero when standing (Booster Gym convention, locomotion policies trained with env.gait_clock). 0: the clock always runs (clip-imitation gestures).
    // @Values: 0:Always,1:WhileCommanded
    // @User: Advanced
    AP_GROUPINFO("CLOCK_AUTO", 22, AP_NNMixer, _clock_auto, 0),

    // @Param: GETUP_IDX
    // @DisplayName: Auto get-up policy index
    // @Description: Alphabetical index of the get-up .nnm under /APM/nnm/<robot>/policies/. When the trunk tilt exceeds NNM_GETUP_TILT the mixer switches to this policy (even while armed in MANUAL), runs a one-shot clock of NNM_GETUP_S seconds, then returns to the previous policy after NNM_GETUP_HOLD of upright. -1 disables.
    // @Range: -1 15
    // @User: Advanced
    AP_GROUPINFO("GETUP_IDX", 23, AP_NNMixer, _getup_idx, -1),

    // @Param: GETUP_TILT
    // @DisplayName: Get-up entry tilt
    // @Description: Trunk tilt (deg from upright) that must be held for 0.3 s to trigger auto get-up.
    // @Units: deg
    // @Range: 30 90
    // @User: Advanced
    AP_GROUPINFO("GETUP_TILT", 24, AP_NNMixer, _getup_tilt, 60.0f),

    // @Param: GETUP_OK
    // @DisplayName: Get-up exit tilt
    // @Description: Trunk tilt (deg) below which the robot is considered upright after get-up.
    // @Units: deg
    // @Range: 5 30
    // @User: Advanced
    AP_GROUPINFO("GETUP_OK", 25, AP_NNMixer, _getup_ok, 12.0f),

    // @Param: GETUP_S
    // @DisplayName: Get-up clip duration
    // @Description: One-shot clock length matching the trained get-up clip (phase 0→π). Microban getup.nnm: 14.2 s (Stage-I motion slowed 2x plus the blend to q0). The previous policy is restored only after the whole clip has played.
    // @Units: s
    // @Range: 1 30
    // @User: Advanced
    AP_GROUPINFO("GETUP_S", 26, AP_NNMixer, _getup_s, 14.2f),

    // @Param: GETUP_HOLD
    // @DisplayName: Get-up upright hold
    // @Description: Time the trunk must stay under NNM_GETUP_OK before returning to the previous policy.
    // @Units: s
    // @Range: 0.2 5
    // @User: Advanced
    AP_GROUPINFO("GETUP_HOLD", 27, AP_NNMixer, _getup_hold, 1.5f),

    // @Param: OUT
    // @DisplayName: Joint output
    // @Description: Where the joint targets go. 0: the servo PWM functions only (simulator, PWM servos). 1: also the joint bus, an actuator link on the serial port with SERIALn_PROTOCOL 51 (robotic servos behind the Arduino Uno Q microcontroller). With the bus running, joint position and velocity come from it.
    // @Values: 0:PWM,1:PWM and joint bus
    // @User: Advanced
    // @RebootRequired: True
    AP_GROUPINFO("OUT", 28, AP_NNMixer, _out, 0),

    // @Param: DIS_POSE
    // @DisplayName: Hold the default pose while disarmed
    // @Description: 0: the outputs are off while disarmed. 1: the joints hold the default pose of the selected policy while disarmed, reached with a one-second ramp from the measured angles, so the robot stands balanced and arming starts the policy from that pose. Torque can be released on request (NNM_SRV operation 1) until the pose is requested again (operation 2) or the vehicle is armed.
    // @Values: 0:Outputs off,1:Hold default pose
    // @User: Advanced
    AP_GROUPINFO("DIS_POSE", 29, AP_NNMixer, _dis_pose, 0),

    // @Param: DEPLOY
    // @DisplayName: Deployment chain
    // @Description: Signal chain around the policy. 0: standard NNMixer chain (complementary gravity filter with NNM_ATT_TAU, raw gyro and joint velocity, action applied as is, cross-fade on a policy switch, get-up after 0.3 s over NNM_GETUP_TILT). 1: XDuck chain, the processing of the original XGO Duck controller (gravity filter with accelerometer trust weighting, smoothed gyro and joint velocity, per-policy low-pass on the action, heading hold while translating, NNM_XD_ACCX trim, policy switch with a clean history and no cross-fade, get-up after 0.15 s followed by one second in the default pose). A change takes effect while disarmed.
    // @Values: 0:Standard,1:XDuck
    // @User: Advanced
    AP_GROUPINFO("DEPLOY", 30, AP_NNMixer, _deploy, 0),

    // @Param: XD_ACCX
    // @DisplayName: XDuck chain accelerometer trim
    // @Description: With NNM_DEPLOY 1, offset added to the forward accelerometer axis ahead of the gravity filter while walking: positive when moving forward, negative when moving backward or turning in place. It leans the estimated attitude against the commanded motion. 0 disables it.
    // @Units: m/s/s
    // @Range: 0 3
    // @User: Advanced
    AP_GROUPINFO("XD_ACCX", 31, AP_NNMixer, _xd_accx, 1.5f),

    // @Param: GAINS
    // @DisplayName: Publish servo gains
    // @Description: With a skills.bin version 2 table, sends the per-servo KP/KD to the joint bus MCU while disarmed; the MCU applies them at ARM. 0 never sends the table: the servos keep their values, except the mouth which the MCU sets to 10/20.
    // @Values: 0:Off,1:On
    // @User: Advanced
    AP_GROUPINFO("GAINS", 32, AP_NNMixer, _gains, 1),

    AP_GROUPEND
};

AP_NNMixer *AP_NNMixer::_singleton;

static uint32_t wall_micros()
{
#if CONFIG_HAL_BOARD == HAL_BOARD_SITL
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return uint32_t(uint64_t(ts.tv_sec) * 1000000ULL + uint64_t(ts.tv_nsec) / 1000ULL);
#else
    return AP_HAL::micros();
#endif
}

AP_NNMixer::AP_NNMixer()
{
    if (_singleton != nullptr) {
        AP_HAL::panic("AP_NNMixer must be singleton");
    }
    _singleton = this;
    AP_Param::setup_object_defaults(this, var_info);
    memset(_nav_twist, 0, sizeof(_nav_twist));
    _active_slot = 0;
    _use_sd_policy = false;
    _use_baked_mlp = false;
    _loaded_policy_idx = -1;
    _load_state = LoadState::IDLE;
    _rejected_idx = -1;
    _last_want = -1;
    _io_registered = false;
    _blending = false;
    _clock_phase = 0.0f;
    _clock_oneshot = false;
    _pose_n = 0;
    _pose_ms = 0;
    memset(_pose_cmd, 0, sizeof(_pose_cmd));
    memset(_pose_filt, 0, sizeof(_pose_filt));
    _getup_state = GetupState::IDLE;
    _getup_prev_idx = -1;
    _getup_tilt_ms = 0;
    _getup_ok_ms = 0;
    _getup_start_ms = 0;
    _getup_preload_pending = false;
    _force_switch = false;
    _bus_ready = false;
    _gains_done = false;
    _gains_ms = 0;
    _gains_seq = 0;
    _bus_fb_ms = 0;
    _torque_off = false;
    _out_active = false;
    _out_armed = false;
    _ramp_ms = 0;
    _hold_idx = -1;
    _aux_cmd = 0.0f;
    _chain = 0;
    _xd_valid = false;
    _xd_accx_cmd = 0.0f;
    _hh_walking = false;
    _hh_valid = false;
    _hh_has_target = false;
    _skill_state = SkillState::IDLE;
    _skill_idx = 0;
    memset(&_deploy_cfg, 0, sizeof(_deploy_cfg));
    memset(_act_filt, 0, sizeof(_act_filt));
}

void AP_NNMixer::set_nav_twist(float vx, float vy, float wz)
{
    _nav_twist[0] = vx;
    _nav_twist[1] = vy;
    _nav_twist[2] = wz;
    _nav_twist_ms = AP_HAL::millis();
}

void AP_NNMixer::set_pose_cmd(const float *values, uint8_t n)
{
    if (values == nullptr || n == 0) {
        return;
    }
    // contract ranges: arms offsets, knee_bend 0..0.55, sway ±0.15
    static const float lo[8] = {-1.55f, -1.15f, -1.35f, -1.55f, -0.05f, -1.35f, 0.0f, -0.15f};
    static const float hi[8] = { 0.15f,  0.05f, -0.05f,  0.15f,  1.15f, -0.05f, 0.55f,  0.15f};
    WITH_SEMAPHORE(_pose_sem);
    _pose_n = MIN(n, (uint8_t)8);
    for (uint8_t i = 0; i < _pose_n; i++) {
        if (_deploy_cfg.valid) {
            // the robot's own symmetric limits (skills.bin); 0 = channel not used
            const float lim = _deploy_cfg.pose_limit[i];
            _pose_cmd[i] = constrain_float(values[i], -lim, lim);
        } else {
            _pose_cmd[i] = constrain_float(values[i], lo[i], hi[i]);
        }
    }
    for (uint8_t i = _pose_n; i < 8; i++) {
        _pose_cmd[i] = 0.0f;
    }
    _pose_ms = AP_HAL::millis();
}

bool AP_NNMixer::ensure_slots_allocated()
{
    for (uint8_t i = 0; i < 2; i++) {
        if (_slot[i].blob != nullptr) {
            continue;
        }
        _slot[i].blob = (uint8_t *)malloc(NNMIXER_POLICY_SLOT_BYTES);
        if (_slot[i].blob == nullptr) {
            GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: slot %u malloc failed", unsigned(i));
            return false;
        }
        _slot[i].blob_cap = NNMIXER_POLICY_SLOT_BYTES;
        _slot[i].blob_len = 0;
        _slot[i].ready = false;
    }
    return true;
}

bool AP_NNMixer::load_robot_topology()
{
    const char *name = nnm_robot_name(NNM_RobotId(_robot.get()));
    if (name == nullptr) {
        return false;
    }
    char path[64];
    hal.util->snprintf(path, sizeof(path), "APM/nnm/%s/robot.bin", name);
    auto *fd = AP::FS().load_file(path);
    if (fd == nullptr) {
        // SITL / alternate mount
        hal.util->snprintf(path, sizeof(path), "/APM/nnm/%s/robot.bin", name);
        fd = AP::FS().load_file(path);
    }
    if (fd == nullptr) {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0) {
            // MicroDuck baked fallback when SD has no robot.bin
            memset(&_topo, 0, sizeof(_topo));
            strncpy(_topo.robot_id, "microduck", sizeof(_topo.robot_id) - 1);
            _topo.n_joints = MLP_ACT_DIM;
            _topo.obs_dim = MLP_OBS_DIM;
            _topo.rate_hz = 50;
            _topo.servo_fn0 = uint16_t(_servo_fn0.get());
            for (uint16_t i = 0; i < _topo.n_joints && i < NNM_MAX_JOINTS; i++) {
                _topo.q0[i] = mlp_default_pose[i];
            }
            _topo.valid = true;
            _use_baked_mlp = true;
            _use_sd_policy = false;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: baked MicroDuck topology (no SD robot.bin)");
            return true;
        }
#endif
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: missing %s", path);
        return false;
    }
    const bool ok = nnm_parse_robot_bin(fd->data, fd->length, _topo);
    delete fd;
    if (!ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: bad robot.bin for %s", name);
        return false;
    }
    // the policy task runs at 50 Hz; the robot rate must be an integer divisor of it
    if (_topo.rate_hz == 0 || 50 % _topo.rate_hz != 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: rate %u Hz does not divide 50", unsigned(_topo.rate_hz));
        _topo.valid = false;
        return false;
    }
    // Param servo offset may override the file
    if (_servo_fn0.get() > 0) {
        _topo.servo_fn0 = uint16_t(_servo_fn0.get());
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: robot %s joints=%u obs=%u",
                  _topo.robot_id, unsigned(_topo.n_joints), unsigned(_topo.obs_dim));
    return true;
}

// Reads /APM/nnm/<robot>/policies/<index-th .nnm> into a slot. Touches only that slot, so it
// runs on the IO thread while the policy task keeps using the other slot.
bool AP_NNMixer::load_policy_index(int8_t index, uint8_t into_slot)
{
    if (!_topo.valid || into_slot > 1 || _slot[into_slot].blob == nullptr) {
        return false;
    }
    const char *name = _topo.robot_id;
    char dirpath[64];
    hal.util->snprintf(dirpath, sizeof(dirpath), "APM/nnm/%s/policies", name);

    auto *dir = AP::FS().opendir(dirpath);
    if (dir == nullptr) {
        hal.util->snprintf(dirpath, sizeof(dirpath), "/APM/nnm/%s/policies", name);
        dir = AP::FS().opendir(dirpath);
    }
    if (dir == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NNMixer: no %s/policies", name);
        return false;
    }

    // Collect .nnm names (alphabetical via insertion into small list)
    char names[8][32] {};
    uint8_t nfiles = 0;
    for (struct dirent *de = AP::FS().readdir(dir); de && nfiles < 8; de = AP::FS().readdir(dir)) {
        const char *fn = de->d_name;
        const size_t len = strlen(fn);
        if (len < 5 || len >= 32) {
            continue;
        }
        if (strcmp(fn + len - 4, ".nnm") != 0) {
            continue;
        }
        // insert sorted
        uint8_t at = nfiles;
        while (at > 0 && strcmp(names[at - 1], fn) > 0) {
            memcpy(names[at], names[at - 1], 32);
            at--;
        }
        strncpy(names[at], fn, 31);
        nfiles++;
    }
    AP::FS().closedir(dir);

    if (nfiles == 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NNMixer: no .nnm for %s", name);
        return false;
    }
    if (index < 0 || index >= int8_t(nfiles)) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: policy %d not in 0..%u", int(index), unsigned(nfiles - 1));
        return false;
    }

    char path[96];
    hal.util->snprintf(path, sizeof(path), "%s/%s", dirpath, names[index]);
    auto *fd = AP::FS().load_file(path);
    if (fd == nullptr) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: cannot read %s", names[index]);
        return false;
    }
    const bool ok = nnm_parse_policy_nnm(fd->data, fd->length, _topo, _slot[into_slot]);
    delete fd;
    if (!ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: rejected %s", names[index]);
        return false;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: slot %u <- %s", unsigned(into_slot), names[index]);
    return true;
}

// IO thread: fill the free slot on request; never touches the active slot
void AP_NNMixer::io_update()
{
    int8_t idx;
    uint8_t slot;
    {
        WITH_SEMAPHORE(_load_sem);
        if (_load_state != LoadState::REQUESTED) {
            return;
        }
        idx = _load_idx;
        slot = _load_slot;
        _load_state = LoadState::LOADING;
    }
    const bool ok = load_policy_index(idx, slot);
    WITH_SEMAPHORE(_load_sem);
    _load_state = ok ? LoadState::DONE_OK : LoadState::DONE_FAIL;
}

// policy task: request loads and flip slots only while standing still (disarmed or HOLD),
// unless _force_switch is set by the automatic get-up state machine.
void AP_NNMixer::service_policy_switch()
{
    const AP_Vehicle *veh = AP::vehicle();
    const uint8_t mode = (veh != nullptr) ? veh->get_mode() : kModeHold;
    const bool disarmed = !hal.util->get_soft_armed();
    const bool hold = (_hold_mode >= 0 && mode == uint8_t(_hold_mode)) || mode == kModeHold;
    // a skill switches while armed, on entry and on the way back
    const bool skill_switch = _skill_state == SkillState::STARTING || _skill_state == SkillState::ENDING;
    // the XDuck chain changes policy on the move (walk <-> roller), between skills
    const bool live_switch = _chain == 1 && _skill_state == SkillState::IDLE && _getup_state == GetupState::IDLE;
    const bool allowed = disarmed || hold || _force_switch || skill_switch || live_switch;
    // XDuck chain and skills: no cross-fade, clean action history
    const bool hard = (_chain == 1) || skill_switch;
    const bool getting_up = _getup_state == GetupState::ACTIVE || _getup_state == GetupState::FALLING;

    // preferred target: get-up override, then a running skill, else the user NNM_POLICY
    int8_t want = int8_t(_policy.get());
    if (_skill_state == SkillState::STARTING || _skill_state == SkillState::RUNNING) {
        want = int8_t(_deploy_cfg.skill[_skill_idx].policy);
    }
    if (getting_up && _getup_idx >= 0) {
        want = int8_t(_getup_idx.get());
    } else if (_getup_state == GetupState::RECOVERED && _getup_prev_idx >= 0) {
        want = _getup_prev_idx;
    }
    if (want != _last_want) {
        _last_want = want;
        _rejected_idx = -1;
    }

    WITH_SEMAPHORE(_load_sem);
    switch (_load_state) {
    case LoadState::REQUESTED:
    case LoadState::LOADING:
        return;
    case LoadState::DONE_FAIL:
        _rejected_idx = _load_idx;
        _load_state = LoadState::IDLE;
        _force_switch = false;
        if (skill_switch) {
            _skill_state = SkillState::IDLE;
        }
        return;
    case LoadState::DONE_OK:
        if (!allowed) {
            return;
        }
        memcpy(_act_blend_from, _last_action, sizeof(_act_blend_from));
        _blend_start_ms = AP_HAL::millis();
        _blending = !disarmed && !hard;
        if (hard) {
            reset_history();
        }
        _active_slot = _load_slot;
        _loaded_policy_idx = _load_idx;
        _use_sd_policy = true;
        _use_baked_mlp = false;
        _load_state = LoadState::IDLE;
        _force_switch = false;
        if (_skill_state == SkillState::STARTING) {
            if (_loaded_policy_idx == int8_t(_deploy_cfg.skill[_skill_idx].policy)) {
                // the skill clock starts when its policy takes over
                _skill_state = SkillState::RUNNING;
                _skill_start_ms = AP_HAL::millis();
            }
        } else if (_skill_state == SkillState::ENDING) {
            _skill_state = SkillState::IDLE;
        }
        if (getting_up && _loaded_policy_idx == int8_t(_getup_idx.get())) {
            // oneshot clock restarts when get-up becomes the active policy
            _clock_phase = 0.0f;
            _clock_oneshot = true;
            _getup_start_ms = AP_HAL::millis();
            _getup_ok_ms = 0;
            memset(_pose_cmd, 0, sizeof(_pose_cmd));
            memset(_pose_filt, 0, sizeof(_pose_filt));
            _pose_ms = 0;
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: get-up active");
        } else if (_getup_state == GetupState::RECOVERED) {
            _clock_oneshot = false;
            _clock_phase = 0.0f;
            _getup_state = GetupState::IDLE;
            _getup_preload_pending = (_getup_idx >= 0);
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: get-up done, policy %d", (int)_loaded_policy_idx);
        } else if (_getup_preload_pending && _getup_idx >= 0
                   && _loaded_policy_idx != int8_t(_getup_idx.get())) {
            // after a manual switch, refill the free slot with get-up for a fast fall response
            _getup_preload_pending = false;
            _load_idx = int8_t(_getup_idx.get());
            _load_slot = _active_slot ^ 1;
            _load_state = LoadState::REQUESTED;
        }
        return;
    case LoadState::IDLE:
        break;
    }
    if (want == _loaded_policy_idx || want == _rejected_idx || !allowed) {
        // keep get-up preloaded in the free slot while idle
        if (_getup_idx >= 0 && !_getup_preload_pending
            && _loaded_policy_idx != int8_t(_getup_idx.get())
            && !_slot[_active_slot ^ 1].ready) {
            _load_idx = int8_t(_getup_idx.get());
            _load_slot = _active_slot ^ 1;
            _load_state = LoadState::REQUESTED;
        }
        return;
    }
    _load_idx = want;
    _load_slot = _active_slot ^ 1;
    _load_state = LoadState::REQUESTED;
}

float AP_NNMixer::trunk_tilt_deg() const
{
    if (_chain == 1) {
        // XDuck chain: angle from upright over the whole 0..180 range, upside down included
        if (!_xd_valid) {
            return 0.0f;
        }
        return degrees(acosf(constrain_float(-_xd_grav.z, -1.0f, 1.0f)));
    }
    // gravity_flu ≈ (0,0,−1) when upright → tilt = acos(|g_z|)
    float g[3];
    Vector3f down = _down_body;
    if (_att_src == 1) {
        Quaternion q;
        AP::ahrs().get_quat_body_to_ned(q);
        Matrix3f m;
        q.rotation_matrix(m);
        down = m.mul_transpose(Vector3f(0, 0, 1));
    }
    g[0] = -down.x;
    g[1] = -down.y;
    g[2] = -down.z;
    const float n = safe_sqrt(g[0]*g[0] + g[1]*g[1] + g[2]*g[2]);
    if (n < 1e-6f) {
        return 0.0f;
    }
    const float gz = constrain_float(g[2] / n, -1.0f, 1.0f);
    return degrees(acosf(fabsf(gz)));
}

void AP_NNMixer::service_getup()
{
    if (_getup_idx < 0) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    const float tilt = trunk_tilt_deg();
    const bool xd = (_chain == 1);
    const int8_t getup = int8_t(_getup_idx.get());

    switch (_getup_state) {
    case GetupState::IDLE: {
        // a skill can suspend the recovery: a roll goes through every attitude on purpose
        const bool suspended = _skill_state != SkillState::IDLE &&
                               (_deploy_cfg.skill[_skill_idx].flags & NNM_SKILL_NO_GETUP) != 0;
        if (tilt > _getup_tilt && !suspended) {
            if (_getup_tilt_ms == 0) {
                _getup_tilt_ms = now;
            } else if (now - _getup_tilt_ms >= (xd ? 150U : 300U)) {
                _getup_prev_idx = _loaded_policy_idx;
                if (_skill_state != SkillState::IDLE || _getup_prev_idx == getup) {
                    // never come back into a skill or into get-up itself
                    _getup_prev_idx = int8_t(_policy.get());
                }
                _skill_state = SkillState::IDLE;
                // XDuck chain: one second in the default pose with a clean history comes first
                _getup_state = xd ? GetupState::FALLING : GetupState::ACTIVE;
                _xd_prehold_ms = now;
                _force_switch = true;
                _getup_tilt_ms = 0;
                _getup_ok_ms = 0;
                // load get-up into the free slot; the flip happens on the DONE_OK path
                WITH_SEMAPHORE(_load_sem);
                if (_load_state == LoadState::IDLE) {
                    _load_idx = getup;
                    _load_slot = _active_slot ^ 1;
                    _load_state = LoadState::REQUESTED;
                }
            }
        } else {
            _getup_tilt_ms = 0;
        }
        break;
    }
    case GetupState::FALLING:
        if (!xd) {
            _getup_state = GetupState::ACTIVE;
            break;
        }
        if (_loaded_policy_idx != getup) {
            // still waiting for the slot flip
            _force_switch = true;
        }
        if (now - _xd_prehold_ms >= 1000U) {
            _getup_state = GetupState::ACTIVE;
            _getup_ok_ms = 0;
        }
        break;
    case GetupState::ACTIVE:
        if (_loaded_policy_idx != getup) {
            // still waiting for the slot flip
            _force_switch = true;
            break;
        }
        if (tilt < _getup_ok) {
            if (_getup_ok_ms == 0) {
                _getup_ok_ms = now;
            } else if (now - _getup_ok_ms >= (uint32_t)(_getup_hold * 1000.0f)
                       && (xd || now - _getup_start_ms >= (uint32_t)(_getup_s * 1000.0f))) {
                // held upright long enough. Standard chain: the whole clip has played too (its
                // tail blends the discovered stand into q0, which the walk policy can take over).
                // Restore the previous policy.
                _getup_state = GetupState::RECOVERED;
                _force_switch = true;
                _clock_oneshot = false;
                WITH_SEMAPHORE(_load_sem);
                if (_load_state == LoadState::IDLE && _getup_prev_idx >= 0
                    && _getup_prev_idx != _loaded_policy_idx) {
                    _load_idx = _getup_prev_idx;
                    _load_slot = _active_slot ^ 1;
                    _load_state = LoadState::REQUESTED;
                }
            }
        } else {
            _getup_ok_ms = 0;
        }
        break;
    case GetupState::RECOVERED:
        // waiting for service_policy_switch to finish the restore
        _force_switch = true;
        break;
    }
}

void AP_NNMixer::request_policy_load(int8_t idx, bool force)
{
    WITH_SEMAPHORE(_load_sem);
    if (_load_state != LoadState::IDLE) {
        return;
    }
    _force_switch = force;
    _load_idx = idx;
    _load_slot = _active_slot ^ 1;
    _load_state = LoadState::REQUESTED;
}

void AP_NNMixer::init()
{
    memset(_last_action, 0, sizeof(_last_action));
    _down_body = Vector3f(0, 0, 1);
    _down_valid = false;
    _joint_count = 0;
    _joint_ms = 0;
    _hil_state_ms = 0;
    memset(_hil_gyro_flu, 0, sizeof(_hil_gyro_flu));
    memset(_hil_gravity_flu, 0, sizeof(_hil_gravity_flu));
    _tick = 0;
    _fail_reason = 5;
    _forward_us = 0;
    _last_telem_ms = 0;
    memset(_obs, 0, sizeof(_obs));
    memset(_act, 0, sizeof(_act));
    _blending = false;

    _load_state = LoadState::IDLE;
    _last_want = int8_t(_policy.get());
    _rejected_idx = -1;
    if (!load_robot_topology()) {
        _initialised = true;
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: init failed (no topology)");
        return;
    }
    load_deploy_table();
    _chain = uint8_t(constrain_int16(_deploy.get(), 0, 1));
#if AP_JOINTBUS_ENABLED
    if (_out == 1) {
        _bus_ready = _bus.init(uint8_t(_topo.n_joints));
        if (!_bus_ready) {
            GCS_SEND_TEXT(MAV_SEVERITY_WARNING, "NNMixer: NNM_OUT 1 but no joint bus (SERIALn_PROTOCOL 51)");
        }
    }
#endif
    // boot load runs here, disarmed, before the task drives anything; later loads use the IO thread
    if (!_use_baked_mlp && ensure_slots_allocated() && load_policy_index(_policy.get(), 0)) {
        _active_slot = 0;
        _loaded_policy_idx = _policy.get();
        _use_sd_policy = true;
    } else {
#if AP_NNMIXER_BAKED_MLP_ENABLED
        if (_robot == 0) {
            _use_baked_mlp = true;
            _use_sd_policy = false;
            _loaded_policy_idx = _policy.get() == 0 ? 0 : -1;
        }
#endif
        if (!_use_baked_mlp && !_use_sd_policy) {
            _rejected_idx = _last_want;
        }
    }
    if (!_io_registered) {
        hal.scheduler->register_io_process(FUNCTOR_BIND_MEMBER(&AP_NNMixer::io_update, void));
        _io_registered = true;
    }
    _initialised = true;
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: %s policy %d %s", _topo.robot_id, int(_loaded_policy_idx),
                  _use_sd_policy ? "int8 SD" : (_use_baked_mlp ? "float32 flash" : "none"));
}

const float *AP_NNMixer::default_pose() const
{
    if (_use_sd_policy && _slot[_active_slot].ready && _slot[_active_slot].view.default_pose != nullptr) {
        return _slot[_active_slot].view.default_pose;
    }
    if (_topo.valid) {
        return _topo.q0;
    }
#if AP_NNMIXER_BAKED_MLP_ENABLED
    return mlp_default_pose;
#else
    return _topo.q0;
#endif
}

void AP_NNMixer::update_attitude()
{
    if (!enabled()) {
        return;
    }
    if (!_initialised) {
        init();
    }
    {
        const uint32_t now_us = AP_HAL::micros();
        if (_loop_last_us != 0) {
            _loop_gap_us = MAX(_loop_gap_us, now_us - _loop_last_us);
        }
        _loop_last_us = now_us;
        _loop_count++;
    }
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f gyro = ins.get_gyro();
    const Vector3f accel = ins.get_accel();
    const float dt = ins.get_loop_delta_t();
    if (_chain == 1) {
        bool from_link = false;
#if AP_INERTIALSENSOR_JOINTBUS_ENABLED
        // the XDuck filters run once per IMU sample of the microcontroller (100 Hz) with the
        // sampling interval of its clock, as in the original, whatever the main loop rate
        AP_InertialSensor_JointBus::Sample smp[8];
        const uint8_t n = AP_InertialSensor_JointBus::pop_samples(smp, ARRAY_SIZE(smp));
        for (uint8_t i = 0; i < n; i++) {
            const float sdt = _xd_src_valid ? float(uint32_t(smp[i].source_us - _xd_src_us)) * 1.0e-6f : 0.01f;
            _xd_src_us = smp[i].source_us;
            _xd_src_valid = true;
            update_attitude_xduck(smp[i].gyro, smp[i].accel, sdt);
        }
        if (n > 0) {
            _xd_src_ms = AP_HAL::millis();
        }
        from_link = AP_InertialSensor_JointBus::link_alive();
        if (!from_link) {
            _xd_src_valid = false;
        }
#endif
        if (!from_link) {
            update_attitude_xduck(gyro, accel, dt);
        }
    }
    const float an = accel.length();
    if (!_down_valid) {
        if (an > 1.0f) {
            _down_body = -accel / an;
            _down_valid = true;
        }
        return;
    }
    Vector3f d = _down_body - (gyro % _down_body) * dt;
    if (an > 0.5f * GRAVITY_MSS && an < 1.5f * GRAVITY_MSS && _att_tau > 0.0f) {
        const float alpha = constrain_float(dt / _att_tau, 0.0f, 1.0f);
        d = d * (1.0f - alpha) + (-accel / an) * alpha;
    }
    const float n = d.length();
    if (n > 1e-6f) {
        _down_body = d / n;
    }
}

void AP_NNMixer::gravity_body_flu(float g[3])
{
    if (_chain == 1) {
        g[0] = _xd_valid ? _xd_grav.x : 0.0f;
        g[1] = _xd_valid ? _xd_grav.y : 0.0f;
        g[2] = _xd_valid ? _xd_grav.z : -1.0f;
        return;
    }
    Vector3f down = _down_body;
    if (_att_src == 1) {
        Quaternion q;
        AP::ahrs().get_quat_body_to_ned(q);
        Matrix3f m;
        q.rotation_matrix(m);
        down = m.mul_transpose(Vector3f(0, 0, 1));
    }
    g[0] = down.x;
    g[1] = -down.y;
    g[2] = -down.z;
}

void AP_NNMixer::set_joint_feedback(const float *pos, const float *vel, uint8_t count)
{
    WITH_SEMAPHORE(_joint_sem);
    const uint8_t nmax = _topo.valid ? uint8_t(_topo.n_joints) : uint8_t(NNM_MAX_JOINTS);
    const uint8_t n = MIN(count, nmax);
    for (uint8_t i = 0; i < n; i++) {
        _joint_pos[i] = pos[i];
        _joint_vel[i] = vel[i];
    }
    _joint_count = n;
    _joint_ms = AP_HAL::millis();
}

void AP_NNMixer::set_hil_state(const float *pos, const float *vel,
                                 const float *gyro_flu, const float *gravity_flu)
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 14;
    set_joint_feedback(pos, vel, n);
    WITH_SEMAPHORE(_joint_sem);
    memcpy(_hil_gyro_flu, gyro_flu, sizeof(_hil_gyro_flu));
    memcpy(_hil_gravity_flu, gravity_flu, sizeof(_hil_gravity_flu));
    _hil_state_ms = AP_HAL::millis();
}

bool AP_NNMixer::read_joint_feedback()
{
    const uint8_t need = _topo.valid ? uint8_t(_topo.n_joints) : 14;
    bool from_bus = false;
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        // the joint bus is the feedback source when it runs
        from_bus = true;
        AP_JointBus::Feedback fb;
        if (_bus.get_feedback(fb) && fb.ms != _bus_fb_ms) {
            _bus_fb_ms = fb.ms;
            set_joint_feedback(fb.pos, fb.vel, MIN(fb.count, need));
        }
    }
#endif
#if AP_NNMIXER_JOINT_FEEDBACK_SITL_ENABLED
    const SITL::SIM *sitl = AP::sitl();
    if (!from_bus && sitl != nullptr && sitl->state.joint_count >= need &&
        sitl->state.joint_time_us != _last_joint_time_us) {
        _last_joint_time_us = sitl->state.joint_time_us;
        set_joint_feedback(sitl->state.joint_pos, sitl->state.joint_vel, need);
    }
#endif
    (void)from_bus;
    WITH_SEMAPHORE(_joint_sem);
    if (_joint_count < need) {
        _fail_reason = 2;
        return false;
    }
    if (AP_HAL::millis() - _joint_ms > uint32_t(MAX(_wd_ms.get(), 1))) {
        _fail_reason = 3;
        return false;
    }
    return true;
}

void AP_NNMixer::read_twist(float twist[3])
{
    twist[0] = twist[1] = twist[2] = 0.0f;
    const AP_Vehicle *veh = AP::vehicle();
    const uint8_t mode = (veh != nullptr) ? veh->get_mode() : kModeManual;

    if (_hold_mode >= 0 && mode == uint8_t(_hold_mode)) {
        return;
    }
    if (mode == kModeHold) {
        return;
    }

    // Autopilot modes: use nav twist published by Rover (desired speed / turn rate)
    if (mode == kModeGuided || mode == kModeAuto || mode == kModeRtl || mode == kModeSmartRtl) {
        if (_nav_twist_ms != 0 && AP_HAL::millis() - _nav_twist_ms < 500) {
            twist[0] = constrain_float(_nav_twist[0], -_vx_max, _vx_max);
            twist[1] = constrain_float(_nav_twist[1], -_vy_max, _vy_max);
            twist[2] = constrain_float(_nav_twist[2], -_wz_max, _wz_max);
        }
        return;
    }

    // MANUAL (and other stick-driven modes): RC
    if (!rc().has_valid_input()) {
        return;
    }
    const struct {
        int8_t ch;
        float scale;
        float *dst;
    } map[3] = {
        { int8_t(_rc_vx), _vx_max, &twist[0] },
        { int8_t(_rc_vy), _vy_max, &twist[1] },
        { int8_t(_rc_wz), _wz_max, &twist[2] },
    };
    for (uint8_t i = 0; i < 3; i++) {
        if (map[i].ch < 1) {
            continue;
        }
        const RC_Channel *c = rc().channel(uint8_t(map[i].ch - 1));
        if (c == nullptr) {
            continue;
        }
        const float n = c->norm_input_dz();
        *map[i].dst = constrain_float(n, -1.0f, 1.0f) * map[i].scale;
    }
}

void AP_NNMixer::write_servos(const float *q_target)
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 0;
    const uint16_t fn0 = _topo.valid ? _topo.servo_fn0 : uint16_t(_servo_fn0.get());
    for (uint8_t i = 0; i < n; i++) {
        const int32_t pwm = NNM_PWM_CENTER + int32_t(roundf(q_target[i] / NNM_RAD_PER_US));
        const uint16_t p = uint16_t(constrain_int32(pwm, NNM_PWM_MIN, NNM_PWM_MAX));
        SRV_Channels::set_output_pwm(SRV_Channel::Function(fn0 + i), p);
    }
}

void AP_NNMixer::write_idle_servos()
{
    const uint8_t n = _topo.valid ? uint8_t(_topo.n_joints) : 0;
    const uint16_t fn0 = _topo.valid ? _topo.servo_fn0 : uint16_t(_servo_fn0.get());
    for (uint8_t i = 0; i < n; i++) {
        SRV_Channels::set_output_pwm(SRV_Channel::Function(fn0 + i), 0);
    }
}

// ---------------------------------------------------------------------------
// XDuck chain (NNM_DEPLOY 1): the receive-side processing of the original XGO Duck controller

// Loop-rate attitude: gravity predicted with the raw gyro and corrected by the accelerometer with
// a gain that fades as its magnitude leaves 1 g; smoothed gyro; yaw integration for heading hold.
// Robot frame, x forward, y left, z up; the accelerometer is used as a gravity vector.
void AP_NNMixer::update_attitude_xduck(const Vector3f &gyro_frd, const Vector3f &accel_frd, float dt_in)
{
    const float kGravityTau = 0.3f;         // s
    const float kGyroKeep = 0.5f;           // per 10 ms sample
    const Vector3f gyro(gyro_frd.x, -gyro_frd.y, -gyro_frd.z);
    Vector3f acc(-accel_frd.x, accel_frd.y, accel_frd.z);
    acc.x += _xd_accx_cmd;
    const float norm = acc.length();
    const bool acc_ok = norm > 1.0e-6f;
    const float dt = constrain_float(dt_in, 0.001f, 0.1f);

    if (!_xd_valid || dt_in > 0.1f) {
        // a long gap cannot be bridged with one rate sample: start again from the accelerometer
        _xd_grav = acc_ok ? acc / norm : Vector3f(0.0f, 0.0f, -1.0f);
        _xd_gyro = gyro;
        _xd_valid = true;
        _hh_valid = false;
        _hh_has_target = false;
        return;
    }
    // a world-fixed vector seen from the rotating body: dg/dt = -omega x g
    Vector3f g = _xd_grav - (gyro % _xd_grav) * dt;
    g.normalize();
    if (acc_ok) {
        const float trust = MAX(0.0f, 1.0f - fabsf(norm / GRAVITY_MSS - 1.0f) / 0.2f);
        const float gain = (1.0f - expf(-dt / kGravityTau)) * trust;
        g = g * (1.0f - gain) + (acc / norm) * gain;
        g.normalize();
    }
    _xd_grav = g;
    const float keep = powf(kGyroKeep, dt / 0.01f);
    _xd_gyro = _xd_gyro * keep + gyro * (1.0f - keep);

    // heading: unwrapped yaw from the body rate and gravity, started only when standing level
    if (!_hh_walking) {
        _hh_valid = false;
        _hh_has_target = false;
    } else if (!_hh_valid) {
        if (-_xd_grav.z >= cosf(radians(5.0f))) {
            _hh_yaw = 0.0f;
            _hh_valid = true;
        }
    } else {
        const float den = sq(_xd_grav.y) + sq(_xd_grav.z);
        if (den < 0.01f) {
            _hh_valid = false;
            _hh_has_target = false;
        } else {
            _hh_yaw -= (_xd_gyro.y * _xd_grav.y + _xd_gyro.z * _xd_grav.z) / den * dt;
        }
    }
}

// Yaw-rate command that keeps the heading while translating with no turn requested
float AP_NNMixer::heading_hold(bool walking, const float twist[3], float kp)
{
    const float kMaxWz = 0.3f;              // rad/s
    const float kDeadband = radians(3.0f);
    if (!walking || !_hh_valid || !is_zero(twist[2]) || (is_zero(twist[0]) && is_zero(twist[1]))) {
        _hh_has_target = false;
        return twist[2];
    }
    if (!_hh_has_target) {
        _hh_target = _hh_yaw;
        _hh_has_target = true;
    }
    const float err = _hh_target - _hh_yaw;
    return fabsf(err) > kDeadband ? constrain_float(kp * err, -kMaxWz, kMaxWz) : 0.0f;
}

// ---------------------------------------------------------------------------
// deployment table, skills

void AP_NNMixer::load_deploy_table()
{
    memset(&_deploy_cfg, 0, sizeof(_deploy_cfg));
    char path[64];
    hal.util->snprintf(path, sizeof(path), "APM/nnm/%s/skills.bin", _topo.robot_id);
    auto *fd = AP::FS().load_file(path);
    if (fd == nullptr) {
        hal.util->snprintf(path, sizeof(path), "/APM/nnm/%s/skills.bin", _topo.robot_id);
        fd = AP::FS().load_file(path);
    }
    if (fd == nullptr) {
        return;     // optional file
    }
    const bool ok = nnm_parse_skills_bin(fd->data, fd->length, _deploy_cfg);
    delete fd;
    if (!ok) {
        GCS_SEND_TEXT(MAV_SEVERITY_ERROR, "NNMixer: bad skills.bin for %s", _topo.robot_id);
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "NNMixer: %u skills, %u policy settings",
                  unsigned(_deploy_cfg.n_skill), unsigned(_deploy_cfg.n_policy));
}

const NNM_PolicyCfg *AP_NNMixer::policy_cfg(int8_t idx) const
{
    for (uint8_t i = 0; i < _deploy_cfg.n_policy; i++) {
        if (int8_t(_deploy_cfg.policy[i].policy) == idx) {
            return &_deploy_cfg.policy[i];
        }
    }
    return nullptr;
}

void AP_NNMixer::reset_history()
{
    memset(_last_action, 0, sizeof(_last_action));
    memset(_act_filt, 0, sizeof(_act_filt));
}

bool AP_NNMixer::start_skill(uint8_t id)
{
    if (!_initialised || !_topo.valid || !_deploy_cfg.valid || !hal.util->get_soft_armed()) {
        return false;
    }
    if (_getup_state != GetupState::IDLE) {
        return false;       // recovering from a fall
    }
    for (uint8_t i = 0; i < _deploy_cfg.n_skill; i++) {
        const NNM_Skill &sk = _deploy_cfg.skill[i];
        if (sk.id != id) {
            continue;
        }
        if (sk.needs != 0) {
            // follows another skill (stand after sit)
            if (_skill_state != SkillState::RUNNING || _deploy_cfg.skill[_skill_idx].id != sk.needs) {
                return false;
            }
        } else if (_skill_state != SkillState::IDLE) {
            return false;
        }
        _skill_idx = i;
        if (int8_t(sk.policy) == _loaded_policy_idx) {
            _skill_state = SkillState::RUNNING;
            _skill_start_ms = AP_HAL::millis();
            if (!(sk.flags & NNM_SKILL_KEEP_HISTORY)) {
                reset_history();
            }
        } else {
            // the policy is loaded into the free slot; the clock starts at the flip
            _skill_state = SkillState::STARTING;
        }
        return true;
    }
    return false;
}

void AP_NNMixer::end_skill()
{
    if (int8_t(_policy.get()) == _loaded_policy_idx) {
        _skill_state = SkillState::IDLE;
        reset_history();
    } else {
        // back to NNM_POLICY; IDLE when the slot flips
        _skill_state = SkillState::ENDING;
    }
}

void AP_NNMixer::service_skill()
{
    if (_skill_state != SkillState::RUNNING) {
        return;
    }
    const NNM_Skill &sk = _deploy_cfg.skill[_skill_idx];
    if (is_positive(sk.duration_s) &&
        AP_HAL::millis() - _skill_start_ms >= uint32_t(sk.duration_s * 1000.0f)) {
        end_skill();
    }
}

// ---------------------------------------------------------------------------
// output stage, joint bus, setup service

static void send_float_array(const char *name, uint16_t id, const float *data, uint8_t n)
{
    mavlink_debug_float_array_t pkt {};
    pkt.time_usec = AP_HAL::micros64();
    strncpy(pkt.name, name, sizeof(pkt.name));
    pkt.array_id = id;
    memcpy(pkt.data, data, sizeof(float) * MIN(n, uint8_t(ARRAY_SIZE(pkt.data))));
    gcs().send_to_active_channels(MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY, (const char *)&pkt);
}

// Drive the joints to q_des. When the outputs come on, or the bus actuators are not powered yet,
// the command starts from the measured angles and reaches q_des with a ramp (1 s disarmed, 0.5 s
// armed); while disarmed a change of the pose to hold is ramped as well.
void AP_NNMixer::drive_outputs(const float *q_des, bool armed)
{
    const uint8_t n = uint8_t(_topo.n_joints);
    const uint32_t now = AP_HAL::millis();
    bool have_meas = false;
    bool powered = true;
    float meas[NNM_MAX_JOINTS] {};
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        AP_JointBus::Feedback fb;
        if (_bus.get_feedback(fb) && fb.count >= n) {
            memcpy(meas, fb.pos, sizeof(float) * n);
            have_meas = true;
        }
        powered = _bus.torque_enabled();
    }
#endif
    if (!_out_active || !powered) {
        if (have_meas) {
            memcpy(_ramp_from, meas, sizeof(float) * n);
            _ramp_ms = armed ? 500 : 1000;
            _ramp_start_ms = now;
        } else if (!_out_active) {
            _ramp_ms = 0;
        }
    } else if (!armed && (_out_armed || _hold_idx != _loaded_policy_idx)) {
        // just disarmed, or the disarmed pose changed with the policy: move there from the
        // last command
        memcpy(_ramp_from, _out_q, sizeof(float) * n);
        _ramp_ms = 1000;
        _ramp_start_ms = now;
    }
    _hold_idx = _loaded_policy_idx;
    _out_armed = armed;

    float q[NNM_MAX_JOINTS];
    float k = 1.0f;
    if (_ramp_ms != 0) {
        k = constrain_float(float(now - _ramp_start_ms) / float(_ramp_ms), 0.0f, 1.0f);
        if (k >= 1.0f) {
            _ramp_ms = 0;
        }
    }
    for (uint8_t i = 0; i < n; i++) {
        q[i] = (k < 1.0f) ? _ramp_from[i] + k * (q_des[i] - _ramp_from[i]) : q_des[i];
        _out_q[i] = q[i];
    }
    _out_active = true;
    write_servos(q);
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        _bus.write(q, n, true);
    }
#endif
}

void AP_NNMixer::outputs_off()
{
    write_idle_servos();
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        _bus.write(nullptr, 0, false);
    }
#endif
    _out_active = false;
    _ramp_ms = 0;
}

void AP_NNMixer::send_service_reply(uint16_t seq, uint8_t op, bool ok, uint8_t id)
{
    float d[11] {};
    d[0] = seq;
    d[1] = op;
    d[2] = ok ? 1.0f : 0.0f;
    d[3] = id;
    d[10] = 1.0f;       // answered by the autopilot
    send_float_array("NNM_SRVR", seq, d, ARRAY_SIZE(d));
}

// Sends the run-gain table of the robot (skills.bin version 2) while disarmed, until the MCU answers
// ok. The MCU keeps it in RAM and applies it at ARM, so a table sent before arming is what runs.
void AP_NNMixer::publish_run_gains()
{
#if AP_JOINTBUS_ENABLED
    if (!_bus_ready || _gains == 0 || _gains_done || !_deploy_cfg.servo_gains_valid ||
        hal.util->get_soft_armed()) {
        return;
    }
    const uint32_t now = AP_HAL::millis();
    if (_gains_ms != 0 && now - _gains_ms < 1000) {
        return;
    }
    _gains_ms = now;
    _bus.set_run_gains(_deploy_cfg.servo_mask, _deploy_cfg.servo_kp, _deploy_cfg.servo_kd,
                       _deploy_cfg.n_servo_gains);
    AP_JointBus::Request req {};
    req.seq = _gains_seq++;
    req.op = uint8_t(AP_JointBus::Op::SERVO_SET_RUN_GAINS);
    _bus.request(req);
#endif
}

void AP_NNMixer::handle_service(const float *data)
{
    if (data == nullptr || !isfinite(data[0]) || !isfinite(data[1]) || !isfinite(data[2])) {
        return;
    }
    const uint16_t seq = uint16_t(constrain_float(data[0], 0.0f, 65535.0f));
    const uint8_t op = uint8_t(constrain_float(data[1], 0.0f, 255.0f));
    const uint8_t id = uint8_t(constrain_float(data[2], 0.0f, 255.0f));
    const bool armed = hal.util->get_soft_armed();
    if (op == 1 || op == 2) {
        // release torque / back to the disarmed pose
        if (!armed) {
            _torque_off = (op == 1);
        }
        send_service_reply(seq, op, !armed, id);
        return;
    }
#if AP_JOINTBUS_ENABLED
    if (_bus_ready && !armed) {
        AP_JointBus::Request req {};
        req.seq = seq;
        req.op = op;
        req.id = id;
        req.arg[0] = data[3];
        req.arg[1] = data[4];
        _bus.request(req);
        return;
    }
#endif
    send_service_reply(seq, op, false, id);
}

void AP_NNMixer::set_aux_cmd(float deg)
{
    const float lim = _deploy_cfg.valid ? MAX(_deploy_cfg.aux_max, 0.0f) : 0.0f;
    _aux_cmd = isfinite(deg) ? constrain_float(deg, 0.0f, lim) : 0.0f;
}

// setup replies and joint telemetry for the web pages
void AP_NNMixer::service_bus()
{
#if AP_JOINTBUS_ENABLED
    if (!_bus_ready) {
        return;
    }
    AP_JointBus::Reply r;
    while (_bus.pop_reply(r)) {
        if (r.op == uint8_t(AP_JointBus::Op::SERVO_SET_RUN_GAINS)) {
            _gains_done = r.ok != 0;
        }
        float d[11] {};
        d[0] = r.seq;
        d[1] = r.op;
        d[2] = r.ok;
        d[3] = r.id;
        d[4] = r.raw_pos;
        d[5] = r.center_pos;
        d[6] = r.kp;
        d[7] = r.kd;
        d[8] = r.protection;
        d[9] = r.current;
        d[10] = r.source;
        send_float_array("NNM_SRVR", r.seq, d, ARRAY_SIZE(d));
    }
    const uint32_t now = AP_HAL::millis();
    if (now - _last_jnt_ms < 100) {
        return;
    }
    _last_jnt_ms = now;
    AP_JointBus::Telemetry t;
    _bus.get_telemetry(t);
    if (!(t.status & AP_JointBus::STATUS_LINK)) {
        // the MCU may have restarted: the table goes out again when the link is back
        _gains_done = false;
    }
    // NNM_JNT: [0] status bits (AP_JointBus::Status, 64 = torque released on request),
    // [1] present mask, [2] held mask, [3..17] angle deg, [18..32] target deg, [33..47] raw encoder,
    // [48] slots, [49] state rate Hz, [50] commands refused by the MCU, [51] CRC errors,
    // [52] policy in use, [53] running skill id, [54] get-up state, [55] deployment chain,
    // [56] auxiliary actuator command deg, [57] 1 while armed
    float d[58] {};
    d[0] = float(t.status | (_torque_off ? 64U : 0U));
    d[1] = t.present_mask;
    d[2] = t.held_mask;
    const uint8_t ns = MIN(t.n_slots, uint8_t(15));
    for (uint8_t i = 0; i < ns; i++) {
        d[3 + i] = t.angle_deg[i];
        d[18 + i] = t.target_deg[i];
        d[33 + i] = t.raw_pos[i];
    }
    d[48] = ns;
    d[49] = t.rx_hz;
    d[50] = float(t.rejected_commands);
    d[51] = float(t.crc_errors);
    d[52] = float(_loaded_policy_idx);
    d[53] = _skill_state != SkillState::IDLE ? float(_deploy_cfg.skill[_skill_idx].id) : 0.0f;
    d[54] = float(uint8_t(_getup_state));
    d[55] = float(_chain);
    d[56] = _aux_cmd;
    d[57] = hal.util->get_soft_armed() ? 1.0f : 0.0f;
    send_float_array("NNM_JNT", uint16_t(_tick), d, ARRAY_SIZE(d));

    if (t.volt_ms != 0 && now - _last_vlt_ms >= 1000) {
        // NNM_VLT: [0] mask, [1..15] volts, [16] age of the reading in seconds
        _last_vlt_ms = now;
        float v[17] {};
        v[0] = t.volt_mask;
        for (uint8_t i = 0; i < ns; i++) {
            v[1 + i] = t.deci_volt[i] * 0.1f;
        }
        v[16] = float(now - t.volt_ms) * 0.001f;
        send_float_array("NNM_VLT", uint16_t(_tick), v, ARRAY_SIZE(v));
    }
#endif
}

void AP_NNMixer::update()
{
    if (!enabled()) {
        return;
    }
    if (!_initialised) {
        init();
    }
    _tick++;
    {
        // measured rate of this task and its longest interval, for the telemetry
        const uint32_t now_us = AP_HAL::micros();
        if (_rate_last_us != 0) {
            _rate_gap_us = MAX(_rate_gap_us, now_us - _rate_last_us);
        }
        _rate_last_us = now_us;
        _rate_count++;
    }

    const bool armed = hal.util->get_soft_armed();
    if (!armed) {
        // the deployment chain changes only at rest
        _chain = uint8_t(constrain_int16(_deploy.get(), 0, 1));
    }

    service_getup();
    service_skill();
    service_policy_switch();
    service_bus();
    publish_run_gains();

    if (!_topo.valid) {
        _fail_reason = 5;
        return;
    }
    // robots trained below 50 Hz tick every rate_div task runs; servo outputs hold in between
    const uint8_t rate_div = uint8_t(50 / _topo.rate_hz);
    if (rate_div > 1 && (_tick % rate_div) != 0) {
        return;
    }

    const bool xd = (_chain == 1);
    float aux = _aux_cmd;
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        // XDuck chain: joint velocity smoothed as in the original receive filter
        _bus.set_velocity_alpha(xd ? 0.4f : 0.0f);
    }
#endif

    if (!armed) {
        reset_history();
        _fail_reason = 1;
        _skill_state = SkillState::IDLE;
        _hh_walking = false;
        _xd_accx_cmd = 0.0f;
        bool setup = false;
#if AP_JOINTBUS_ENABLED
        if (_bus_ready) {
            _bus.set_aux_deg(aux);
            setup = _bus.exclusive();
        }
#endif
        if (_dis_pose == 1 && !_torque_off && !setup) {
            // stand in the default pose of the selected policy
            drive_outputs(default_pose(), false);
        } else {
            outputs_off();
        }
        send_telemetry();
        return;
    }
    _torque_off = false;
#if AP_JOINTBUS_ENABLED
    if (_bus_ready && _bus.exclusive()) {
        _bus.end_exclusive();
    }
#endif

    const uint32_t now_ms = AP_HAL::millis();
    const NNM_PolicyCfg *pc = policy_cfg(_loaded_policy_idx);
    const bool skill_on = _skill_state == SkillState::RUNNING || _skill_state == SkillState::ENDING;
    const NNM_Skill *skill = skill_on ? &_deploy_cfg.skill[_skill_idx] : nullptr;
    const bool recovering = _getup_state != GetupState::IDLE;

    float twist[3];
    read_twist(twist);
    if (pc != nullptr && is_positive(pc->wz_max)) {
        twist[2] = constrain_float(twist[2], -pc->wz_max, pc->wz_max);
    }
    bool zero_pose = pc != nullptr && (pc->flags & NNM_POLICY_ZERO_POSE) != 0;
    if (xd) {
        // trim and heading hold only while plainly walking, on the pilot's own twist
        const bool walking = !recovering && _skill_state == SkillState::IDLE;
        float trim = 0.0f;
        if (walking && !(pc != nullptr && (pc->flags & NNM_POLICY_NO_ACCX))) {
            if (is_positive(twist[0])) {
                trim = _xd_accx;
            } else if (is_negative(twist[0]) || !is_zero(twist[2])) {
                trim = -_xd_accx;
            }
        }
        _xd_accx_cmd = trim;
        _hh_walking = walking;
        twist[2] = heading_hold(walking, twist, (pc != nullptr && is_positive(pc->heading_kp)) ? pc->heading_kp : 1.5f);
        if (recovering) {
            twist[0] = twist[1] = twist[2] = 0.0f;
            zero_pose = true;
        }
    }
    if (skill != nullptr) {
        // the skill owns the command channels
        float phase = 0.0f;
        if (is_positive(skill->duration_s)) {
            phase = constrain_float(float(now_ms - _skill_start_ms) * 0.001f / skill->duration_s, 0.0f, 1.0f);
        }
        if (skill->flags & NNM_SKILL_CIRCLE) {
            twist[0] = cosf(M_2PI * phase);
            twist[1] = sinf(M_2PI * phase);
            twist[2] = 0.0f;
        } else {
            twist[0] = skill->cmd[0];
            twist[1] = skill->cmd[1];
            twist[2] = skill->cmd[2];
        }
        if (skill->flags & NNM_SKILL_ZERO_POSE) {
            zero_pose = true;
        }
        if (skill->flags & NNM_SKILL_AUX) {
            aux = (phase < skill->aux_until) ? skill->aux_deg : 0.0f;
        }
    }

    const bool joints_ok = read_joint_feedback();
    const bool hil_state_ok = _hil_state_ms != 0 &&
                              AP_HAL::millis() - _hil_state_ms <= uint32_t(MAX(_wd_ms.get(), 1));
    const AP_InertialSensor &ins = AP::ins();
    const Vector3f gyro = ins.get_gyro();
    const uint16_t nj = _topo.n_joints;
    const uint16_t od = _topo.obs_dim;

    float *o = _obs;
    memset(o, 0, sizeof(_obs));
    const bool use_hil_att = hil_state_ok && _hil_att != 1;
    if (use_hil_att) {
        WITH_SEMAPHORE(_joint_sem);
        memcpy(&o[0], _hil_gyro_flu, sizeof(_hil_gyro_flu));
        memcpy(&o[3], _hil_gravity_flu, sizeof(_hil_gravity_flu));
    } else if (xd) {
        o[0] = _xd_gyro.x;  o[1] = _xd_gyro.y;  o[2] = _xd_gyro.z;
        gravity_body_flu(&o[3]);
    } else {
        o[0] = gyro.x;  o[1] = -gyro.y;  o[2] = -gyro.z;
        gravity_body_flu(&o[3]);
    }
    if (use_hil_att && _hil_att == 2) {
        float board_g[3];
        gravity_body_flu(board_g);
        o[0] += gyro.x;  o[1] += -gyro.y;  o[2] += -gyro.z;
        o[3] += board_g[0];
        o[4] += board_g[1];
        o[5] += board_g[2] + 1.0f;
        const float n = sqrtf(o[3] * o[3] + o[4] * o[4] + o[5] * o[5]);
        if (n > 1e-6f) {
            o[3] /= n;  o[4] /= n;  o[5] /= n;
        }
    }
    const float *q0 = default_pose();
    {
        WITH_SEMAPHORE(_joint_sem);
        for (uint16_t i = 0; i < nj; i++) {
            o[6 + i] = _joint_pos[i] - q0[i];
            o[6 + nj + i] = _joint_vel[i];
        }
    }
    for (uint16_t i = 0; i < nj; i++) {
        o[6 + 2 * nj + i] = _last_action[i];
    }
    const uint16_t twist_off = 6 + 3 * nj;
    if (twist_off + 3 <= od) {
        o[twist_off] = twist[0];
        o[twist_off + 1] = twist[1];
        o[twist_off + 2] = twist[2];
    }
    // gesture / gait clock: sin/cos of a phase advancing at NNM_CLOCK_HZ, one step per policy tick.
    // With CLOCK_AUTO the clock stops (channels zero) while the twist is zero: stand still.
    // Get-up oneshot: θ = π · min(t/NNM_GETUP_S, 1) → ends at (sin, cos) = (0, −1), matching training.
    const bool commanded = fabsf(twist[0]) + fabsf(twist[1]) + fabsf(twist[2]) > 1.0e-3f;
    if (twist_off + 5 <= od && _clock_oneshot) {
        const float dt = float(rate_div) / 50.0f;
        const float T = MAX(_getup_s.get(), 0.5f);
        _clock_phase = MIN(_clock_phase + (float)M_PI * dt / T, (float)M_PI);
        o[twist_off + 3] = sinf(_clock_phase);
        o[twist_off + 4] = cosf(_clock_phase);
    } else if (twist_off + 5 <= od && is_positive(_clock_hz) && (_clock_auto == 0 || commanded)) {
        _clock_phase = wrap_2PI(_clock_phase + M_2PI * _clock_hz * float(rate_div) / 50.0f);
        o[twist_off + 3] = sinf(_clock_phase);
        o[twist_off + 4] = cosf(_clock_phase);
    } else {
        _clock_phase = 0.0f;
        if (twist_off + 5 <= od) {
            o[twist_off + 3] = 0.0f;
            o[twist_off + 4] = 0.0f;
        }
    }
    // pose teleop: channels after the clock (twist_off+5 ..). Fresh NNM_POSE is filtered in;
    // when the watchdog expires the same filter eases to zero (rest), never a hard jump.
    // The XDuck chain feeds the command as received; a policy or a skill can force it to zero.
    if (twist_off + 6 <= od) {
        const uint16_t n_pose = MIN((uint16_t)(od - (twist_off + 5)), (uint16_t)8);
        const float dt = float(rate_div) / 50.0f;
        const float tau = MAX(_pose_tau, 0.02f);
        const float alpha = xd ? 1.0f : MIN(1.0f, dt / tau);
        float target[8] {};
        {
            WITH_SEMAPHORE(_pose_sem);
            const bool fresh = (_pose_ms != 0) &&
                               (AP_HAL::millis() - _pose_ms <= (uint32_t)MAX(int16_t(_pose_wd_ms), 50));
            if (fresh && !zero_pose) {
                for (uint16_t i = 0; i < n_pose; i++) {
                    target[i] = _pose_cmd[i];
                }
            }
        }
        for (uint16_t i = 0; i < n_pose; i++) {
            _pose_filt[i] = zero_pose ? 0.0f : _pose_filt[i] + alpha * (target[i] - _pose_filt[i]);
            o[twist_off + 5 + i] = _pose_filt[i];
        }
    }

    const bool att_ok = use_hil_att || (xd ? _xd_valid : _down_valid);
    bool ok = joints_ok && att_ok && (_use_sd_policy || _use_baked_mlp);
    if (ok) {
        const uint32_t t0 = wall_micros();
        if (_use_sd_policy && _slot[_active_slot].ready) {
            ok = nnmixer_forward_int8(&_slot[_active_slot].view, _obs, _act) == 0;
#if AP_NNMIXER_BAKED_MLP_ENABLED
        } else if (_use_baked_mlp) {
            static const nnmixer_policy_t mlp_policy = {
                MLP_OBS_DIM, MLP_ACT_DIM, MLP_N_LAYERS,
                mlp_dims, mlp_act, mlp_obs_mean, mlp_obs_std, mlp_W, mlp_b,
            };
            ok = nnmixer_forward(&mlp_policy, _obs, _act) == 0;
#endif
        } else {
            ok = false;
        }
        _forward_us = wall_micros() - t0;
        if (!ok) {
            _fail_reason = 4;
        }
    }
    if (ok) {
        _fail_reason = 0;
        for (uint16_t i = 0; i < nj; i++) {
            _act[i] = constrain_float(_act[i], -_act_max, _act_max);
        }
        if (_blending) {
            const uint32_t elapsed = AP_HAL::millis() - _blend_start_ms;
            const uint32_t dur = uint32_t(MAX(_blend_ms.get(), 1));
            float a = constrain_float(float(elapsed) / float(dur), 0.0f, 1.0f);
            for (uint16_t i = 0; i < nj; i++) {
                _act[i] = _act_blend_from[i] * (1.0f - a) + _act[i] * a;
            }
            if (a >= 1.0f) {
                _blending = false;
            }
        }
    } else {
        memset(_act, 0, sizeof(_act));
    }
    if (xd && _getup_state == GetupState::FALLING) {
        // after a fall: default pose, action and history at zero, until the get-up starts
        memset(_act, 0, sizeof(_act));
        reset_history();
    }

    float q_target[NNM_MAX_JOINTS];
    if (xd) {
        // the history takes the action as computed; the joints take a low-passed copy, with the
        // head joints on their own constant
        float keep_leg = _deploy_cfg.valid ? _deploy_cfg.alpha_leg : 0.35f;
        float keep_head = _deploy_cfg.valid ? _deploy_cfg.alpha_head : 0.45f;
        if (pc != nullptr) {
            keep_leg = pc->alpha_leg;
            keep_head = pc->alpha_head;
        }
        const uint16_t head0 = _deploy_cfg.head_first;
        const uint16_t head1 = uint16_t(head0 + _deploy_cfg.head_count);
        for (uint16_t i = 0; i < nj; i++) {
            const float keep = constrain_float((i >= head0 && i < head1) ? keep_head : keep_leg, 0.0f, 0.99f);
            _last_action[i] = _act[i];
            _act_filt[i] = keep * _act_filt[i] + (1.0f - keep) * _act[i];
            q_target[i] = q0[i] + _act_filt[i];
        }
    } else {
        for (uint16_t i = 0; i < nj; i++) {
            q_target[i] = q0[i] + _act[i];
            _last_action[i] = _act[i];
        }
    }
#if AP_JOINTBUS_ENABLED
    if (_bus_ready) {
        _bus.set_aux_deg(aux);
    }
#endif
    drive_outputs(q_target, true);

    if (hil_state_ok) {
        static float hil_action[58] {};
        static char hil_name[10] = "NNM_ACT";
        memcpy(hil_action, q_target, nj * sizeof(float));
        mavlink_msg_debug_float_array_send(MAVLINK_COMM_0, AP_HAL::micros64(),
                                           hil_name, uint16_t(_tick), hil_action);
    }

    log_tick(twist);
    send_telemetry();
}

void AP_NNMixer::log_tick(const float twist[3])
{
#if HAL_LOGGING_ENABLED
    if (_log == 0) {
        return;
    }
    const uint64_t now = AP_HAL::micros64();
    const float *o = _obs;
    AP::logger().WriteStreaming("NNM", "TimeUS,Tick,Fail,FwdUS,GX,GY,GZ,PGX,PGY,PGZ,VX,VY,WZ",
                                "QIBIfffffffff",
                                now, _tick, _fail_reason, _forward_us,
                                o[0], o[1], o[2], o[3], o[4], o[5], twist[0], twist[1], twist[2]);
    if (_topo.n_joints >= 14) {
        AP::logger().WriteStreaming("NNMQ", "TimeUS,Q0,Q1,Q2,Q3,Q4,Q5,Q6,Q7,Q8,Q9,Q10,Q11,Q12,Q13",
                                    "Qffffffffffffff",
                                    now, o[6], o[7], o[8], o[9], o[10], o[11], o[12], o[13], o[14], o[15], o[16], o[17], o[18], o[19]);
        AP::logger().WriteStreaming("NNMV", "TimeUS,V0,V1,V2,V3,V4,V5,V6,V7,V8,V9,V10,V11,V12,V13",
                                    "Qffffffffffffff",
                                    now, o[20], o[21], o[22], o[23], o[24], o[25], o[26], o[27], o[28], o[29], o[30], o[31], o[32], o[33]);
        AP::logger().WriteStreaming("NNMA", "TimeUS,A0,A1,A2,A3,A4,A5,A6,A7,A8,A9,A10,A11,A12,A13",
                                    "Qffffffffffffff",
                                    now, _act[0], _act[1], _act[2], _act[3], _act[4], _act[5], _act[6], _act[7], _act[8], _act[9], _act[10], _act[11], _act[12], _act[13]);
    }
#endif
}

void AP_NNMixer::send_telemetry()
{
    const uint32_t now = AP_HAL::millis();
    if (now - _last_telem_ms < 500) {
        return;
    }
    const float window_s = (_last_telem_ms != 0) ? float(now - _last_telem_ms) * 1.0e-3f : 0.0f;
    _last_telem_ms = now;
    // policy task: measured rate and longest interval since the last report; attitude task the same
    _rate_hz = window_s > 0.0f ? float(_rate_count) / window_s : 0.0f;
    gcs().send_named_float("PPO_HZ", _rate_hz);
    gcs().send_named_float("PPO_GAP", _rate_gap_us * 1.0e-3f);
    gcs().send_named_float("PPO_LHZ", window_s > 0.0f ? float(_loop_count) / window_s : 0.0f);
    gcs().send_named_float("PPO_LGAP", _loop_gap_us * 1.0e-3f);
    _rate_count = 0;
    _rate_gap_us = 0;
    _loop_count = 0;
    _loop_gap_us = 0;
    gcs().send_named_float("PPO_MS", _forward_us * 1.0e-3f);
    gcs().send_named_float("PPO_PGZ", _obs[5]);
    const uint16_t twist_off = _topo.valid ? uint16_t(6 + 3 * _topo.n_joints) : 48;
    gcs().send_named_float("PPO_VX", (twist_off < NNM_MAX_OBS) ? _obs[twist_off] : 0.0f);
    gcs().send_named_float("PPO_FAIL", float(_fail_reason));
    gcs().send_named_float("PPO_SLOT", float(_active_slot));
    // first filtered pose-command channel (right shoulder pitch): shows the NNM_POSE stream and
    // the watchdog decay from the GCS
    gcs().send_named_float("PPO_POSE0", _pose_filt[0]);
    // policy in use, running skill (0 = none), get-up state, deployment chain
    gcs().send_named_float("PPO_POL", float(_loaded_policy_idx));
    gcs().send_named_float("PPO_SKILL", _skill_state != SkillState::IDLE ? float(_deploy_cfg.skill[_skill_idx].id) : 0.0f);
    gcs().send_named_float("PPO_GETUP", float(uint8_t(_getup_state)));
    gcs().send_named_float("PPO_CHAIN", float(_chain));
}

namespace AP {
    AP_NNMixer *nnmixer()
    {
        return AP_NNMixer::get_singleton();
    }
};

#endif // AP_NNMIXER_ENABLED

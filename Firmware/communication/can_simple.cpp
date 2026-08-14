
#include "can_simple.hpp"
#include <odrive_main.h>

#include <cmath>
#include <cstring>

static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

void CANSimple::handle_can_message(can_Message_t& msg) {
    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking to fix the syntax.
    //
    // auto func = callback_map.find(msg.id);
    // if(func != callback_map.end()){
    //     func->second(msg);
    // }

    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);
    uint32_t cmd = get_cmd_id(msg.id);

    Axis* axis = nullptr;

    bool validAxis = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if ((axes[i]->config_.can_node_id == nodeID) && (axes[i]->config_.can_node_id_extended == msg.isExt)) {
            axis = axes[i];
            if (!validAxis) {
                validAxis = true;
            } else {
                // Duplicate can IDs, don't assign to any axis
                odCAN->set_error(ODriveCAN::ERROR_DUPLICATE_CAN_IDS);
                validAxis = false;
                break;
            }
        }
    }

    if (validAxis) {
        axis->watchdog_feed();
        switch (cmd) {
            case MSG_CO_NMT_CTRL:
                break;
            case MSG_CO_HEARTBEAT_CMD:
                break;
            case MSG_ODRIVE_HEARTBEAT:
                // We don't currently do anything to respond to ODrive heartbeat messages
                break;
            case MSG_ODRIVE_ESTOP:
                estop_callback(axis, msg);
                break;
            case MSG_GET_MOTOR_ERROR:
                get_motor_error_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ERROR:
                get_encoder_error_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ERROR:
                get_sensorless_error_callback(axis, msg);
                break;
            case MSG_SET_AXIS_NODE_ID:
                set_axis_nodeid_callback(axis, msg);
                break;
            case MSG_SET_AXIS_REQUESTED_STATE:
                set_axis_requested_state_callback(axis, msg);
                break;
            case MSG_SET_AXIS_STARTUP_CONFIG:
                set_axis_startup_config_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_ESTIMATES:
                get_encoder_estimates_callback(axis, msg);
                break;
            case MSG_GET_ENCODER_COUNT:
                get_encoder_count_callback(axis, msg);
                break;
            case MSG_SET_INPUT_POS:
                set_input_pos_callback(axis, msg);
                break;
            case MSG_SET_INPUT_VEL:
                set_input_vel_callback(axis, msg);
                break;
            case MSG_SET_INPUT_TORQUE:
                set_input_torque_callback(axis, msg);
                break;
            case MSG_SET_CONTROLLER_MODES:
                set_controller_modes_callback(axis, msg);
                break;
            case MSG_SET_LIMITS:
                set_limits_callback(axis, msg);
                break;
            case MSG_START_ANTICOGGING:
                start_anticogging_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_INERTIA:
                set_traj_inertia_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_ACCEL_LIMITS:
                set_traj_accel_limits_callback(axis, msg);
                break;
            case MSG_SET_TRAJ_VEL_LIMIT:
                set_traj_vel_limit_callback(axis, msg);
                break;
            case MSG_GET_IQ:
                get_iq_callback(axis, msg);
                break;
            case MSG_GET_SENSORLESS_ESTIMATES:
                get_sensorless_estimates_callback(axis, msg);
                break;
            case MSG_RESET_ODRIVE:
                // LOCAL CHANGE: ignored on purpose -- this used to be a bare
                // NVIC_SystemReset(). One data OR RTR frame, no magic key, no
                // IDLE check, and the board reboots. That is unsafe here for
                // two reasons:
                //   * every board's unused axis1 still answers to node id 1
                //     (its heartbeat is muted, but the RX filter is not), so a
                //     single frame to node 1 resets the WHOLE fleet at once;
                //   * a reset that lands inside save_configuration() leaves the
                //     NVM mid-transaction, and load_configuration() then falls
                //     back to full defaults -- which sets can_node_id back to
                //     the axis index (0 and 1) and drops motor/encoder
                //     calibration, load_encoder_axis and the endstops with it.
                // The sanctioned reboot is MSG_CONFIG_COMMIT with
                // CONFIG_ACTION_REBOOT: magic-key gated, and it cannot race the
                // deferred save because both are serialised through this thread.
                break;
            case MSG_GET_VBUS_VOLTAGE:
                get_vbus_voltage_callback(axis, msg);
                break;
            case MSG_CLEAR_ERRORS:
                clear_errors_callback(axis, msg);
                break;
            case MSG_CONFIG_ACCESS:
                config_access_callback(axis, msg);
                break;
            case MSG_CONFIG_COMMIT:
                config_commit_callback(axis, msg);
                break;
            default:
                break;
        }
    }
}

void CANSimple::nmt_callback(Axis* axis, can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis* axis, can_Message_t& msg) {
    axis->error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

void CANSimple::get_motor_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->motor_.error_;
        txmsg.buf[1] = axis->motor_.error_ >> 8;
        txmsg.buf[2] = axis->motor_.error_ >> 16;
        txmsg.buf[3] = axis->motor_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.error_;
        txmsg.buf[1] = axis->encoder_.error_ >> 8;
        txmsg.buf[2] = axis->encoder_.error_ >> 16;
        txmsg.buf[3] = axis->encoder_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->sensorless_estimator_.error_;
        txmsg.buf[1] = axis->sensorless_estimator_.error_ >> 8;
        txmsg.buf[2] = axis->sensorless_estimator_.error_ >> 16;
        txmsg.buf[3] = axis->sensorless_estimator_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_axis_nodeid_callback(Axis* axis, can_Message_t& msg) {
    axis->config_.can_node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis* axis, can_Message_t& msg) {
    axis->requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 16, true));
}
void CANSimple::set_axis_startup_config_callback(Axis* axis, can_Message_t& msg) {
    // Not Implemented
}

void CANSimple::get_encoder_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Local patch: report the SAME estimate sources the controller closes
        // the loop on, not blindly this axis's own encoder:
        //   position <- load encoder (controller.config.load_encoder_axis), so
        //     geared joints publish true OUTPUT-shaft angle over CAN (the MT6701
        //     mounted after the gearbox on the other axis) instead of the
        //     motor-shaft AS5047P.
        //   velocity <- vel encoder (controller.config.vel_encoder_axis), the
        //     motor-shaft encoder used for commutation/velocity.
        // This mirrors the axis-selection + fallback in
        // Axis::run_closed_loop_control_loop() (vel falls back to load; load
        // falls back to this axis) so it stays correct even while the axis is
        // IDLE -- the controller's cached *_src_ pointers are only bound on
        // closed-loop entry, but the host must read joint angle any time.
        // Non-split joints keep load_encoder_axis == their own axis, so they
        // report their own encoder exactly as before (backward compatible).
        Encoder& pos_enc = (axis->controller_.config_.load_encoder_axis < AXIS_COUNT)
                ? axes[axis->controller_.config_.load_encoder_axis]->encoder_
                : axis->encoder_;
        Encoder& vel_enc = (axis->controller_.config_.vel_encoder_axis < AXIS_COUNT)
                ? axes[axis->controller_.config_.vel_encoder_axis]->encoder_
                : pos_enc;

        uint32_t floatBytes;
        static_assert(sizeof pos_enc.pos_estimate_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &pos_enc.pos_estimate_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof vel_enc.vel_estimate_);
        std::memcpy(&floatBytes, &vel_enc.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->sensorless_estimator_.pll_pos_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.pll_pos_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->sensorless_estimator_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_count_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_COUNT;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.shadow_count_;
        txmsg.buf[1] = axis->encoder_.shadow_count_ >> 8;
        txmsg.buf[2] = axis->encoder_.shadow_count_ >> 16;
        txmsg.buf[3] = axis->encoder_.shadow_count_ >> 24;

        txmsg.buf[4] = axis->encoder_.count_in_cpr_;
        txmsg.buf[5] = axis->encoder_.count_in_cpr_ >> 8;
        txmsg.buf[6] = axis->encoder_.count_in_cpr_ >> 16;
        txmsg.buf[7] = axis->encoder_.count_in_cpr_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_input_pos_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_pos_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis->controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis->controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.control_mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis->controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
}

// 0x00F Set_Limits: vel_limit in bytes 0-3, motor current_lim in bytes 4-7.
// Stock v0.5.1 only carried the velocity limit, so the current limit is applied
// ONLY for a full 8-byte frame -- a legacy 4-byte Set_Vel_Limit must not be read
// as "set current_lim = 0", which would silently disarm the motor.
// The value is RAM-only (no NVM write) and is still clamped downstream by
// Motor::effective_current_lim() against the hardware max and the thermistor
// limiters, so a bad host value cannot exceed what the board can survive.
void CANSimple::set_limits_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);

    if (msg.len >= 8) {
        float current_lim = can_getSignal<float>(msg, 32, 32, true);
        if (std::isfinite(current_lim) && current_lim > 0.0f) {
            axis->motor_.config_.current_lim = current_lim;
        }
    }
}

void CANSimple::start_anticogging_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis->trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::get_iq_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_IQ;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof axis->motor_.current_control_.Iq_setpoint == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_setpoint, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->motor_.current_control_.Iq_measured);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_measured, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_vbus_voltage_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;

        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_VBUS_VOLTAGE;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof vbus_voltage == sizeof floatBytes);
        std::memcpy(&floatBytes, &vbus_voltage, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
    }
}

void CANSimple::clear_errors_callback(Axis* axis, can_Message_t& msg) {
    axis->clear_errors();
}

// ---------------------------------------------------------------------------
// LOCAL ADDITION: live configuration over CAN (MSG_CONFIG_ACCESS / _COMMIT)
//
// Motivation: everything needed to bring a joint up -- endstops, joint zero and
// direction, position gains, current limit -- lived only behind USB, so tuning
// a leg on the robot meant physically reaching the board and replugging. This
// exposes exactly those fields, plus the temperatures CAN Simple otherwise
// cannot reach, as a small typed parameter table.
//
// No config STRUCT changes here, so `config_version` in nvm_config.hpp is NOT
// bumped and an existing saved configuration survives this firmware update.
//
// Wire format, MSG_CONFIG_ACCESS (0x01C). Always a DATA frame, never RTR: an
// RTR frame carries no payload and so could not name a parameter.
//   request : [0] op (0=read, 1=write)   [1] param id   [2..3] reserved (0)
//             [4..7] value, little-endian, type per the table
//   reply   : [0] op, | 0x80 if it failed [1] param id   [2] status
//             [3] type                    [4..7] value AFTER the operation
// The reply always carries the resulting value, so a write is self-verifying:
// a rejected or clamped write shows up immediately without a second read.
// ---------------------------------------------------------------------------

namespace {

struct ConfigValue {
    uint8_t type = CANSimple::CONFIG_TYPE_FLOAT;
    union {
        float f;
        int32_t i;
        uint32_t u;
    };
    ConfigValue() : u(0) {}
};

// The load/joint encoder for this axis: the axis pointed at by
// controller.config.load_encoder_axis, mirroring the fallback in
// Axis::run_closed_loop_control_loop() and get_encoder_estimates_callback().
// Returns nullptr only if the configured axis index is out of range.
Encoder* joint_encoder(Axis* axis) {
    uint8_t idx = axis->controller_.config_.load_encoder_axis;
    if (idx < AXIS_COUNT) {
        return &axes[idx]->encoder_;
    }
    return &axis->encoder_;
}

bool all_axes_idle() {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (axes[i]->current_state_ != Axis::AXIS_STATE_IDLE &&
            axes[i]->current_state_ != Axis::AXIS_STATE_UNDEFINED) {
            return false;
        }
    }
    return true;
}

bool finite_positive(float v) { return std::isfinite(v) && v > 0.0f; }

}  // namespace

void CANSimple::config_access_callback(Axis* axis, can_Message_t& msg) {
    // The node id is taken from the RECEIVED frame, not from the axis config:
    // PARAM_CAN_NODE_ID may change it, and the reply has to come back on the
    // address the host is listening on.
    const uint32_t reply_node = get_node_id(msg.id);
    const bool reply_ext = msg.isExt;

    if (msg.rtr || msg.len < 2) {
        return;  // malformed; stay silent rather than answer nonsense
    }

    const uint8_t op = msg.buf[0];
    const uint8_t param = msg.buf[1];
    const bool write = (op == CONFIG_OP_WRITE);

    ConfigValue in;
    if (msg.len >= 8) {
        in.u = static_cast<uint32_t>(msg.buf[4]) |
               (static_cast<uint32_t>(msg.buf[5]) << 8) |
               (static_cast<uint32_t>(msg.buf[6]) << 16) |
               (static_cast<uint32_t>(msg.buf[7]) << 24);
    }
    std::memcpy(&in.f, &in.u, sizeof in.f);

    uint8_t status = CONFIG_OK;
    ConfigValue out;
    Encoder* enc = joint_encoder(axis);
    Encoder::Config_t& ec = enc->config_;
    Controller::Config_t& cc = axis->controller_.config_;
    Motor::Config_t& mc = axis->motor_.config_;

    switch (param) {
        // ------------------------------------------------ joint/load encoder
        // Both endpoints refuse a write that would INVERT an already-enabled
        // range. Controller::update() only clamps while max >= min, so an
        // inverted range silently stops enforcing the endstops while
        // enable_position_limit still reads back as 1 -- the joint would look
        // protected and not be. Widening a range is always allowed; to reorder
        // one, disable the limits first (that is the order the jog pendant and
        // can_config use).
        case PARAM_JOINT_MIN_POSITION:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!std::isfinite(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else if (ec.enable_position_limit && in.f > ec.max_position) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    ec.min_position = in.f;
                }
            }
            out.f = ec.min_position;
            break;
        case PARAM_JOINT_MAX_POSITION:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!std::isfinite(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else if (ec.enable_position_limit && in.f < ec.min_position) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    ec.max_position = in.f;
                }
            }
            out.f = ec.max_position;
            break;
        case PARAM_JOINT_LIMIT_ENABLE:
            out.type = CONFIG_TYPE_BOOL;
            if (write) {
                // Refuse to arm the endstops on a range that would clamp the
                // setpoint to nonsense. An inverted or non-finite range would
                // otherwise pin the joint at a limit it can never satisfy.
                if (in.u && !(std::isfinite(ec.min_position) &&
                              std::isfinite(ec.max_position) &&
                              ec.max_position >= ec.min_position)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    ec.enable_position_limit = (in.u != 0);
                }
            }
            out.u = ec.enable_position_limit ? 1u : 0u;
            break;
        case PARAM_JOINT_DIRECTION:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                ec.set_direction(in.i);  // also re-seeds the published position
            }
            out.i = ec.direction;
            break;
        case PARAM_JOINT_ZERO_OFFSET:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                ec.set_zero_offset(in.i);
            }
            out.i = ec.zero_offset;
            break;
        case PARAM_JOINT_SET_ZERO:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                enc->set_zero();  // current pose becomes published zero
            }
            out.i = ec.zero_offset;
            break;
        case PARAM_JOINT_RESEED:
            // The linear pos_estimate is an accumulator seeded once at boot; a
            // burst of stick-slip plus dropped SPI samples can cost it a whole
            // turn. Writing zero_offset onto itself calls reset_user_position()
            // and re-seeds it from the (always-correct) circular count.
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                ec.set_zero_offset(ec.zero_offset);
            }
            out.f = enc->pos_estimate_;
            break;
        case PARAM_JOINT_POS_ESTIMATE:
            out.type = CONFIG_TYPE_FLOAT;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.f = enc->pos_estimate_;
            break;
        case PARAM_JOINT_COUNT_IN_CPR:
            out.type = CONFIG_TYPE_INT32;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.i = enc->count_in_cpr_;
            break;
        case PARAM_JOINT_CPR:
            out.type = CONFIG_TYPE_INT32;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.i = ec.cpr;
            break;
        case PARAM_JOINT_ENCODER_ERROR:
            out.type = CONFIG_TYPE_UINT32;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.u = enc->error_;
            break;
        case PARAM_JOINT_TURN_SNAPS:
            // How many times the automatic turn-snap has had to pull the linear
            // pos_estimate back onto [min_position, max_position]. Non-zero
            // means this joint IS losing turns -- the snap papers over it, the
            // cause is dropped SPI samples (EMI) or stick-slip. Watch the rate.
            out.type = CONFIG_TYPE_UINT32;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.u = enc->turn_snap_count_;
            break;

        // ------------------------------------------------------- controller
        case PARAM_POS_GAIN:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    cc.pos_gain = in.f;
                }
            }
            out.f = cc.pos_gain;
            break;
        case PARAM_VEL_GAIN:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    cc.vel_gain = in.f;
                }
            }
            out.f = cc.vel_gain;
            break;
        case PARAM_VEL_INTEGRATOR_GAIN:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!std::isfinite(in.f) || in.f < 0.0f) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    cc.vel_integrator_gain = in.f;
                }
            }
            out.f = cc.vel_integrator_gain;
            break;
        case PARAM_VEL_LIMIT:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    cc.vel_limit = in.f;
                }
            }
            out.f = cc.vel_limit;
            break;
        case PARAM_POSITION_DIRECTION:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                cc.set_position_direction(in.i);
            }
            out.i = cc.position_direction;
            break;
        case PARAM_LOAD_ENCODER_AXIS:
        case PARAM_VEL_ENCODER_AXIS:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                // The controller binds its encoder pointers on closed-loop
                // entry, so swapping the source under a running loop would
                // leave it reading one encoder and clamping to another's
                // limits. Idle only.
                if (axis->current_state_ != Axis::AXIS_STATE_IDLE &&
                    axis->current_state_ != Axis::AXIS_STATE_UNDEFINED) {
                    status = CONFIG_REQUIRES_IDLE;
                } else if (in.i < 0 || in.i >= static_cast<int32_t>(AXIS_COUNT)) {
                    status = CONFIG_VALUE_REJECTED;
                } else if (param == PARAM_LOAD_ENCODER_AXIS) {
                    cc.load_encoder_axis = static_cast<uint8_t>(in.i);
                } else {
                    cc.vel_encoder_axis = static_cast<uint8_t>(in.i);
                }
            }
            out.i = (param == PARAM_LOAD_ENCODER_AXIS) ? cc.load_encoder_axis
                                                       : cc.vel_encoder_axis;
            break;
        case PARAM_INPUT_FILTER_BANDWIDTH:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    cc.set_input_filter_bandwidth(in.f);
                }
            }
            out.f = cc.input_filter_bandwidth;
            break;

        // ------------------------------------------------------------ motor
        case PARAM_CURRENT_LIM:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    mc.current_lim = in.f;
                }
            }
            out.f = mc.current_lim;
            break;
        case PARAM_TORQUE_CONSTANT:
            out.type = CONFIG_TYPE_FLOAT;
            if (write) {
                if (!finite_positive(in.f)) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    mc.torque_constant = in.f;
                }
            }
            out.f = mc.torque_constant;
            break;

        // ------------------------------------------------------- telemetry
        case PARAM_FET_TEMPERATURE:
            out.type = CONFIG_TYPE_FLOAT;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.f = axis->fet_thermistor_.temperature_;
            break;
        case PARAM_MOTOR_TEMPERATURE:
            // NB: with no thermistor wired, the ADC floats and this reads a
            // plausible-looking value. Trust it only where the motor
            // thermistor is actually fitted and enabled.
            out.type = CONFIG_TYPE_FLOAT;
            status = write ? CONFIG_READ_ONLY : CONFIG_OK;
            out.f = axis->motor_thermistor_.temperature_;
            break;
        case PARAM_MOTOR_THERM_ENABLE:
            out.type = CONFIG_TYPE_BOOL;
            if (write) {
                axis->motor_thermistor_.config_.enabled = (in.u != 0);
            }
            out.u = axis->motor_thermistor_.config_.enabled ? 1u : 0u;
            break;

        // ------------------------------------------------------ node / bus
        case PARAM_CAN_NODE_ID:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                if (!all_axes_idle()) {
                    status = CONFIG_REQUIRES_IDLE;
                } else if (in.i < 0 || in.i > 0x3F) {
                    status = CONFIG_VALUE_REJECTED;  // 6-bit node field
                } else {
                    axis->config_.can_node_id = static_cast<uint32_t>(in.i);
                }
            }
            out.i = static_cast<int32_t>(axis->config_.can_node_id);
            break;
        case PARAM_CAN_HEARTBEAT_RATE:
            out.type = CONFIG_TYPE_INT32;
            if (write) {
                if (in.i < 0) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    axis->config_.can_heartbeat_rate_ms = static_cast<uint32_t>(in.i);
                }
            }
            out.i = static_cast<int32_t>(axis->config_.can_heartbeat_rate_ms);
            break;
        case PARAM_OTHER_AXIS_HEARTBEAT: {
            // Every axis defaults to can_node_id 1 with a 100 ms heartbeat and
            // the firmware transmits it whether or not the axis is used, so an
            // unmuted second axis makes every board on the bus claim node 1.
            // Reachable from the motor axis so it can be fixed without ever
            // addressing the offending axis.
            out.type = CONFIG_TYPE_INT32;
            Axis* other = nullptr;
            for (size_t i = 0; i < AXIS_COUNT; ++i) {
                if (axes[i] != axis) {
                    other = axes[i];
                    break;
                }
            }
            if (other == nullptr) {
                status = CONFIG_NO_TARGET;
                break;
            }
            if (write) {
                if (in.i < 0) {
                    status = CONFIG_VALUE_REJECTED;
                } else {
                    other->config_.can_heartbeat_rate_ms = static_cast<uint32_t>(in.i);
                }
            }
            out.i = static_cast<int32_t>(other->config_.can_heartbeat_rate_ms);
            break;
        }

        default:
            status = CONFIG_UNKNOWN_PARAM;
            break;
    }

    can_Message_t txmsg;
    txmsg.id = (reply_node << NUM_CMD_ID_BITS) + MSG_CONFIG_ACCESS;
    txmsg.isExt = reply_ext;
    txmsg.len = 8;
    txmsg.buf[0] = static_cast<uint8_t>(op | (status ? CONFIG_OP_ERROR_FLAG : 0));
    txmsg.buf[1] = param;
    txmsg.buf[2] = status;
    txmsg.buf[3] = out.type;
    if (out.type == CONFIG_TYPE_FLOAT) {
        std::memcpy(&out.u, &out.f, sizeof out.u);
    }
    txmsg.buf[4] = out.u;
    txmsg.buf[5] = out.u >> 8;
    txmsg.buf[6] = out.u >> 16;
    txmsg.buf[7] = out.u >> 24;
    odCAN->write(txmsg);
}

void CANSimple::config_commit_callback(Axis* axis, can_Message_t& msg) {
    const uint32_t reply_node = get_node_id(msg.id);

    if (msg.rtr || msg.len < 5) {
        return;
    }
    const uint32_t magic = static_cast<uint32_t>(msg.buf[0]) |
                           (static_cast<uint32_t>(msg.buf[1]) << 8) |
                           (static_cast<uint32_t>(msg.buf[2]) << 16) |
                           (static_cast<uint32_t>(msg.buf[3]) << 24);
    const uint8_t action = msg.buf[4];

    uint8_t status = CONFIG_OK;
    if (magic != CONFIG_COMMIT_MAGIC) {
        status = CONFIG_BAD_MAGIC;
    } else if (action == CONFIG_ACTION_SAVE) {
        // Writing NVM erases a flash sector, which stalls the core for far
        // longer than one 8 kHz control period -- it must never happen under a
        // spinning motor. It also needs more stack than this CAN thread has
        // (1 kB), so it is handed to the communication thread, which replies
        // with CONFIG_SAVE_DONE / CONFIG_SAVE_FAILED when it is finished.
        if (!all_axes_idle()) {
            status = CONFIG_REQUIRES_IDLE;
        } else if (odrv.config_save_request_) {
            status = CONFIG_BUSY;
        } else {
            odrv.config_save_reply_node_ = reply_node;
            odrv.config_save_reply_ext_ = msg.isExt;
            odrv.config_save_request_ = 1;
        }
    } else if (action == CONFIG_ACTION_REBOOT) {
        // Acknowledge before the reset, or the host never hears anything.
        send_config_commit_reply(reply_node, msg.isExt, action, CONFIG_OK);
        osDelay(50);
        NVIC_SystemReset();
        return;
    } else {
        status = CONFIG_VALUE_REJECTED;
    }

    send_config_commit_reply(reply_node, msg.isExt, action, status);
}

void CANSimple::send_config_commit_reply(uint32_t node_id, bool is_ext,
                                         uint8_t action, uint8_t status) {
    can_Message_t txmsg;
    txmsg.id = (node_id << NUM_CMD_ID_BITS) + MSG_CONFIG_COMMIT;
    txmsg.isExt = is_ext;
    txmsg.len = 8;
    txmsg.buf[0] = action;
    txmsg.buf[1] = status;
    for (size_t i = 2; i < 8; ++i) {
        txmsg.buf[i] = 0;
    }
    odCAN->write(txmsg);
}

void CANSimple::send_heartbeat(Axis* axis) {
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;

    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = axis->error_;
    txmsg.buf[1] = axis->error_ >> 8;
    txmsg.buf[2] = axis->error_ >> 16;
    txmsg.buf[3] = axis->error_ >> 24;

    // Current state of axis in 2nd 32-bit value
    txmsg.buf[4] = axis->current_state_;
    txmsg.buf[5] = axis->current_state_ >> 8;
    txmsg.buf[6] = axis->current_state_ >> 16;
    txmsg.buf[7] = axis->current_state_ >> 24;
    odCAN->write(txmsg);
}

uint32_t CANSimple::get_node_id(uint32_t msgID) {
    return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
}

uint8_t CANSimple::get_cmd_id(uint32_t msgID) {
    return (msgID & 0x01F);  // Bottom 5 bits
}
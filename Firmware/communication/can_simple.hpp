#ifndef __CAN_SIMPLE_HPP_
#define __CAN_SIMPLE_HPP_

#include "interface_can.hpp"

class CANSimple {
   public:
    enum {
        MSG_CO_NMT_CTRL = 0x000,       // CANOpen NMT Message REC
        MSG_ODRIVE_HEARTBEAT,
        MSG_ODRIVE_ESTOP,
        MSG_GET_MOTOR_ERROR,  // Errors
        MSG_GET_ENCODER_ERROR,
        MSG_GET_SENSORLESS_ERROR,
        MSG_SET_AXIS_NODE_ID,
        MSG_SET_AXIS_REQUESTED_STATE,
        MSG_SET_AXIS_STARTUP_CONFIG,
        MSG_GET_ENCODER_ESTIMATES,
        MSG_GET_ENCODER_COUNT,
        MSG_SET_CONTROLLER_MODES,
        MSG_SET_INPUT_POS,
        MSG_SET_INPUT_VEL,
        MSG_SET_INPUT_TORQUE,
        MSG_SET_LIMITS,  // 0x00F: vel_limit (bytes 0-3) + current_lim (bytes 4-7, optional)
        MSG_START_ANTICOGGING,
        MSG_SET_TRAJ_VEL_LIMIT,
        MSG_SET_TRAJ_ACCEL_LIMITS,
        MSG_SET_TRAJ_INERTIA,
        MSG_GET_IQ,
        MSG_GET_SENSORLESS_ESTIMATES,
        MSG_RESET_ODRIVE,  // 0x016: IGNORED here -- reboot only via MSG_CONFIG_COMMIT.
                           // The id stays in the enum so the ones after it keep
                           // their numbers; the handler is a no-op on purpose.
        MSG_GET_VBUS_VOLTAGE,
        MSG_CLEAR_ERRORS,  // 0x018

        // --- LOCAL ADDITION: live configuration over CAN --------------------
        // 0x019..0x01B are deliberately left unused: upstream ODrive 0.5.x
        // later assigned them to SET_LINEAR_COUNT / SET_POS_GAIN /
        // SET_VEL_GAINS, and we do not want to collide with a host library
        // that already knows those.
        MSG_CONFIG_ACCESS = 0x01C,  // read/write one parameter, see ConfigParam
        MSG_CONFIG_COMMIT = 0x01D,  // save to NVM / reboot, magic-key gated

        MSG_CO_HEARTBEAT_CMD = 0x700,  // CANOpen NMT Heartbeat  SEND
    };

    // Parameters reachable through MSG_CONFIG_ACCESS.
    //
    // 0x0x -- the JOINT (load) encoder. Resolved through
    //         controller.config.load_encoder_axis, exactly like the
    //         Get_Encoder_Estimates patch, so a split-feedback geared joint is
    //         configured through the MOTOR axis's node id. The load encoder
    //         (axis1) normally has its CAN heartbeat muted and is not
    //         separately addressable on the bus.
    // 0x1x -- this axis's controller.   0x2x -- this axis's motor.
    // 0x3x -- read-only telemetry that CAN Simple otherwise cannot reach.
    // 0x4x -- node / bus identity.
    enum ConfigParam {
        PARAM_JOINT_MIN_POSITION = 0x01,  // float  [turn]
        PARAM_JOINT_MAX_POSITION = 0x02,  // float  [turn]
        PARAM_JOINT_LIMIT_ENABLE = 0x03,  // bool
        PARAM_JOINT_DIRECTION = 0x04,     // int32  +1 / -1
        PARAM_JOINT_ZERO_OFFSET = 0x05,   // int32  [count]
        PARAM_JOINT_SET_ZERO = 0x06,      // write any: capture current pose as zero
        PARAM_JOINT_RESEED = 0x07,        // write any: re-seed linear pos from count_in_cpr
        PARAM_JOINT_POS_ESTIMATE = 0x08,  // float  [turn]   read-only
        PARAM_JOINT_COUNT_IN_CPR = 0x09,  // int32  [count]  read-only
        PARAM_JOINT_CPR = 0x0A,           // int32  [count]  read-only
        PARAM_JOINT_ENCODER_ERROR = 0x0B, // uint32          read-only
        PARAM_JOINT_TURN_SNAPS = 0x0C,    // uint32          read-only

        PARAM_POS_GAIN = 0x10,                // float
        PARAM_VEL_GAIN = 0x11,                // float
        PARAM_VEL_INTEGRATOR_GAIN = 0x12,     // float
        PARAM_VEL_LIMIT = 0x13,               // float  [turn/s]
        PARAM_POSITION_DIRECTION = 0x14,      // int32  +1 / -1
        PARAM_LOAD_ENCODER_AXIS = 0x15,       // int32           idle only
        PARAM_VEL_ENCODER_AXIS = 0x16,        // int32           idle only
        PARAM_INPUT_FILTER_BANDWIDTH = 0x17,  // float  [1/s]

        PARAM_CURRENT_LIM = 0x20,      // float  [A]
        PARAM_TORQUE_CONSTANT = 0x21,  // float  [Nm/A]

        PARAM_FET_TEMPERATURE = 0x30,     // float  [degC]  read-only
        PARAM_MOTOR_TEMPERATURE = 0x31,   // float  [degC]  read-only
        PARAM_MOTOR_THERM_ENABLE = 0x32,  // bool

        PARAM_CAN_NODE_ID = 0x40,           // int32          idle only
        PARAM_CAN_HEARTBEAT_RATE = 0x41,    // int32  [ms]    this axis
        PARAM_OTHER_AXIS_HEARTBEAT = 0x42,  // int32  [ms]    the other axis
    };

    enum ConfigOp {
        CONFIG_OP_READ = 0,
        CONFIG_OP_WRITE = 1,
        CONFIG_OP_ERROR_FLAG = 0x80,  // OR'd into byte0 of a failed reply
    };

    enum ConfigStatus {
        CONFIG_OK = 0,
        CONFIG_UNKNOWN_PARAM = 1,
        CONFIG_READ_ONLY = 2,
        CONFIG_VALUE_REJECTED = 3,
        CONFIG_REQUIRES_IDLE = 4,
        CONFIG_NO_TARGET = 5,   // e.g. load_encoder_axis points nowhere
        CONFIG_BAD_MAGIC = 6,
        CONFIG_BUSY = 7,
        CONFIG_SAVE_DONE = 0x10,
        CONFIG_SAVE_FAILED = 0x11,
    };

    enum ConfigType {
        CONFIG_TYPE_FLOAT = 0,
        CONFIG_TYPE_INT32 = 1,
        CONFIG_TYPE_BOOL = 2,
        CONFIG_TYPE_UINT32 = 3,
    };

    enum ConfigAction {
        CONFIG_ACTION_SAVE = 1,
        CONFIG_ACTION_REBOOT = 2,
    };

    // Guards MSG_CONFIG_COMMIT so bus noise or a stray frame can never erase a
    // flash sector under a running robot.
    static constexpr uint32_t CONFIG_COMMIT_MAGIC = 0x0DC0FFEE;

    static void handle_can_message(can_Message_t& msg);
    static void send_heartbeat(Axis* axis);
    // Called from the communication thread once a deferred NVM save finishes.
    static void send_config_commit_reply(uint32_t node_id, bool is_ext,
                                         uint8_t action, uint8_t status);

   private:
    static void nmt_callback(Axis* axis, can_Message_t& msg);
    static void estop_callback(Axis* axis, can_Message_t& msg);
    static void get_motor_error_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_error_callback(Axis* axis, can_Message_t& msg);
    static void get_controller_error_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_error_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_nodeid_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_requested_state_callback(Axis* axis, can_Message_t& msg);
    static void set_axis_startup_config_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_encoder_count_callback(Axis* axis, can_Message_t& msg);
    static void set_input_pos_callback(Axis* axis, can_Message_t& msg);
    static void set_input_vel_callback(Axis* axis, can_Message_t& msg);
    static void set_input_torque_callback(Axis* axis, can_Message_t& msg);
    static void set_controller_modes_callback(Axis* axis, can_Message_t& msg);
    static void set_limits_callback(Axis* axis, can_Message_t& msg);
    static void start_anticogging_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg);
    static void set_traj_inertia_callback(Axis* axis, can_Message_t& msg);
    static void get_iq_callback(Axis* axis, can_Message_t& msg);
    static void get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg);
    static void get_vbus_voltage_callback(Axis* axis, can_Message_t& msg);
    static void clear_errors_callback(Axis* axis, can_Message_t& msg);
    static void config_access_callback(Axis* axis, can_Message_t& msg);
    static void config_commit_callback(Axis* axis, can_Message_t& msg);

    // Utility functions
    static uint32_t get_node_id(uint32_t msgID);
    static uint8_t get_cmd_id(uint32_t msgID);

    // Fetch a specific signal from the message

    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking
    //
    // const std::map<uint32_t, std::function<void(can_Message_t&)>> callback_map = {
    //     {0x000, std::bind(&CANSimple::heartbeat_callback, this, _1)}
    // };
};





#endif
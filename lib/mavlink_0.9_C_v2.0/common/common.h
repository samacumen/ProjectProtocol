/** @file
 *	@brief MAVLink comm protocol generated from common.xml
 *	@see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_COMMON_H
#define MAVLINK_COMMON_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_COMMON.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{0, 72, 3, 0, 0, 0}, {1, 39, 4, 0, 0, 0}, {2, 190, 8, 0, 0, 0}, {3, 105, 14, 3, 12, 13}, {4, 191, 8, 0, 0, 0}, {5, 217, 28, 1, 0, 0}, {6, 104, 3, 0, 0, 0}, {7, 119, 32, 0, 0, 0}, {9, 219, 2, 0, 0, 0}, {10, 60, 3, 2, 0, 1}, {11, 186, 2, 0, 0, 0}, {12, 10, 2, 0, 0, 0}, {20, 39, 19, 3, 2, 3}, {21, 159, 2, 3, 0, 1}, {22, 205, 23, 0, 0, 0}, {23, 244, 21, 3, 4, 5}, {25, 180, 37, 0, 0, 0}, {26, 222, 26, 0, 0, 0}, {27, 110, 101, 0, 0, 0}, {28, 179, 26, 0, 0, 0}, {29, 136, 16, 0, 0, 0}, {30, 66, 32, 0, 0, 0}, {31, 126, 32, 0, 0, 0}, {32, 220, 37, 0, 0, 0}, {33, 147, 32, 0, 0, 0}, {34, 64, 11, 0, 0, 0}, {35, 252, 17, 0, 0, 0}, {36, 162, 17, 0, 0, 0}, {37, 215, 16, 0, 0, 0}, {38, 229, 18, 0, 0, 0}, {39, 205, 36, 3, 30, 31}, {40, 51, 4, 3, 2, 3}, {41, 80, 4, 3, 2, 3}, {42, 101, 2, 0, 0, 0}, {43, 213, 2, 3, 0, 1}, {44, 8, 4, 3, 2, 3}, {45, 229, 2, 3, 0, 1}, {46, 21, 2, 0, 0, 0}, {47, 214, 3, 3, 0, 1}, {48, 170, 14, 3, 12, 13}, {49, 14, 12, 0, 0, 0}, {50, 73, 18, 3, 16, 17}, {51, 50, 16, 0, 0, 0}, {52, 157, 8, 0, 0, 0}, {53, 15, 27, 3, 24, 25}, {54, 3, 25, 0, 0, 0}, {55, 100, 18, 3, 16, 17}, {56, 24, 18, 3, 16, 17}, {57, 5, 24, 0, 0, 0}, {58, 127, 24, 0, 0, 0}, {62, 183, 26, 0, 0, 0}, {63, 126, 16, 0, 0, 0}, {64, 130, 36, 0, 0, 0}, {65, 54, 5, 0, 0, 0}, {66, 148, 6, 3, 2, 3}, {67, 191, 56, 0, 0, 0}, {68, 236, 26, 0, 0, 0}, {69, 52, 21, 0, 0, 0}, {70, 124, 18, 3, 16, 17}, {73, 104, 18, 0, 0, 0}, {74, 20, 20, 0, 0, 0}, {75, 133, 20, 3, 16, 17}, {76, 8, 8, 0, 0, 0}, {100, 146, 18, 0, 0, 0}, {140, 179, 36, 0, 0, 0}, {251, 15, 30, 0, 0, 0}, {252, 248, 14, 0, 0, 0}, {253, 63, 14, 0, 0, 0}, {254, 106, 51, 0, 0, 0}, {255, 127, 5, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_COMMON

// ENUM DEFINITIONS


/** @brief Data stream IDs. A data stream is not a fixed set of messages, but rather a
     recommendation to the autopilot software. Individual autopilots may or may not obey
     the recommended messages.
      */
#ifndef HAVE_ENUM_MAV_DATA_STREAM
#define HAVE_ENUM_MAV_DATA_STREAM
typedef enum MAV_DATA_STREAM
{
	MAV_DATA_STREAM_ALL=0, /* Enable all data streams | */
	MAV_DATA_STREAM_RAW_SENSORS=1, /* Enable IMU_RAW, GPS_RAW, GPS_STATUS packets. | */
	MAV_DATA_STREAM_EXTENDED_STATUS=2, /* Enable GPS_STATUS, CONTROL_STATUS, AUX_STATUS | */
	MAV_DATA_STREAM_RC_CHANNELS=3, /* Enable RC_CHANNELS_SCALED, RC_CHANNELS_RAW, SERVO_OUTPUT_RAW | */
	MAV_DATA_STREAM_RAW_CONTROLLER=4, /* Enable ATTITUDE_CONTROLLER_OUTPUT, POSITION_CONTROLLER_OUTPUT, NAV_CONTROLLER_OUTPUT. | */
	MAV_DATA_STREAM_POSITION=6, /* Enable LOCAL_POSITION, GLOBAL_POSITION/GLOBAL_POSITION_INT messages. | */
	MAV_DATA_STREAM_EXTRA1=10, /* Dependent on the autopilot | */
	MAV_DATA_STREAM_EXTRA2=11, /* Dependent on the autopilot | */
	MAV_DATA_STREAM_EXTRA3=12, /* Dependent on the autopilot | */
	MAV_DATA_STREAM_ENUM_END=13, /*  | */
} MAV_DATA_STREAM;
#endif

/** @brief  The ROI (region of interest) for the vehicle. This can be
                be used by the vehicle for camera/vehicle attitude alignment (see
                MAV_CMD_NAV_ROI).
             */
#ifndef HAVE_ENUM_MAV_ROI
#define HAVE_ENUM_MAV_ROI
typedef enum MAV_ROI
{
	MAV_ROI_NONE=0, /* No region of interest. | */
	MAV_ROI_WPNEXT=1, /* Point toward next waypoint. | */
	MAV_ROI_WPINDEX=2, /* Point toward given waypoint. | */
	MAV_ROI_LOCATION=3, /* Point toward fixed location. | */
	MAV_ROI_TARGET=4, /* Point toward of given id. | */
	MAV_ROI_ENUM_END=5, /*  | */
} MAV_ROI;
#endif

// MAVLINK VERSION

#ifndef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

#if (MAVLINK_VERSION == 0)
#undef MAVLINK_VERSION
#define MAVLINK_VERSION 2
#endif

// MESSAGE DEFINITIONS
#include "./mavlink_msg_heartbeat.h"
#include "./mavlink_msg_boot.h"
#include "./mavlink_msg_system_time.h"
#include "./mavlink_msg_ping.h"
#include "./mavlink_msg_system_time_utc.h"
#include "./mavlink_msg_change_operator_control.h"
#include "./mavlink_msg_change_operator_control_ack.h"
#include "./mavlink_msg_auth_key.h"
#include "./mavlink_msg_action_ack.h"
#include "./mavlink_msg_action.h"
#include "./mavlink_msg_set_mode.h"
#include "./mavlink_msg_set_nav_mode.h"
#include "./mavlink_msg_param_request_read.h"
#include "./mavlink_msg_param_request_list.h"
#include "./mavlink_msg_param_value.h"
#include "./mavlink_msg_param_set.h"
#include "./mavlink_msg_gps_raw_int.h"
#include "./mavlink_msg_scaled_imu.h"
#include "./mavlink_msg_gps_status.h"
#include "./mavlink_msg_raw_imu.h"
#include "./mavlink_msg_raw_pressure.h"
#include "./mavlink_msg_scaled_pressure.h"
#include "./mavlink_msg_attitude.h"
#include "./mavlink_msg_local_position.h"
#include "./mavlink_msg_global_position.h"
#include "./mavlink_msg_gps_raw.h"
#include "./mavlink_msg_sys_status.h"
#include "./mavlink_msg_rc_channels_raw.h"
#include "./mavlink_msg_rc_channels_scaled.h"
#include "./mavlink_msg_servo_output_raw.h"
#include "./mavlink_msg_waypoint.h"
#include "./mavlink_msg_waypoint_request.h"
#include "./mavlink_msg_waypoint_set_current.h"
#include "./mavlink_msg_waypoint_current.h"
#include "./mavlink_msg_waypoint_request_list.h"
#include "./mavlink_msg_waypoint_count.h"
#include "./mavlink_msg_waypoint_clear_all.h"
#include "./mavlink_msg_waypoint_reached.h"
#include "./mavlink_msg_waypoint_ack.h"
#include "./mavlink_msg_gps_set_global_origin.h"
#include "./mavlink_msg_gps_local_origin_set.h"
#include "./mavlink_msg_local_position_setpoint_set.h"
#include "./mavlink_msg_local_position_setpoint.h"
#include "./mavlink_msg_control_status.h"
#include "./mavlink_msg_safety_set_allowed_area.h"
#include "./mavlink_msg_safety_allowed_area.h"
#include "./mavlink_msg_set_roll_pitch_yaw_thrust.h"
#include "./mavlink_msg_set_roll_pitch_yaw_speed_thrust.h"
#include "./mavlink_msg_roll_pitch_yaw_thrust_setpoint.h"
#include "./mavlink_msg_roll_pitch_yaw_speed_thrust_setpoint.h"
#include "./mavlink_msg_nav_controller_output.h"
#include "./mavlink_msg_position_target.h"
#include "./mavlink_msg_state_correction.h"
#include "./mavlink_msg_set_altitude.h"
#include "./mavlink_msg_request_data_stream.h"
#include "./mavlink_msg_hil_state.h"
#include "./mavlink_msg_hil_controls.h"
#include "./mavlink_msg_manual_control.h"
#include "./mavlink_msg_rc_channels_override.h"
#include "./mavlink_msg_global_position_int.h"
#include "./mavlink_msg_vfr_hud.h"
#include "./mavlink_msg_command.h"
#include "./mavlink_msg_command_ack.h"
#include "./mavlink_msg_optical_flow.h"
#include "./mavlink_msg_object_detection_event.h"
#include "./mavlink_msg_debug_vect.h"
#include "./mavlink_msg_named_value_float.h"
#include "./mavlink_msg_named_value_int.h"
#include "./mavlink_msg_statustext.h"
#include "./mavlink_msg_debug.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 1

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_HEARTBEAT, MAVLINK_MESSAGE_INFO_BOOT, MAVLINK_MESSAGE_INFO_SYSTEM_TIME, MAVLINK_MESSAGE_INFO_PING, MAVLINK_MESSAGE_INFO_SYSTEM_TIME_UTC, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL, MAVLINK_MESSAGE_INFO_CHANGE_OPERATOR_CONTROL_ACK, MAVLINK_MESSAGE_INFO_AUTH_KEY, MAVLINK_MESSAGE_INFO_ACTION_ACK, MAVLINK_MESSAGE_INFO_ACTION, MAVLINK_MESSAGE_INFO_SET_MODE, MAVLINK_MESSAGE_INFO_SET_NAV_MODE, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_READ, MAVLINK_MESSAGE_INFO_PARAM_REQUEST_LIST, MAVLINK_MESSAGE_INFO_PARAM_VALUE, MAVLINK_MESSAGE_INFO_PARAM_SET, MAVLINK_MESSAGE_INFO_GPS_RAW_INT, MAVLINK_MESSAGE_INFO_SCALED_IMU, MAVLINK_MESSAGE_INFO_GPS_STATUS, MAVLINK_MESSAGE_INFO_RAW_IMU, MAVLINK_MESSAGE_INFO_RAW_PRESSURE, MAVLINK_MESSAGE_INFO_ATTITUDE, MAVLINK_MESSAGE_INFO_LOCAL_POSITION, MAVLINK_MESSAGE_INFO_GPS_RAW, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION, MAVLINK_MESSAGE_INFO_SYS_STATUS, MAVLINK_MESSAGE_INFO_RC_CHANNELS_RAW, MAVLINK_MESSAGE_INFO_RC_CHANNELS_SCALED, MAVLINK_MESSAGE_INFO_SERVO_OUTPUT_RAW, MAVLINK_MESSAGE_INFO_SCALED_PRESSURE, MAVLINK_MESSAGE_INFO_WAYPOINT, MAVLINK_MESSAGE_INFO_WAYPOINT_REQUEST, MAVLINK_MESSAGE_INFO_WAYPOINT_SET_CURRENT, MAVLINK_MESSAGE_INFO_WAYPOINT_CURRENT, MAVLINK_MESSAGE_INFO_WAYPOINT_REQUEST_LIST, MAVLINK_MESSAGE_INFO_WAYPOINT_COUNT, MAVLINK_MESSAGE_INFO_WAYPOINT_CLEAR_ALL, MAVLINK_MESSAGE_INFO_WAYPOINT_REACHED, MAVLINK_MESSAGE_INFO_WAYPOINT_ACK, MAVLINK_MESSAGE_INFO_GPS_SET_GLOBAL_ORIGIN, MAVLINK_MESSAGE_INFO_GPS_LOCAL_ORIGIN_SET, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_SETPOINT_SET, MAVLINK_MESSAGE_INFO_LOCAL_POSITION_SETPOINT, MAVLINK_MESSAGE_INFO_CONTROL_STATUS, MAVLINK_MESSAGE_INFO_SAFETY_SET_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SAFETY_ALLOWED_AREA, MAVLINK_MESSAGE_INFO_SET_ROLL_PITCH_YAW_THRUST, MAVLINK_MESSAGE_INFO_SET_ROLL_PITCH_YAW_SPEED_THRUST, MAVLINK_MESSAGE_INFO_ROLL_PITCH_YAW_THRUST_SETPOINT, MAVLINK_MESSAGE_INFO_ROLL_PITCH_YAW_SPEED_THRUST_SETPOINT, MAVLINK_MESSAGE_INFO_NAV_CONTROLLER_OUTPUT, MAVLINK_MESSAGE_INFO_POSITION_TARGET, MAVLINK_MESSAGE_INFO_STATE_CORRECTION, MAVLINK_MESSAGE_INFO_SET_ALTITUDE, MAVLINK_MESSAGE_INFO_REQUEST_DATA_STREAM, MAVLINK_MESSAGE_INFO_HIL_STATE, MAVLINK_MESSAGE_INFO_HIL_CONTROLS, MAVLINK_MESSAGE_INFO_MANUAL_CONTROL, MAVLINK_MESSAGE_INFO_RC_CHANNELS_OVERRIDE, MAVLINK_MESSAGE_INFO_GLOBAL_POSITION_INT, MAVLINK_MESSAGE_INFO_VFR_HUD, MAVLINK_MESSAGE_INFO_COMMAND, MAVLINK_MESSAGE_INFO_COMMAND_ACK, MAVLINK_MESSAGE_INFO_OPTICAL_FLOW, MAVLINK_MESSAGE_INFO_OBJECT_DETECTION_EVENT, MAVLINK_MESSAGE_INFO_DEBUG_VECT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_FLOAT, MAVLINK_MESSAGE_INFO_NAMED_VALUE_INT, MAVLINK_MESSAGE_INFO_STATUSTEXT, MAVLINK_MESSAGE_INFO_DEBUG}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_COMMON_H

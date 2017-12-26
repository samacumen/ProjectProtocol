/** @file
 *  @brief MAVLink comm protocol generated from dronebox_messages.xml
 *  @see http://mavlink.org
 */
#pragma once
#ifndef MAVLINK_DRONEBOX_MESSAGES_H
#define MAVLINK_DRONEBOX_MESSAGES_H

#ifndef MAVLINK_H
    #error Wrong include order: MAVLINK_DRONEBOX_MESSAGES.H MUST NOT BE DIRECTLY USED. Include mavlink.h from the same directory instead or set ALL AND EVERY defines from MAVLINK.H manually accordingly, including the #define MAVLINK_H call.
#endif

#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 2

#ifdef __cplusplus
extern "C" {
#endif

// MESSAGE LENGTHS AND CRCS

#ifndef MAVLINK_MESSAGE_LENGTHS
#define MAVLINK_MESSAGE_LENGTHS {}
#endif

#ifndef MAVLINK_MESSAGE_CRCS
#define MAVLINK_MESSAGE_CRCS {{500, 212, 1, 0, 0, 0}, {501, 167, 1, 0, 0, 0}, {502, 122, 1, 0, 0, 0}, {510, 107, 1, 0, 0, 0}}
#endif

#include "../protocol.h"

#define MAVLINK_ENABLED_DRONEBOX_MESSAGES

// ENUM DEFINITIONS


/** @brief  Enumeration of the actions that can be performed on actuators. */
#ifndef HAVE_ENUM_DS_ACTUATOR_DO_ACTION_TYPE
#define HAVE_ENUM_DS_ACTUATOR_DO_ACTION_TYPE
typedef enum DS_ACTUATOR_DO_ACTION_TYPE
{
   DS_ACTUATOR_DO_ACTION_TYPE_OPEN_SHUTTER=0, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_CLOSE_SHUTTER=1, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_STOP_SHUTTER=2, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_RAISE_PLATFORM=3, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_LOWER_PLATFORM=4, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_STOP_PLATFORM=5, /*   | */
   DS_ACTUATOR_DO_ACTION_TYPE_ENUM_END=6, /*  | */
} DS_ACTUATOR_DO_ACTION_TYPE;
#endif

/** @brief  Enumeration of the actions that can be performed on XY table. */
#ifndef HAVE_ENUM_DS_XYTABLE_DO_ACTION_TYPE
#define HAVE_ENUM_DS_XYTABLE_DO_ACTION_TYPE
typedef enum DS_XYTABLE_DO_ACTION_TYPE
{
   DS_XYTABLE_DO_ACTION_TYPE_START=0, /*   | */
   DS_XYTABLE_DO_ACTION_TYPE_STOP=1, /*   | */
   DS_XYTABLE_DO_ACTION_TYPE_ENUM_END=2, /*  | */
} DS_XYTABLE_DO_ACTION_TYPE;
#endif

/** @brief  Enumeration of the slantrage sync actions (namely, start and stop). */
#ifndef HAVE_ENUM_BS_SLANTRANGE_SYNC_ACTION_TYPE
#define HAVE_ENUM_BS_SLANTRANGE_SYNC_ACTION_TYPE
typedef enum BS_SLANTRANGE_SYNC_ACTION_TYPE
{
   BS_SLANTRANGE_SYNC_ACTION_TYPE_START=0, /*   | */
   BS_SLANTRANGE_SYNC_ACTION_TYPE_STOP=1, /*   | */
   BS_SLANTRANGE_SYNC_ACTION_TYPE_ENUM_END=2, /*  | */
} BS_SLANTRANGE_SYNC_ACTION_TYPE;
#endif

/** @brief  Enumeration of the drone station acknowledgment cmds sent to remote user station. */
#ifndef HAVE_ENUM_DS_RECEIVED_CMD_ACK_TYPE
#define HAVE_ENUM_DS_RECEIVED_CMD_ACK_TYPE
typedef enum DS_RECEIVED_CMD_ACK_TYPE
{
   DS_RECEIVED_CMD_ACK_TYPE_SHUTTER_OPEN_REQ=0, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_SHUTTER_CLOSE_REQ=1, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_SHUTTER_STOP_REQ=2, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_PLATFORM_RAISE_REQ=3, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_PLATFORM_LOWER_REQ=4, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_PLATFORM_STOP_REQ=5, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_XYTABLE_START_REQ=6, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_XYTABLE_STOP_REQ=7, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_FAILED=8, /*   | */
   DS_RECEIVED_CMD_ACK_TYPE_ENUM_END=9, /*  | */
} DS_RECEIVED_CMD_ACK_TYPE;
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
#include "./mavlink_msg_ds_actuator_do_action.h"
#include "./mavlink_msg_ds_xytable_do_action.h"
#include "./mavlink_msg_bs_slantrange_do_sync.h"
#include "./mavlink_msg_ds_received_cmd_ack.h"

// base include


#undef MAVLINK_THIS_XML_IDX
#define MAVLINK_THIS_XML_IDX 2

#if MAVLINK_THIS_XML_IDX == MAVLINK_PRIMARY_XML_IDX
# define MAVLINK_MESSAGE_INFO {MAVLINK_MESSAGE_INFO_DS_ACTUATOR_DO_ACTION, MAVLINK_MESSAGE_INFO_DS_XYTABLE_DO_ACTION, MAVLINK_MESSAGE_INFO_BS_SLANTRANGE_DO_SYNC, MAVLINK_MESSAGE_INFO_DS_RECEIVED_CMD_ACK}
# if MAVLINK_COMMAND_24BIT
#  include "../mavlink_get_info.h"
# endif
#endif

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // MAVLINK_DRONEBOX_MESSAGES_H

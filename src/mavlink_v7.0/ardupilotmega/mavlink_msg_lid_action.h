#pragma once
// MESSAGE LID_ACTION PACKING

#define MAVLINK_MSG_ID_LID_ACTION 190

MAVPACKED(
typedef struct __mavlink_lid_action_t {
 uint16_t action_type; /*< 1 refers to opening the lid, 0 acts on closing the lid*/
}) mavlink_lid_action_t;

#define MAVLINK_MSG_ID_LID_ACTION_LEN 2
#define MAVLINK_MSG_ID_LID_ACTION_MIN_LEN 2
#define MAVLINK_MSG_ID_190_LEN 2
#define MAVLINK_MSG_ID_190_MIN_LEN 2

#define MAVLINK_MSG_ID_LID_ACTION_CRC 249
#define MAVLINK_MSG_ID_190_CRC 249



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_LID_ACTION { \
	190, \
	"LID_ACTION", \
	1, \
	{  { "action_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lid_action_t, action_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_LID_ACTION { \
	"LID_ACTION", \
	1, \
	{  { "action_type", NULL, MAVLINK_TYPE_UINT16_T, 0, 0, offsetof(mavlink_lid_action_t, action_type) }, \
         } \
}
#endif

/**
 * @brief Pack a lid_action message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param action_type 1 refers to opening the lid, 0 acts on closing the lid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lid_action_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint16_t action_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LID_ACTION_LEN];
	_mav_put_uint16_t(buf, 0, action_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LID_ACTION_LEN);
#else
	mavlink_lid_action_t packet;
	packet.action_type = action_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LID_ACTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LID_ACTION;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
}

/**
 * @brief Pack a lid_action message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param action_type 1 refers to opening the lid, 0 acts on closing the lid
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_lid_action_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint16_t action_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LID_ACTION_LEN];
	_mav_put_uint16_t(buf, 0, action_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_LID_ACTION_LEN);
#else
	mavlink_lid_action_t packet;
	packet.action_type = action_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_LID_ACTION_LEN);
#endif

	msg->msgid = MAVLINK_MSG_ID_LID_ACTION;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
}

/**
 * @brief Encode a lid_action struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param lid_action C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lid_action_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_lid_action_t* lid_action)
{
	return mavlink_msg_lid_action_pack(system_id, component_id, msg, lid_action->action_type);
}

/**
 * @brief Encode a lid_action struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param lid_action C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_lid_action_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_lid_action_t* lid_action)
{
	return mavlink_msg_lid_action_pack_chan(system_id, component_id, chan, msg, lid_action->action_type);
}

/**
 * @brief Send a lid_action message
 * @param chan MAVLink channel to send the message
 *
 * @param action_type 1 refers to opening the lid, 0 acts on closing the lid
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_lid_action_send(mavlink_channel_t chan, uint16_t action_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[MAVLINK_MSG_ID_LID_ACTION_LEN];
	_mav_put_uint16_t(buf, 0, action_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LID_ACTION, buf, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
#else
	mavlink_lid_action_t packet;
	packet.action_type = action_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LID_ACTION, (const char *)&packet, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
#endif
}

/**
 * @brief Send a lid_action message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_lid_action_send_struct(mavlink_channel_t chan, const mavlink_lid_action_t* lid_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_lid_action_send(chan, lid_action->action_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LID_ACTION, (const char *)lid_action, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
#endif
}

#if MAVLINK_MSG_ID_LID_ACTION_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_lid_action_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint16_t action_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char *buf = (char *)msgbuf;
	_mav_put_uint16_t(buf, 0, action_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LID_ACTION, buf, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
#else
	mavlink_lid_action_t *packet = (mavlink_lid_action_t *)msgbuf;
	packet->action_type = action_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_LID_ACTION, (const char *)packet, MAVLINK_MSG_ID_LID_ACTION_MIN_LEN, MAVLINK_MSG_ID_LID_ACTION_LEN, MAVLINK_MSG_ID_LID_ACTION_CRC);
#endif
}
#endif

#endif

// MESSAGE LID_ACTION UNPACKING


/**
 * @brief Get field action_type from lid_action message
 *
 * @return 1 refers to opening the lid, 0 acts on closing the lid
 */
static inline uint16_t mavlink_msg_lid_action_get_action_type(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint16_t(msg,  0);
}

/**
 * @brief Decode a lid_action message into a struct
 *
 * @param msg The message to decode
 * @param lid_action C-struct to decode the message contents into
 */
static inline void mavlink_msg_lid_action_decode(const mavlink_message_t* msg, mavlink_lid_action_t* lid_action)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	lid_action->action_type = mavlink_msg_lid_action_get_action_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_LID_ACTION_LEN? msg->len : MAVLINK_MSG_ID_LID_ACTION_LEN;
        memset(lid_action, 0, MAVLINK_MSG_ID_LID_ACTION_LEN);
	memcpy(lid_action, _MAV_PAYLOAD(msg), len);
#endif
}

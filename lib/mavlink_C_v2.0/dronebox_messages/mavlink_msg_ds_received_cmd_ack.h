#pragma once
// MESSAGE DS_RECEIVED_CMD_ACK PACKING

#define MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK 510

MAVPACKED(
typedef struct __mavlink_ds_received_cmd_ack_t {
 uint8_t cmd_ackd; /*< The type of acknowledgment issued back to remote user station*/
}) mavlink_ds_received_cmd_ack_t;

#define MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN 1
#define MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN 1
#define MAVLINK_MSG_ID_510_LEN 1
#define MAVLINK_MSG_ID_510_MIN_LEN 1

#define MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC 107
#define MAVLINK_MSG_ID_510_CRC 107



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_DS_RECEIVED_CMD_ACK { \
    510, \
    "DS_RECEIVED_CMD_ACK", \
    1, \
    {  { "cmd_ackd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ds_received_cmd_ack_t, cmd_ackd) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_DS_RECEIVED_CMD_ACK { \
    "DS_RECEIVED_CMD_ACK", \
    1, \
    {  { "cmd_ackd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_ds_received_cmd_ack_t, cmd_ackd) }, \
         } \
}
#endif

/**
 * @brief Pack a ds_received_cmd_ack message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param cmd_ackd The type of acknowledgment issued back to remote user station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ds_received_cmd_ack_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t cmd_ackd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN];
    _mav_put_uint8_t(buf, 0, cmd_ackd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN);
#else
    mavlink_ds_received_cmd_ack_t packet;
    packet.cmd_ackd = cmd_ackd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
}

/**
 * @brief Pack a ds_received_cmd_ack message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param cmd_ackd The type of acknowledgment issued back to remote user station
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_ds_received_cmd_ack_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t cmd_ackd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN];
    _mav_put_uint8_t(buf, 0, cmd_ackd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN);
#else
    mavlink_ds_received_cmd_ack_t packet;
    packet.cmd_ackd = cmd_ackd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
}

/**
 * @brief Encode a ds_received_cmd_ack struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param ds_received_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ds_received_cmd_ack_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_ds_received_cmd_ack_t* ds_received_cmd_ack)
{
    return mavlink_msg_ds_received_cmd_ack_pack(system_id, component_id, msg, ds_received_cmd_ack->cmd_ackd);
}

/**
 * @brief Encode a ds_received_cmd_ack struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param ds_received_cmd_ack C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_ds_received_cmd_ack_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_ds_received_cmd_ack_t* ds_received_cmd_ack)
{
    return mavlink_msg_ds_received_cmd_ack_pack_chan(system_id, component_id, chan, msg, ds_received_cmd_ack->cmd_ackd);
}

/**
 * @brief Send a ds_received_cmd_ack message
 * @param chan MAVLink channel to send the message
 *
 * @param cmd_ackd The type of acknowledgment issued back to remote user station
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_ds_received_cmd_ack_send(mavlink_channel_t chan, uint8_t cmd_ackd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN];
    _mav_put_uint8_t(buf, 0, cmd_ackd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK, buf, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
#else
    mavlink_ds_received_cmd_ack_t packet;
    packet.cmd_ackd = cmd_ackd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK, (const char *)&packet, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
#endif
}

/**
 * @brief Send a ds_received_cmd_ack message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_ds_received_cmd_ack_send_struct(mavlink_channel_t chan, const mavlink_ds_received_cmd_ack_t* ds_received_cmd_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_ds_received_cmd_ack_send(chan, ds_received_cmd_ack->cmd_ackd);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK, (const char *)ds_received_cmd_ack, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
#endif
}

#if MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_ds_received_cmd_ack_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t cmd_ackd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, cmd_ackd);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK, buf, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
#else
    mavlink_ds_received_cmd_ack_t *packet = (mavlink_ds_received_cmd_ack_t *)msgbuf;
    packet->cmd_ackd = cmd_ackd;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK, (const char *)packet, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_CRC);
#endif
}
#endif

#endif

// MESSAGE DS_RECEIVED_CMD_ACK UNPACKING


/**
 * @brief Get field cmd_ackd from ds_received_cmd_ack message
 *
 * @return The type of acknowledgment issued back to remote user station
 */
static inline uint8_t mavlink_msg_ds_received_cmd_ack_get_cmd_ackd(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a ds_received_cmd_ack message into a struct
 *
 * @param msg The message to decode
 * @param ds_received_cmd_ack C-struct to decode the message contents into
 */
static inline void mavlink_msg_ds_received_cmd_ack_decode(const mavlink_message_t* msg, mavlink_ds_received_cmd_ack_t* ds_received_cmd_ack)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    ds_received_cmd_ack->cmd_ackd = mavlink_msg_ds_received_cmd_ack_get_cmd_ackd(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN? msg->len : MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN;
        memset(ds_received_cmd_ack, 0, MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_LEN);
    memcpy(ds_received_cmd_ack, _MAV_PAYLOAD(msg), len);
#endif
}

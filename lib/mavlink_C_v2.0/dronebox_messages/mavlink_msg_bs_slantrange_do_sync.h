#pragma once
// MESSAGE BS_SLANTRANGE_DO_SYNC PACKING

#define MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC 502

MAVPACKED(
typedef struct __mavlink_bs_slantrange_do_sync_t {
 uint8_t sync_type; /*< Action to trigger start/stop sync, see BS_SLANTRANGE_SYNC_ACTION_TYPE*/
}) mavlink_bs_slantrange_do_sync_t;

#define MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN 1
#define MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN 1
#define MAVLINK_MSG_ID_502_LEN 1
#define MAVLINK_MSG_ID_502_MIN_LEN 1

#define MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC 122
#define MAVLINK_MSG_ID_502_CRC 122



#if MAVLINK_COMMAND_24BIT
#define MAVLINK_MESSAGE_INFO_BS_SLANTRANGE_DO_SYNC { \
    502, \
    "BS_SLANTRANGE_DO_SYNC", \
    1, \
    {  { "sync_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_bs_slantrange_do_sync_t, sync_type) }, \
         } \
}
#else
#define MAVLINK_MESSAGE_INFO_BS_SLANTRANGE_DO_SYNC { \
    "BS_SLANTRANGE_DO_SYNC", \
    1, \
    {  { "sync_type", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_bs_slantrange_do_sync_t, sync_type) }, \
         } \
}
#endif

/**
 * @brief Pack a bs_slantrange_do_sync message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param sync_type Action to trigger start/stop sync, see BS_SLANTRANGE_SYNC_ACTION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bs_slantrange_do_sync_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
                               uint8_t sync_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, sync_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN);
#else
    mavlink_bs_slantrange_do_sync_t packet;
    packet.sync_type = sync_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC;
    return mavlink_finalize_message(msg, system_id, component_id, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
}

/**
 * @brief Pack a bs_slantrange_do_sync message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param sync_type Action to trigger start/stop sync, see BS_SLANTRANGE_SYNC_ACTION_TYPE
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_bs_slantrange_do_sync_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
                               mavlink_message_t* msg,
                                   uint8_t sync_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, sync_type);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN);
#else
    mavlink_bs_slantrange_do_sync_t packet;
    packet.sync_type = sync_type;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN);
#endif

    msg->msgid = MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC;
    return mavlink_finalize_message_chan(msg, system_id, component_id, chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
}

/**
 * @brief Encode a bs_slantrange_do_sync struct
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param bs_slantrange_do_sync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bs_slantrange_do_sync_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_bs_slantrange_do_sync_t* bs_slantrange_do_sync)
{
    return mavlink_msg_bs_slantrange_do_sync_pack(system_id, component_id, msg, bs_slantrange_do_sync->sync_type);
}

/**
 * @brief Encode a bs_slantrange_do_sync struct on a channel
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message will be sent over
 * @param msg The MAVLink message to compress the data into
 * @param bs_slantrange_do_sync C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_bs_slantrange_do_sync_encode_chan(uint8_t system_id, uint8_t component_id, uint8_t chan, mavlink_message_t* msg, const mavlink_bs_slantrange_do_sync_t* bs_slantrange_do_sync)
{
    return mavlink_msg_bs_slantrange_do_sync_pack_chan(system_id, component_id, chan, msg, bs_slantrange_do_sync->sync_type);
}

/**
 * @brief Send a bs_slantrange_do_sync message
 * @param chan MAVLink channel to send the message
 *
 * @param sync_type Action to trigger start/stop sync, see BS_SLANTRANGE_SYNC_ACTION_TYPE
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_bs_slantrange_do_sync_send(mavlink_channel_t chan, uint8_t sync_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char buf[MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN];
    _mav_put_uint8_t(buf, 0, sync_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC, buf, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
#else
    mavlink_bs_slantrange_do_sync_t packet;
    packet.sync_type = sync_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC, (const char *)&packet, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
#endif
}

/**
 * @brief Send a bs_slantrange_do_sync message
 * @param chan MAVLink channel to send the message
 * @param struct The MAVLink struct to serialize
 */
static inline void mavlink_msg_bs_slantrange_do_sync_send_struct(mavlink_channel_t chan, const mavlink_bs_slantrange_do_sync_t* bs_slantrange_do_sync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    mavlink_msg_bs_slantrange_do_sync_send(chan, bs_slantrange_do_sync->sync_type);
#else
    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC, (const char *)bs_slantrange_do_sync, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
#endif
}

#if MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN <= MAVLINK_MAX_PAYLOAD_LEN
/*
  This varient of _send() can be used to save stack space by re-using
  memory from the receive buffer.  The caller provides a
  mavlink_message_t which is the size of a full mavlink message. This
  is usually the receive buffer for the channel, and allows a reply to an
  incoming message with minimum stack space usage.
 */
static inline void mavlink_msg_bs_slantrange_do_sync_send_buf(mavlink_message_t *msgbuf, mavlink_channel_t chan,  uint8_t sync_type)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    char *buf = (char *)msgbuf;
    _mav_put_uint8_t(buf, 0, sync_type);

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC, buf, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
#else
    mavlink_bs_slantrange_do_sync_t *packet = (mavlink_bs_slantrange_do_sync_t *)msgbuf;
    packet->sync_type = sync_type;

    _mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC, (const char *)packet, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_MIN_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_CRC);
#endif
}
#endif

#endif

// MESSAGE BS_SLANTRANGE_DO_SYNC UNPACKING


/**
 * @brief Get field sync_type from bs_slantrange_do_sync message
 *
 * @return Action to trigger start/stop sync, see BS_SLANTRANGE_SYNC_ACTION_TYPE
 */
static inline uint8_t mavlink_msg_bs_slantrange_do_sync_get_sync_type(const mavlink_message_t* msg)
{
    return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a bs_slantrange_do_sync message into a struct
 *
 * @param msg The message to decode
 * @param bs_slantrange_do_sync C-struct to decode the message contents into
 */
static inline void mavlink_msg_bs_slantrange_do_sync_decode(const mavlink_message_t* msg, mavlink_bs_slantrange_do_sync_t* bs_slantrange_do_sync)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
    bs_slantrange_do_sync->sync_type = mavlink_msg_bs_slantrange_do_sync_get_sync_type(msg);
#else
        uint8_t len = msg->len < MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN? msg->len : MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN;
        memset(bs_slantrange_do_sync, 0, MAVLINK_MSG_ID_BS_SLANTRANGE_DO_SYNC_LEN);
    memcpy(bs_slantrange_do_sync, _MAV_PAYLOAD(msg), len);
#endif
}

/** @file
 *	@brief MAVLink comm protocol testsuite generated from dronebox_messages.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#pragma once
#ifndef DRONEBOX_MESSAGES_TESTSUITE_H
#define DRONEBOX_MESSAGES_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL

static void mavlink_test_dronebox_messages(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{

	mavlink_test_dronebox_messages(system_id, component_id, last_msg);
}
#endif




static void mavlink_test_ds_actuator_do_action(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DS_ACTUATOR_DO_ACTION >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ds_actuator_do_action_t packet_in = {
		5
    };
	mavlink_ds_actuator_do_action_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.action_type = packet_in.action_type;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DS_ACTUATOR_DO_ACTION_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DS_ACTUATOR_DO_ACTION_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_actuator_do_action_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ds_actuator_do_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_actuator_do_action_pack(system_id, component_id, &msg , packet1.action_type );
	mavlink_msg_ds_actuator_do_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_actuator_do_action_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.action_type );
	mavlink_msg_ds_actuator_do_action_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ds_actuator_do_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_actuator_do_action_send(MAVLINK_COMM_1 , packet1.action_type );
	mavlink_msg_ds_actuator_do_action_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_ds_received_cmd_ack(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
	mavlink_status_t *status = mavlink_get_channel_status(MAVLINK_COMM_0);
        if ((status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) && MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK >= 256) {
        	return;
        }
#endif
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ds_received_cmd_ack_t packet_in = {
		5
    };
	mavlink_ds_received_cmd_ack_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        packet1.cmd_ackd = packet_in.cmd_ackd;
        
        
#ifdef MAVLINK_STATUS_FLAG_OUT_MAVLINK1
        if (status->flags & MAVLINK_STATUS_FLAG_OUT_MAVLINK1) {
           // cope with extensions
           memset(MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN + (char *)&packet1, 0, sizeof(packet1)-MAVLINK_MSG_ID_DS_RECEIVED_CMD_ACK_MIN_LEN);
        }
#endif
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_received_cmd_ack_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ds_received_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_received_cmd_ack_pack(system_id, component_id, &msg , packet1.cmd_ackd );
	mavlink_msg_ds_received_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_received_cmd_ack_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.cmd_ackd );
	mavlink_msg_ds_received_cmd_ack_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ds_received_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ds_received_cmd_ack_send(MAVLINK_COMM_1 , packet1.cmd_ackd );
	mavlink_msg_ds_received_cmd_ack_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_dronebox_messages(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_ds_actuator_do_action(system_id, component_id, last_msg);
	mavlink_test_ds_received_cmd_ack(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // DRONEBOX_MESSAGES_TESTSUITE_H

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <iostream>

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>

#include "MavlinkDef.h"

/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <time.h>
#include <arpa/inet.h>
#include <unistd.h>

#define BUFFER_LENGTH 2048

///
/// Sleeps for the number of milliseconds specified
/// (It can be interrupted by a signal or event)
///
void msleep (unsigned int ms) {
    int microsecs;
    struct timeval tv;
    microsecs = ms * 1000;
    tv.tv_sec  = microsecs / 1000000;
    tv.tv_usec = microsecs % 1000000;
    select (0, NULL, NULL, NULL, &tv);
}

/* Time since the beginning of time Jan 01, 1970
 * in computer terminology
 */
uint64_t microsSinceEpoch()
{

    struct timeval tv;

    uint64_t micros = 0;

    gettimeofday(&tv, NULL);
    micros =  ((uint64_t)tv.tv_sec) * 1000000 + tv.tv_usec;

    return micros;
}

/* Bind the socket to the port and make it non-blocking, i.e. do not wait for the device
 * or file to be ready or available. After the file is open, the read() and write()
 * calls always return immediately. If the process would be delayed in the read or write
 * operation, -1 is returned and errno is set instead of blocking the caller.
 */
bool bindSocket(__CONST_SOCKADDR_ARG __addr, int sock)
{
    // Bind the socket to port XXXXX - necessary to receive packets from mavproxy */
    if (-1 == bind(sock, __addr, sizeof(struct sockaddr)))
    {
        perror("error bind failed");
        close(sock);
        printf("Error binding to socket %d.. %s", sock, strerror(errno));
        return false;
    }

    // Attempt to make it non blocking
    if (fcntl(sock, F_SETFL, O_NONBLOCK | FASYNC) < 0)
    {
        fprintf(stderr, "error setting nonblocking: %s\n", strerror(errno));
        close(sock);
        printf("Error setting nonblocking: %s, Socket is %d", strerror(errno), sock);   // Exception on failure
        return false;
    }

    return true;
}

/// *********************************************************************************************
/// *********************************************************************************************
/// @brief This basic test is working. Uncomment below for a single threaded
/// operation and call this from the constructor.
void comm_udp()
{
    mavlink_message_t msg;

    std::cout << " Size is: " << sizeof(msg);

    int sock1 = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in locAddr, gcs_snd_Addr;

    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;
    int bytes_sent, i = 0;
    uint32_t len, temp = 0;
    float position[6] = {};

    printf("Socket 1: %d\n: ", (int)sock1);

    // Bind the socket to port 18540 - necessary to receive packets from mavproxy
    // 1. Create a channel for direct QGC communication.
    // The local address/port pair to receive packets from.
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("192.168.10.20");
    locAddr.sin_port = htons (14551);           // This can be any remote port. QGC will adjust its sending port.

    // Prepare the remote address/port pair to send packets to.
//    memset(&gcs_snd_Addr, 0, sizeof(gcs_snd_Addr));
//    gcs_snd_Addr.sin_family = AF_INET;
//    gcs_snd_Addr.sin_addr.s_addr = inet_addr("127.0.0.1");
//    gcs_snd_Addr.sin_port = htons (14550);
//    // Bind the socket to port 14551- necessary to receive packets from QGroundcontrol

    bindSocket((struct sockaddr *)&locAddr, sock1);

    for (;;)
    {
        /*Send Heartbeat */
        mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        bytes_sent = sendto(sock1, buf, len, 0, (struct sockaddr*)&gcs_snd_Addr, sizeof(struct sockaddr_in));

        /***********************************************************/

        /***********************************************************/

        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock1, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcs_snd_Addr, &fromlen);   // auto fills gcs_snd_Addr
        //printf("\nRecv from addr: %s, %d",inet_ntoa(gcs_snd_Addr.sin_addr), ntohs(gcs_snd_Addr.sin_port));

        if (recsize > 0)
        {
            // Something received - print out all bytes and parse packet
            mavlink_message_t msg;
            mavlink_status_t status;

            // printf("Bytes Received: %d\nDatagram: ", (int)recsize);
            for (i = 0; i < recsize; ++i)
            {
                temp = buf[i];
                // printf("%02x ", (unsigned char)temp);
                if (mavlink_parse_char(2, buf[i], &msg, &status))
                {
                    //std::cout << "\nmsg id: "<< msg.msgid << std::endl;

                    if (msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)
                    {
                        printf("Heartbeat received from QGC..\n");
                    }
//                    if (msg.msgid == MAVLINK_MSG_ID_DS_ACTUATOR_DO_ACTION)
//                    {
//                        mavlink_ds_actuator_do_action_t action;
//                        mavlink_msg_ds_actuator_do_action_decode(&msg, &action);
//                        std::cout << "\nmsg id: "<< msg.msgid << ",action: " << (DS_ACTUATOR_DO_ACTION_TYPE) action.action_type << std::endl;

//                        // Packet received
//                        printf("\nReceived packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d", msg.sysid, msg.compid, msg.len, msg.msgid);
//                        printf("\nStatus packet  : Buffer exceed: %d, Drops: %d, Parse err: %d, Success: %d\n", status.buffer_overrun, status.packet_rx_drop_count, status.parse_error, status.packet_rx_success_count);

//                        printf("\nRecv from addr: %s, %d",inet_ntoa(gcs_snd_Addr.sin_addr), ntohs(gcs_snd_Addr.sin_port));
//                    }
                }
            }
            printf("\n");
        }
        memset(buf, 0, BUFFER_LENGTH);
        msleep(100); // Sleep one second
    }
}

void comm_udp1()
{
    mavlink_message_t msg;
    std::cout << " Size is: " << sizeof(msg) << std::endl;
    std::cout << " Char bit is: " << CHAR_BIT << std::endl;
}

int main()
{
    std::cout << "Starting communication over udp..\n";
    comm_udp();
    sleep(2);
}


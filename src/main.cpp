#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <string.h>
#include <iostream>
#include <sys/types.h>
#include <pthread.h>

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>

#include "MavlinkDef.h"

/* Linux / MacOS POSIX timer headers */
#include <unistd.h>

#define BUFFER_LENGTH 2048

/// *********************************************************************************************
/// *********************************************************************************************
/// @brief This basic test is working. Uncomment below for a single threaded
/// operation and call this from the constructor.
void comm_udp()
{
    char target_ip[100];
    int sock1 = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    int sock2 = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);
    struct sockaddr_in px4_rx_Addr, px4_snd_Addr;
    struct sockaddr_in gcs_rx_Addr, gcs_snd_Addr;
    uint8_t buf[BUFFER_LENGTH];
    ssize_t recsize;
    socklen_t fromlen;
    int bytes_sent, i = 0;
    uint16_t len;
    unsigned int temp = 0;
    printf("Socket 1: %d\n: ", (int)sock1);
    printf("Socket 2: %d\n: ", (int)sock2);
    // Change the target ip if parameter was given
    strcpy(target_ip, "127.0.0.1");
    // 1. Create a channel for mavproxy communication.
    memset(&px4_rx_Addr, 0, sizeof(px4_rx_Addr));
    px4_rx_Addr.sin_family = AF_INET;
    px4_rx_Addr.sin_addr.s_addr = inet_addr("127.0.0.1"); //htonl(INADDR_LOOPBACK);
    px4_rx_Addr.sin_port = htons(18540);
    memset(&px4_snd_Addr, 0, sizeof(px4_snd_Addr));
    px4_snd_Addr.sin_family = AF_INET;
    px4_snd_Addr.sin_addr.s_addr = inet_addr("127.0.0.1");
    px4_snd_Addr.sin_port = htons(18540);        // For the client it is not necessary to set the port..nw stack/OS should set it?
    // Bind the socket to port 18540 - necessary to receive packets from mavproxy
    //bindSocket(sock1, (struct sockaddr *)&px4_rx_Addr);
    // 2. Create a channel for direct QGC communication.
    // The local address/port pair to receive packets from.
    memset(&gcs_rx_Addr, 0, sizeof(gcs_rx_Addr));
    gcs_rx_Addr.sin_family = AF_INET;
    gcs_rx_Addr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("192.168.10.20");
    gcs_rx_Addr.sin_port = htons (14552);           // This can be any remote port. QGC will adjust its sending port.
    // Prepare the remote address/port pair to send packets to.
    memset(&gcs_snd_Addr, 0, sizeof(gcs_snd_Addr));
    gcs_snd_Addr.sin_family = AF_INET;
    gcs_snd_Addr.sin_addr.s_addr = inet_addr("192.168.10.10");
    gcs_snd_Addr.sin_port = htons (14550);
    // Bind the socket to port 14551- necessary to receive packets from QGroundcontrol
    //bindSocket(sock2, (struct sockaddr *)&gcs_rx_Addr);
    for (;;)
    {
        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock1, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&px4_snd_Addr, &fromlen);
        if (recsize > 0)
        {
            // Something received - print out all bytes and parse packet
            mavlink_message_t msg;
            mavlink_status_t status;
            for (i = 0; i < recsize; ++i)
            {
                temp = buf[i];
                if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status))
                {
                    //if (msg.sysid != 1 && msg.compid != 1)
                    //    break;
                    memset(buf, 0, BUFFER_LENGTH);
                    printf("\nSOCK 1: Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = sendto(sock2, buf, len, 0, (struct sockaddr*)&gcs_snd_Addr, sizeof(struct sockaddr_in));
                }            }
            //printf("\n");
        }
        memset(buf, 0, BUFFER_LENGTH);
        ////////////////////////////////////////////////////////////
        memset(buf, 0, BUFFER_LENGTH);
        recsize = recvfrom(sock2, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcs_snd_Addr, &fromlen);
        if (recsize > 0)
        {
            // Something received - print out all bytes and parse packet
            mavlink_message_t msg;
            mavlink_status_t status;
            // printf("Bytes Received: %d\nDatagram: ", (int)recsize);
            for (i = 0; i < recsize; ++i)
            {
                if (mavlink_parse_char(MAVLINK_COMM_1, buf[i], &msg, &status))
                {
                    //if (msg.sysid != 255)
                   //     break;
                    memset(buf, 0, BUFFER_LENGTH);
                    printf("\nSOCK 2: Received packet: SYS: %d, COMP: %d, LEN: %d, MSG ID: %d\n", msg.sysid, msg.compid, msg.len, msg.msgid);
                    len = mavlink_msg_to_send_buffer(buf, &msg);
                    bytes_sent = sendto(sock1, buf, len, 0, (struct sockaddr*)&px4_snd_Addr, sizeof(struct sockaddr_in));
                }
            }
        }
        // memset(buf, 0, BUFFER_LENGTH); // Already done above
    }
}

int main()
{
    std::cout << "Starting communication over udp..\n";
    comm_udp();
}


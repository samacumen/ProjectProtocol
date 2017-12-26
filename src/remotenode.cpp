#include "remotenode.h"
#include <errno.h>
#include <iostream>

/* Linux / MacOS POSIX timer headers */
#include <sys/time.h>
#include <unistd.h>

/* Linux / MacOS POSIX timer headers */
#include "mavlinkdefinitions.h"

/****************** HELPER FUNCTIONS ******************/
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

/****************** COMMUNICATION FUNCTIONS ******************/

RemoteNode::RemoteNode()
{
    mavlink_message_t msg;
    std::cout << " Size is: " << sizeof(msg);
}

void RemoteNode::setup()
{
    sock1 = socket(PF_INET, SOCK_DGRAM, IPPROTO_UDP);

    printf("Socket 1: %d\n: ", (int)sock1);

    // Bind the socket to port 18540 - necessary to receive packets from mavproxy
    // 1. Create a channel for direct QGC communication.
    // The local address/port pair to receive packets from.
    memset(&locAddr, 0, sizeof(locAddr));
    locAddr.sin_family = AF_INET;
    locAddr.sin_addr.s_addr = htonl(INADDR_ANY); //inet_addr("192.168.10.20");
    locAddr.sin_port = htons (14551);           // This can be any remote port. QGC will adjust its sending port.

    // Bind the socket to port 14551- necessary to receive packets from QGroundcontrol
    bindSocket((struct sockaddr *)&locAddr, sock1);
}

/* Bind the socket to the port and make it non-blocking, i.e. do not wait for the device
 * or file to be ready or available. After the file is open, the read() and write()
 * calls always return immediately. If the process would be delayed in the read or write
 * operation, -1 is returned and errno is set instead of blocking the caller.
 */
bool RemoteNode::bindSocket(__CONST_SOCKADDR_ARG __addr, int sock)
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

int RemoteNode::send(MAVMSG msg)
{
    uint8_t buf[BUFFER_LENGTH];

    uint32_t len = mavlink_msg_to_send_buffer(buf, &msg);
    int bytes_sent = sendto(sock1, buf, len, 0, (struct sockaddr*)&gcs_snd_Addr, sizeof(struct sockaddr_in));
    return bytes_sent;
}

MAVMSG* RemoteNode::recv()
{
    uint8_t buf[BUFFER_LENGTH], temp;
    ssize_t recsize;
    socklen_t fromlen;
    MAVMSG* msg = new MAVMSG;

    memset(buf, 0, BUFFER_LENGTH);
    recsize = recvfrom(sock1, (void *)buf, BUFFER_LENGTH, 0, (struct sockaddr *)&gcs_snd_Addr, &fromlen);   // auto fills gcs_snd_Addr
    //printf("\nRecv from addr: %s, %d",inet_ntoa(gcs_snd_Addr.sin_addr), ntohs(gcs_snd_Addr.sin_port));

    if (recsize > 0)
    {
        // Something received - print out all bytes and parse packet
        mavlink_status_t status;

        // printf("Bytes Received: %d\nDatagram: ", (int)recsize);
        for (int i = 0; i < recsize; ++i)
        {
            temp = buf[i];
            // printf("%02x ", (unsigned char)temp);
            if (mavlink_parse_char(2, buf[i], msg, &status))
            {
                return msg;
            }
        }
        printf("\n");
    }
    memset(buf, 0, BUFFER_LENGTH); // clear the array
    msleep(100); // Sleep for 100ms
}

void RemoteNode::handlemessage(MAVMSG* msg)
{
    std::cout << "\nmsg id: "<< msg->msgid << std::endl;

    if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT)
    {
        printf("Heartbeat received from QGC..\n");
    }
}

MAVMSG RemoteNode::pack_hb()
{
    MAVMSG msg;
    /* Pack Heartbeat message */
    mavlink_msg_heartbeat_pack(1, 200, &msg, MAV_TYPE_HELICOPTER, MAV_AUTOPILOT_GENERIC, MAV_MODE_GUIDED_ARMED, 0, MAV_STATE_ACTIVE);
    return msg;
}

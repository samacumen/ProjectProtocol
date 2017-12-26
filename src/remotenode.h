/**********************************************************************************************************************
* @File: remotenode.h
*
* @Description : This header file contains all mavlink utility function of send and receive.
*
* @Date        : 26-08-2016
*
* @Author: Shyam Balasubramanian (info@shyamb.nl)
*
***********************************************************************************************************************/

#ifndef REMOTENODE_H
#define REMOTENODE_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>
#include "mavlinkdefinitions.h"

#define BUFFER_LENGTH 1024

class RemoteNode
{
public:
    RemoteNode();

    void setup();
    int send(MAVMSG msg);
    MAVMSG *recv();
    void start_comm();
    void handlemessage(MAVMSG *msg);
    MAVMSG pack_hb();

private:
    bool bindSocket(__CONST_SOCKADDR_ARG __addr, int sock);

    int sock1;
    struct sockaddr_in locAddr, gcs_snd_Addr;

};

#endif // REMOTENODE_H

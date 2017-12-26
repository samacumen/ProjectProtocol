#ifndef REMOTENODE_H
#define REMOTENODE_H

#include <sys/socket.h>
#include <netinet/in.h>
#include <fcntl.h>
#include <arpa/inet.h>

#define BUFFER_LENGTH 2048

class RemoteNode
{
public:
    RemoteNode();

    void setup();
    int send();
    void recv();
    void start_comm();

private:
    bool bindSocket(__CONST_SOCKADDR_ARG __addr, int sock);

    int sock1;
    struct sockaddr_in locAddr, gcs_snd_Addr;

};

#endif // REMOTENODE_H


#include <iostream>
#include "remotenode.h"

void start_comm();

int main()
{
//    mavlink_message_t msg;
//    std::cout << " Size is: " << sizeof(msg) << std::endl;
//    std::cout << " Char bit is: " << CHAR_BIT << std::endl;

    std::cout << "Starting communication over udp..\n";
    start_comm();
}

///
/// Communicate with remote node via mavlink. This function does two things:
/// 1) Sends a simple heartbeat to the remote node
/// 2) Listens to incoming heartbeat from remote node
///
void start_comm()
{
    RemoteNode node;

    // Setup the communication channel between sender and receiver
    node.setup();

    for (; ;)
    {
        // Pack a message and send to destination
        MAVMSG msg_to_send = node.pack_hb();
        node.send(msg_to_send);

        // Listen to any pending (incoming) message in OS buffer
        MAVMSG* msg = node.recv();
        if (msg != nullptr)
        {
            node.handlemessage(msg);
        }
    }
}


#include <iostream>
#include "remotenode.h"

int main()
{
    // mavlink_message_t msg;
    //std::cout << " Size is: " << sizeof(msg) << std::endl;
    //std::cout << " Char bit is: " << CHAR_BIT << std::endl;

    std::cout << "Starting communication over udp..\n";
    RemoteNode node;
    node.start_comm();
}


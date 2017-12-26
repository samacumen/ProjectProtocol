/**********************************************************************************************************************
* @File: MavlinkDef.h
*
* @Description : This header file contains all mavlink based definitions used in the project.
*
* @Date        : 26-08-2016
*
* @Author: Shyam Balasubramanian (info@shyamb.nl)
*
***********************************************************************************************************************/

#ifndef MAVLINKDEF_H
#define MAVLINKDEF_H
#include <limits.h>

#include "/home/sam/mav_proj/project_protocol/lib/mavlink_C_v2.0/mavlink_types.h"
#include "/home/sam/mav_proj/project_protocol/lib/mavlink_C_v2.0/ardupilotmega/mavlink.h"

// Version v1.0 has a mavlink_message size of 272 bytes
// Version v2.0 has a mavlink_message size of 291 bytes

typedef mavlink_message_t MAVMSG;

#endif // MAVLINKDEF_H

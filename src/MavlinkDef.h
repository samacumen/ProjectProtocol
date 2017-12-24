/**********************************************************************************************************************
* @File: MavlinkDef.h
*
* @Description : This header file contains all mavlink based definitions used in the project.
*
* @Date        : 26-10-2013
*
* @Author: Shyam Balasubramanian (info@shyamb.nl)
*
***********************************************************************************************************************/

#ifndef MAVLINKDEF_H
#define MAVLINKDEF_H
#include <limits.h>

//#include "../lib/mavlink_0.9_C_v1.0/mavlink_types.h"              // v0.9_v1.0 protocol: mavlink_msg size: 272
//#include "../lib/mavlink_0.9_C_v1.0/ardupilotmega/mavlink.h"      //

//#include "../lib/mavlink_0.9_C_v2.0/mavlink_types.h"              // v0.9_v2.0 protocol: mavlink_msg size: 291
//#include "../lib/mavlink_0.9_C_v2.0/ardupilotmega/mavlink.h"      //

//#include "../lib/mavlink_1.0_C_v1.0/mavlink_types.h"              // v1.0_v1.0 protocol: mavlink_msg size: 272
//#include "../lib/mavlink_1.0_C_v1.0/ardupilotmega/mavlink.h"      //

//#include "../lib/mavlink_1.0_C_v2.0/mavlink_types.h"                // v1.0_v2.0 protocol: mavlink_msg size: 291
//#include "../lib/mavlink_1.0_C_v2.0/ardupilotmega/mavlink.h"        //

#include "/home/sam/github/qgroundcontrol/libs/mavlink/include/mavlink/v2.0/mavlink_types.h"
#include "/home/sam/github/qgroundcontrol/libs/mavlink/include/mavlink/v2.0/ardupilotmega/mavlink.h"

// v0.9_v1.0 protocol: mavlink_msg size: 272
// v1.0_v1.0 protocol: mavlink_msg size: 272

// v0.9_v2.0 protocol: mavlink_msg size: 291
// v1.0_v2.0 protocol: mavlink_msg size: 291

// Version v2.0 has a mavlink_message size of 291 bytes

typedef mavlink_message_t MAVMSG;

#endif // MAVLINKDEF_H

/*
 * Control.h
 *
 *  Created on: Nov 4, 2016
 *      Author: DoThanhTuan
 */

#ifndef CONTROL_H_
#define CONTROL_H_

#include "os.h"
#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include "Common.h"

class Control {
public:
    Control();
    virtual ~Control();

    int mTargetSystemID;
    int mTargetComponentID;

    int mSystemID;
    int mComponentID;
    
    uint8_t mCoordinateFrame;

    mavlink_set_position_target_local_ned_t mPosition;

    mavlink_message_t getArmCommand(int sysID, int compID);
    mavlink_message_t getTakeoffCommand(int sysID, int compID);
    mavlink_message_t getLandCommand(int sysID, int compID);
    mavlink_message_t getChangeToGuidedModeCommand(int sysID, int compID);
    mavlink_message_t getChangeToBrakeModeCommand(int sysID, int compID);
    //mavlink_message_t toggle_offboard_control( bool flag );

    //Navigation control
    mavlink_message_t setCurrentPosition(mavlink_set_position_target_local_ned_t ip);
    void setCoordinateFrame(uint8_t framType);
    void setPosition(float x, float y, float z);
    void setVelocity(float vx, float vy, float vz);
    void setYaw(float yaw);
    void setYawRate(float yaw_rate);
    mavlink_message_t setConditionYaw(float targetAngle, bool isClockwise);

    mavlink_message_t getNavConrolMessage(NAV_CMD_TYPE navCmd);
    uint64_t get_time_usec();
    
};

#endif /* CONTROL_H_ */

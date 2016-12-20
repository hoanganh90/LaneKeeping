/*
 * Control.cpp
 *
 *  Created on: Nov 4, 2016
 *      Author: DoThanhTuan
 */

#include "Control.h"

Control::Control() {
    // TODO Auto-generated constructor stub
    mTargetSystemID = 1;
    mTargetComponentID = 1;
    mSystemID = 250;
    mComponentID = 190;
    mCoordinateFrame = MAV_FRAME_BODY_OFFSET_NED;
    //cong.anh
    velocityX = 0;
    velocityY = 0;
}

Control::~Control() {
    // TODO Auto-generated destructor stub
}

mavlink_message_t Control::getArmCommand(int sysID, int compID)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_command_long_t msg;

    msg.target_system = mTargetSystemID;
    msg.target_component = mTargetComponentID;
    msg.command = MAV_CMD_COMPONENT_ARM_DISARM;
    msg.param1 = 1;
    msg.param2 = 0;
    msg.param3 = 0;
    msg.param4 = 0;
    msg.param5 = 0;
    msg.param6 = 0;
    msg.param7 = 0;
    msg.confirmation = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}

mavlink_message_t Control::getTakeoffCommand(int sysID, int compID)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_command_long_t msg;

    msg.target_system = mTargetSystemID;
    msg.target_component = mTargetComponentID;
    msg.command = MAV_CMD_NAV_TAKEOFF;
    msg.param1 = 0;
    msg.param2 = 0;
    msg.param3 = 0;
    msg.param4 = 0;
    msg.param5 = 0;
    msg.param6 = 0;
    msg.param7 = 2;
    msg.confirmation = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}
mavlink_message_t Control::getLandCommand(int sysID, int compID)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_command_long_t msg;

    msg.target_system = mTargetSystemID;
    msg.target_component = mTargetComponentID;
    msg.command = MAV_CMD_NAV_LAND;
    msg.param1 = 0;
    msg.param2 = 0;
    msg.param3 = 0;
    msg.param4 = 0;
    msg.param5 = 0;
    msg.param6 = 0;
    msg.param7 = 0;
    msg.confirmation = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}

mavlink_message_t Control::getChangeToGuidedModeCommand(int sysID, int compID)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_set_mode_t msg;
    msg.target_system = mTargetSystemID;
    msg.base_mode = 1;
    msg.custom_mode = 4;

    mavlink_message_t message;
    mavlink_msg_set_mode_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}

mavlink_message_t Control::getChangeToBrakeModeCommand(int sysID, int compID)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_set_mode_t msg;
    msg.target_system = mTargetSystemID;
    msg.base_mode = 1;
    msg.custom_mode = 17;

    mavlink_message_t message;
    mavlink_msg_set_mode_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}

//mavlink_message_t Control::toggle_offboard_control( bool flag )
//{
//    // Prepare command for off-board mode
//    mavlink_command_long_t com = { 0 };
//    com.target_system    = mTargetSystemID;
//    com.target_component = mTargetComponentID;
//    com.command          = MAV_CMD_NAV_GUIDED_ENABLE;
//    com.confirmation     = true;
//    com.param1           = (float) flag; // flag >0.5 => start, <0.5 => stop
//
//    // Encode
//    mavlink_message_t message;
//    mavlink_msg_command_long_encode(mSystemID, mComponentID, &message, &com);
//
//    return message;
//}

mavlink_message_t Control::setCurrentPosition(mavlink_set_position_target_local_ned_t ip)
{
    //mPosition = ip;
}

void Control::setCoordinateFrame(uint8_t frameType)
{
    mCoordinateFrame = frameType;
}

void Control::setPosition(float x, float y, float z)
{
        // double check some system parameters
    mPosition.time_boot_ms = 0;
    mPosition.target_system    = mTargetSystemID;
    mPosition.target_component = mTargetComponentID;
    
    mPosition.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION;

    mPosition.coordinate_frame = mCoordinateFrame;

    mPosition.x   = x;
    mPosition.y   = y;
    mPosition.z   = z;
    mPosition.vx = 0;
    mPosition.vy = 0;
    mPosition.vz = 0;
    mPosition.afx = 0;
    mPosition.afy = 0;
    mPosition.afz = 0;
    mPosition.yaw = 0;
    mPosition.yaw_rate = 0;
}

void Control::setVelocity(float vx, float vy, float vz)
{
    mPosition.time_boot_ms = 0;
    mPosition.target_system    = mTargetSystemID;
    mPosition.target_component = mTargetComponentID;
    
    mPosition.type_mask = MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY;

    mPosition.coordinate_frame = mCoordinateFrame;

    mPosition.vx  = vx;
    mPosition.vy  = vy;
    mPosition.vz  = vz;
    mPosition.x   = 0;
    mPosition.y   = 0;
    mPosition.z   = 0;
    mPosition.afx = 0;
    mPosition.afy = 0;
    mPosition.afz = 0;
    mPosition.yaw = 0;
    mPosition.yaw_rate = 0;
}
void Control::setYaw(float yaw)
{
    mPosition.time_boot_ms = 0;
    mPosition.target_system    = mTargetSystemID;
    mPosition.target_component = mTargetComponentID;
    
    mPosition.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE ;
    mPosition.coordinate_frame = mCoordinateFrame;
    mPosition.yaw  = yaw;
}

void Control::setYawRate(float yaw_rate)
{
    mPosition.time_boot_ms = 0;
    mPosition.target_system    = mTargetSystemID;
    mPosition.target_component = mTargetComponentID;
    
    mPosition.type_mask &= MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE ;
    mPosition.coordinate_frame = mCoordinateFrame;
    mPosition.yaw_rate  = yaw_rate;
}

mavlink_message_t Control::setConditionYaw(float targetAngle, bool isClockwise)
{
    // --------------------------------------------------------------------------
    //   PACK PAYLOAD
    // --------------------------------------------------------------------------
    mavlink_command_long_t msg;

    msg.target_system = mTargetSystemID;
    msg.target_component = mTargetComponentID;
    msg.command = MAV_CMD_CONDITION_YAW;
    msg.param1 = targetAngle;
    msg.param2 = 0;
    msg.param3 = isClockwise ? 1 : -1;
    msg.param4 = 1; // Relative to the current yaw direction
    msg.param5 = 0;
    msg.param6 = 0;
    msg.param7 = 0;
    msg.confirmation = 0;

    mavlink_message_t message;
    mavlink_msg_command_long_encode(mSystemID, mComponentID, &message, &msg);

    return message;
}

mavlink_message_t Control::getNavConrolMessage(NAV_CMD_TYPE navCmd)
{
    // --------------------------------------------------------------------------
    //   ENCODE
    // --------------------------------------------------------------------------
    switch(navCmd)
    {
        case Forward:
            setVelocity(0.5, 0, 0);
            break;
        case Backward:
            setVelocity(-0.5, 0, 0);
            break;
        case Left:
            setVelocity(0, -0.5, 0);
            break;
        case Right:
            setVelocity(0, 0.5, 0);
            break;
        case Up:
            break;
        case Down:
            break;
        default:
            break;
    }
    
    mavlink_message_t message;
    mavlink_msg_set_position_target_local_ned_encode(mSystemID, mComponentID, &message, &mPosition);
    
    return message;
}

uint64_t Control::get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

#ifndef COMMON_H_
#define COMMON_H_

extern "C" {
#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

#include "libavcodec/avcodec.h"
#include "libavformat/avformat.h"
}
#include "SDL/SDL.h"
#include "SDL/SDL_image.h"
#include <string>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio.hpp"
//#include "LineSegment.h"
#include "LaneInfo.h"
#ifndef EXTERN
#define EXTERN extern
#endif

#include "mavlink/include/mavlink/v1.0/common/mavlink.h"

#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_POSITION     0b0000111111111000
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_VELOCITY     0b0000111111000111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_ACCELERATION 0b0000110000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_FORCE        0b0000111000111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_ANGLE    0b0000100111111111
#define MAVLINK_MSG_SET_POSITION_TARGET_LOCAL_NED_YAW_RATE     0b0000010111111111

// ------------------------------------------------------------------------------
//   Data Structures
// ------------------------------------------------------------------------------

typedef enum {
    Arm = 0,
    Takeoff, //1
    Land, //2
    Stop,//3
    Auto,//4
    Manual,//5
    Backward,//6
    Forward,//7
    Left,//8
    Right,//9
    CounterClockwise,//10
    Clockwise,//11
    Up,//12
    Down,//13
    ForceStop, //14
    ForceClockwise,
    ForceCounterClockwise,
    NONE//15
} NAV_CMD_TYPE;

//cong.anh
typedef enum {
    Center_lane = 0,
    Left_Lane,
    Right_Lane
}POSITION;

typedef enum
{
    LANDED,
    TAKINGOFF,
    HOVERING,
    FLYING,
    LANDING,
    EMERGENCY,
    MAX
} DRONE_STATE;

struct Time_Stamps
{
    Time_Stamps()
    {
        reset_timestamps();
    }

    uint64_t heartbeat;
    uint64_t sys_status;
    uint64_t battery_status;
    uint64_t radio_status;
    uint64_t local_position_ned;
    uint64_t global_position_int;
    uint64_t position_target_local_ned;
    uint64_t position_target_global_int;
    uint64_t highres_imu;
    uint64_t attitude;

    void
    reset_timestamps()
    {
        heartbeat = 0;
        sys_status = 0;
        battery_status = 0;
        radio_status = 0;
        local_position_ned = 0;
        global_position_int = 0;
        position_target_local_ned = 0;
        position_target_global_int = 0;
        highres_imu = 0;
        attitude = 0;
    }

};


// Struct containing information on the MAV we are currently connected to

struct Mavlink_Messages {

    int sysid;
    int compid;

    // Heartbeat
    mavlink_heartbeat_t heartbeat;

    // System Status
    mavlink_sys_status_t sys_status;

    // Battery Status
    mavlink_battery_status_t battery_status;

    // Radio Status
    mavlink_radio_status_t radio_status;

    // Local Position
    mavlink_local_position_ned_t local_position_ned;

    // Global Position
    mavlink_global_position_int_t global_position_int;

    // Local Position Target
    mavlink_position_target_local_ned_t position_target_local_ned;

    // Global Position Target
    mavlink_position_target_global_int_t position_target_global_int;

    // HiRes IMU
    mavlink_highres_imu_t highres_imu;

    // Attitude
    mavlink_attitude_t attitude;

    // Status text
    mavlink_statustext_t status_text;

    // Params
    mavlink_param_value_t param_value;
    
    // Heading
    mavlink_vfr_hud_t vfr_hud;
    // System Parameters?

    // Time Stamps
    Time_Stamps time_stamps;

    void
    reset_timestamps()
    {
        time_stamps.reset_timestamps();
    }

};

//Global variable for sending command
EXTERN volatile NAV_CMD_TYPE mNavigationCommand;
//cong.anh
EXTERN volatile double angle;
EXTERN volatile float velocityX;
EXTERN volatile float velocityY;
//EXTERN volatile double velocity;
EXTERN volatile bool bIsDroneStopped;
EXTERN volatile double timeStep;// Save time value of each ste
EXTERN POSITION position;
EXTERN LaneInfo primaryLane;
EXTERN timeval timeStepBegin; // Calculate time step for Canndy detection, Hough line, Vanishing , Tracking, SendCMD2Control
EXTERN FILE *visionProcessFile;
EXTERN FILE *visionPositionFile;
EXTERN std::vector<StoredLine> sLines;
EXTERN cv::Point2f C_0;
EXTERN cv::Point2f C_1;
EXTERN cv::Point2f C_2;
EXTERN volatile bool isAutoMode;
EXTERN timeval visionBegin;
#endif /* COMMON_H_ */
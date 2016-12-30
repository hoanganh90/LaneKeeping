/*
 * UDPConnection.cpp
 *
 *  Created on: Nov 2, 2016
 *      Author: DoThanhTuan
 */

#include "UDP_Connection.h"

const char* SAVE_DATA_PATH = "SaveData/Speed";
//const char* SPEED_FILE_NAME = "SaveData/speed.txt";

UDP_Connection::UDP_Connection(Control control) {
    // TODO Auto-generated constructor stub
    mControl = control;
    bIsHasInitialPosition = false;
    bShouldExit = false;
}

UDP_Connection::~UDP_Connection() {
    // TODO Auto-generated destructor stub
}

void* runningConnectionThread(void *args)
{
    UDP_Connection* udpConnection = (UDP_Connection*) args;
    udpConnection->run();
    return NULL;
}

void UDP_Connection::start() {

    printf("START READ THREAD \n");

    int result = pthread_create( &mCommunicationThread, NULL, &runningConnectionThread, this );
    if ( result ) throw result;
}

int UDP_Connection::setupM2CSock(int port)
{
    printf("Try to connect\n");

    struct sockaddr_in my_addr;

    int sock = Socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
    memset(&my_addr, 0, sizeof(my_addr));
    my_addr.sin_family = AF_INET;
    my_addr.sin_port = htons(port);
    my_addr.sin_addr.s_addr = htonl(INADDR_ANY);

    Bind(sock, (struct sockaddr *)&my_addr, sizeof(my_addr));

    printf("Connection result: %d\n", sock);

    //Waiting for incoming message to determine mav proxy port
    sockaddr_in client;
    int len = sizeof(client);
    char buf[264];
    recvfrom(sock, buf, sizeof(buf), 0, (struct sockaddr *)&client, (socklen_t *)&len);
    serverPort = ntohs(client.sin_port);

    printf("MAVProxy port: %d\n", serverPort);

    return sock;
}

int UDP_Connection::setupC2MSock(char *desip, int port)
{
    struct sockaddr_in mavProxy_addr;

   int sock = Socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
   memset(&mavProxy_addr, 0, sizeof(mavProxy_addr));
   mavProxy_addr.sin_family = AF_INET;
   mavProxy_addr.sin_port = htons(port);

   if(inet_aton(desip, &mavProxy_addr.sin_addr) == 0) {
       std::cerr << "mavProxy ip addr translation error!" << std::endl;
       close(sock);
       return -1;
   }

   Connect(sock, (struct sockaddr *)&mavProxy_addr, sizeof(mavProxy_addr));

   return sock;
}

double UDP_Connection::getPeriodOfTimeInSecondSinceStartTime(timeval startTime)
{
    timeval currentTime;
    gettimeofday(&currentTime, 0);
    return currentTime.tv_sec + 0.000001*currentTime.tv_usec - startTime.tv_sec - 0.000001*startTime.tv_usec;
}

void UDP_Connection::writeLog(NAV_CMD_TYPE navCommand, double time)
{
    switch(navCommand)
    {
        case Forward:
            fprintf(fileSpeed, "%.6f    0.30    0.00\n", time);
            fflush(fileSpeed);
            break;
        case Backward:
            fprintf(fileSpeed, "%.6f    -0.3    0.00\n", time);
            fflush(fileSpeed);
            break;
        case Left:
            fprintf(fileSpeed, "%.6f    0.00    -0.3\n", time);
            fflush(fileSpeed);
            break;
        case Right:
            fprintf(fileSpeed, "%.6f    0.00    0.30\n", time);
            fflush(fileSpeed);
            break;
        default:
            break;
    }
}

void UDP_Connection::run()
{

    int nSelected;
    struct timeval timeout;
    fd_set readfds;
    int max_fd;
    int ntimeout;

    timeval startTime;
    gettimeofday(&startTime, 0);
    
    //Create Vision Folders
    struct stat st = {0};
    if (stat(SAVE_DATA_PATH, &st) == -1)
    {
       mkdir(SAVE_DATA_PATH, 0700);
    }
    else
    {
         system("exec rm -r SaveData/Speed/*");
    }
    
    fileSpeed = fopen("SaveData/Speed/speed.txt", "w");
    if(fileSpeed == NULL)
    {
        std::cout << "Can not open Speed file" << std::endl;
        return;
    }
    else 
    {
        fprintf(fileSpeed, "  Time   VelocityX   VelocityY\n");
    }
    NAV_CMD_TYPE mLastControlCommand = NONE;
    bool bIsControlCmd = false;
    bool bIsGuidedMode = false;
    
    m2cSock = setupM2CSock(14551);
    c2mSock = setupC2MSock("127.0.0.1", serverPort);
    printf("C2MSock connection result: %d\n", c2mSock);
    
    int fd_stdin = fileno(stdin);

    FD_ZERO(&readfds);
    FD_SET(m2cSock, &readfds);
    FD_SET(fd_stdin, &readfds);
    max_fd = (fd_stdin > m2cSock)?fd_stdin: m2cSock;

    while(!bShouldExit)
    {
        timeout.tv_sec = 0;
        timeout.tv_usec = 25000; // 25ms

        nSelected =  Select(max_fd +1, &readfds, (fd_set *)NULL, (fd_set *)NULL, &timeout);

        if(nSelected > 0)
        {   //printf("nSelected > 0\n");
            if (FD_ISSET(m2cSock, &readfds))
            {
                //printf("Receive message\n");
                // 1. handle RX data
                handle_message(m2cSock);

                //3. Setup next select
                FD_ZERO(&readfds);
                FD_SET(m2cSock, &readfds);
                FD_SET(fd_stdin, &readfds);
                max_fd = (fd_stdin > m2cSock)?fd_stdin: m2cSock;
            }

 //           if(bIsHasInitialPosition)
 //           {
                //Send command when it has current position
                if(mNavigationCommand != NONE)
                {
                    bIsControlCmd = false;
                    switch(mNavigationCommand)
                    {
                        case Arm:
                            sendChangeToGuidedModeCommand();
                            bIsGuidedMode = true;
                            fprintf(fileSpeed, "Guided Mode\n");
                            sleep(1);
                            sendArmCommand();
                            break;
                        case Stop:
                            sendChangeToBrakeModeCommand();
                            fprintf(fileSpeed, "Brake Mode\n");
                            bIsGuidedMode = false;
                            break;
                        case Takeoff:
                            sendTakeoffCommand();
                            break;
                        case Land:
                            sendLandCommand();
                            fprintf(fileSpeed, "Land Mode\n");
                            bIsGuidedMode = false;
                            break;
//                        case Clockwise:
//                            sendControlCommand(mNavigationCommand);
//                            break;
//                        case CounterClockwise:
//                            sendControlCommand(mNavigationCommand);
//                            break;
                        default:
                            if (bIsGuidedMode == false) {
                                sendChangeToGuidedModeCommand();
                                fprintf(fileSpeed, "Guided Mode\n");
                                sleep(1);
                                bIsGuidedMode = true;
                            }
                            
                            double period = getPeriodOfTimeInSecondSinceStartTime(startTime); //Send control command every 1 second
                            if (mLastControlCommand != NONE)
                            {
                                if (mLastControlCommand != mNavigationCommand)
                                {                   
                                    sendControlCommand(mNavigationCommand);
                                    gettimeofday(&startTime, 0);
                                }
                                else 
                                {
                                    if( period > 0.2)//Send command to Drone 5Hz
                                    {
                                        sendControlCommand(mNavigationCommand);
                                        gettimeofday(&startTime, 0);
                                    }
                                }
                            }
                            else
                            {
                                sendControlCommand(mNavigationCommand);  
                                gettimeofday(&startTime, 0);
                            }
                            writeLog(mNavigationCommand, period);
                            mLastControlCommand = mNavigationCommand;
                            bIsControlCmd = true;
                            break; 
                    }
                    if (!bIsControlCmd) 
                    {
                        mNavigationCommand = NONE;
                    }
                }
//            }
        }
        else
            if(nSelected == 0){ // no data arrival, time outed
                if(ntimeout++% 100 == 99){
                    std::cout << "timeout : " << ntimeout<< std::endl;
                }

                FD_ZERO(&readfds);
                FD_SET(fd_stdin, &readfds);
                FD_SET(m2cSock, &readfds);

            }else if(nSelected < 0){
                std::cerr << "Select call error"  << std::endl;
            }

    }
}

int UDP_Connection::readData(int fd, void *usrbuf, size_t n)
{
    int nleft = n;
    int nread;
    unsigned char *bufp = (unsigned char *)usrbuf;

    int len = 0;

    while (nleft > 0) {
        if ((nread = read(fd, bufp, nleft)) < 0) {
            if (errno == EINTR) /* interrupted by sig handler return */
                nread = 0;      /* and call read() again */
            else
                return -1;      /* errno set by read() */
        }
        else if (nread == 0)
            break;              /* EOF */

        nleft -= nread;
        bufp += nread;
    }

    //Get length of packet
    len = *(bufp + 1) + 8; //Payload length + 8 bytes

    return len;         /* return >= 0 */
}

unsigned char rxBuff[264];

void UDP_Connection::handle_message(int sock)
{
    int len = readData(sock, rxBuff, 264);

    if(len <= 0)
    {
        printf("Can not read data");
        return;
    }

    decode_message(rxBuff);
}

uint64_t UDP_Connection::get_time_usec()
{
	static struct timeval _time_stamp;
	gettimeofday(&_time_stamp, NULL);
	return _time_stamp.tv_sec*1000000 + _time_stamp.tv_usec;
}

void UDP_Connection::decode_message(unsigned char *rxBuff)
{
    Time_Stamps this_timestamps;

    mavlink_message_t *message = (mavlink_message_t*) rxBuff;

    current_messages.sysid  = message->sysid;
    current_messages.compid = message->compid;
    system_id = message->sysid;
    autopilot_id = message->compid;

    // Handle Message ID
    switch (message->msgid)
    {

        case MAVLINK_MSG_ID_HEARTBEAT:
        {
            //printf("MAVLINK_MSG_ID_HEARTBEAT\n");
            mavlink_msg_heartbeat_decode(message, &(current_messages.heartbeat));
            current_messages.time_stamps.heartbeat = get_time_usec();
            this_timestamps.heartbeat = current_messages.time_stamps.heartbeat;
            break;
        }

        case MAVLINK_MSG_ID_SYS_STATUS:
        {
            printf("MAVLINK_MSG_ID_SYS_STATUS\n");
            mavlink_msg_sys_status_decode(message, &(current_messages.sys_status));
            current_messages.time_stamps.sys_status = get_time_usec();
            this_timestamps.sys_status = current_messages.time_stamps.sys_status;
            break;
        }

        case MAVLINK_MSG_ID_BATTERY_STATUS:
        {
            //printf("MAVLINK_MSG_ID_BATTERY_STATUS\n");
            mavlink_msg_battery_status_decode(message, &(current_messages.battery_status));
            current_messages.time_stamps.battery_status = get_time_usec();
            this_timestamps.battery_status = current_messages.time_stamps.battery_status;
            break;
        }

        case MAVLINK_MSG_ID_RADIO_STATUS:
        {
            //printf("MAVLINK_MSG_ID_RADIO_STATUS\n");
            mavlink_msg_radio_status_decode(message, &(current_messages.radio_status));
            current_messages.time_stamps.radio_status = get_time_usec();
            this_timestamps.radio_status = current_messages.time_stamps.radio_status;
            break;
        }

        case MAVLINK_MSG_ID_LOCAL_POSITION_NED:
        {
            printf("MAVLINK_MSG_ID_LOCAL_POSITION_NED\n");
            mavlink_msg_local_position_ned_decode(message, &(current_messages.local_position_ned));
            current_messages.time_stamps.local_position_ned = get_time_usec();
            this_timestamps.local_position_ned = current_messages.time_stamps.local_position_ned;

            //[Do.Tuan] Init position
            bIsHasInitialPosition = true;

            current_position.x        = current_messages.local_position_ned.x;
            current_position.y        = current_messages.local_position_ned.y;
            current_position.z        = current_messages.local_position_ned.z;
            current_position.vx       = current_messages.local_position_ned.vx;
            current_position.vy       = current_messages.local_position_ned.vy;
            current_position.vz       = current_messages.local_position_ned.vz;
            current_position.yaw      = current_messages.attitude.yaw;
            current_position.yaw_rate = current_messages.attitude.yawspeed;
            mControl.setCurrentPosition(current_position);

            break;
        }

        case MAVLINK_MSG_ID_GLOBAL_POSITION_INT:
        {
            printf("MAVLINK_MSG_ID_GLOBAL_POSITION_INT\n");
            mavlink_msg_global_position_int_decode(message, &(current_messages.global_position_int));
            current_messages.time_stamps.global_position_int = get_time_usec();
            this_timestamps.global_position_int = current_messages.time_stamps.global_position_int;
            
            velocityX = current_messages.global_position_int.vx;
            velocityY = current_messages.global_position_int.vy;
            printf("Velocity X = %f | Y= %f\n", velocityX, velocityY);
            
            if (velocityX > 2 && velocityY > 2)
            {
                bIsDroneStopped = false;
            }
            else 
            {
                double velocity = sqrt(pow(current_messages.global_position_int.vx, 2) + pow(current_messages.global_position_int.vy, 2));
                if (velocity < 0.25) {
                    bIsDroneStopped = true;
                } else {
                    bIsDroneStopped = false;
                }
            }
            
            break;
        }

        case MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED:
        {
            printf("MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED\n");
            mavlink_msg_position_target_local_ned_decode(message, &(current_messages.position_target_local_ned));
            current_messages.time_stamps.position_target_local_ned = get_time_usec();
            this_timestamps.position_target_local_ned = current_messages.time_stamps.position_target_local_ned;
            
            //[Do.Tuan] Init position
            bIsHasInitialPosition = true;
            
            break;
        }
        
        case MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT:
        {
            printf("MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT\n");
            mavlink_msg_position_target_global_int_decode(message, &(current_messages.position_target_global_int));
            current_messages.time_stamps.position_target_global_int = get_time_usec();
            this_timestamps.position_target_global_int = current_messages.time_stamps.position_target_global_int;
            
            break;
        }

        case MAVLINK_MSG_ID_HIGHRES_IMU:
        {
            //printf("MAVLINK_MSG_ID_HIGHRES_IMU\n");
            mavlink_msg_highres_imu_decode(message, &(current_messages.highres_imu));
            current_messages.time_stamps.highres_imu = get_time_usec();
            this_timestamps.highres_imu = current_messages.time_stamps.highres_imu;
            break;
        }

        case MAVLINK_MSG_ID_ATTITUDE:
        {
            //printf("MAVLINK_MSG_ID_ATTITUDE\n");
            mavlink_msg_attitude_decode(message, &(current_messages.attitude));
            current_messages.time_stamps.attitude = get_time_usec();
            this_timestamps.attitude = current_messages.time_stamps.attitude;
            break;
        }

        case MAVLINK_MSG_ID_STATUSTEXT:
        {
            //printf("MAVLINK_MSG_ID_STATUSTEXT:");
            mavlink_msg_statustext_decode(message, &(current_messages.status_text));
            printf(current_messages.status_text.text);
            printf("\n");
            break;
        }

        case MAVLINK_MSG_ID_PARAM_VALUE:
        {
            //printf("MAVLINK_MSG_ID_PARAM_VALUE:");
            mavlink_msg_param_value_decode(message, &(current_messages.param_value));
            //printf("RECEIVED PARAM = [ %s %.4f , %.4f , %.4f ] \n", current_messages.param_value.param_id, current_messages.param_value.param_value, current_messages.param_value.param_type, current_messages.param_value.param_count, current_messages.param_value.param_index);
            break;
        }

        case MAVLINK_MSG_ID_VFR_HUD:
        {
            
            mavlink_msg_vfr_hud_decode(message, &(current_messages.vfr_hud));
            mCurrentHeading = current_messages.vfr_hud.heading;
            printf("MAVLINK_MSG_ID_VFR_HUD: %f",mCurrentHeading);
            break;
        }
        
        default:
        {
            //printf("Warning, did not handle message id %i\n",message->msgid);
            break;
        }
    }
}

void UDP_Connection::sendArmCommand()
{
    mavlink_message_t msg = mControl.getArmCommand(system_id,autopilot_id);
    send_message(msg);
}

void UDP_Connection::sendTakeoffCommand()
{
    mavlink_message_t msg = mControl.getTakeoffCommand(system_id,autopilot_id);
    send_message(msg);
}

void UDP_Connection::sendLandCommand()
{
    mavlink_message_t msg = mControl.getLandCommand(system_id,autopilot_id);
    send_message(msg);
}

void UDP_Connection::sendControlCommand(NAV_CMD_TYPE navCommand)
{
    if (navCommand == Clockwise || navCommand == CounterClockwise)
    {
        if (navCommand == Clockwise)
        {
            mavlink_message_t msg = mControl.setConditionYaw(10, true); //mavlink_command_long_t
            send_message(msg);
        }
        else
        {
            mavlink_message_t msg = mControl.setConditionYaw(10, false); //mavlink_command_long_t
            send_message(msg);
        }
    }
    else
    {
        mavlink_message_t msg = mControl.getNavConrolMessage(navCommand);
        send_message(msg);
    }
}

void UDP_Connection::sendChangeToGuidedModeCommand()
{
    mavlink_message_t msg = mControl.getChangeToGuidedModeCommand(system_id,autopilot_id);
    send_message(msg);
}

void UDP_Connection::sendChangeToBrakeModeCommand()
{
    mavlink_message_t msg = mControl.getChangeToBrakeModeCommand(system_id,autopilot_id);
    send_message(msg);
}


void UDP_Connection::send_message(mavlink_message_t message)
{
    unsigned char buf[300];

    // Translate message to buffer
    unsigned len = mavlink_msg_to_send_buffer((uint8_t*)buf, &message);
    for(int i = 0; i < len; i++)
    {
        printf("%d:", *(buf+i));
    }

    // Write buffer to serial port, locks port while writing
    int n = rio_writen(c2mSock, buf, len);
}

void UDP_Connection::handle_quit(int sig)
{
    try {
        //stop();
        bShouldExit = true;
    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop serial port\n");
    }
}

/*
 * UDPConnection.h
 *
 *  Created on: Nov 2, 2016
 *      Author: DoThanhTuan
 */

#ifndef UDPCONNECTION_H_
#define UDPCONNECTION_H_

//[Do.Tuan]
#include "os.h"
#include <iostream>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdio.h>

#include "Common.h"
#include "Control.h"

void* runningThread(void *args);

class UDP_Connection {
public:
    UDP_Connection(Control control);
    virtual ~UDP_Connection();

    int setupM2CSock(int port);
    int setupC2MSock(char *desip, int port);

    void start();
    void run();

    double getPeriodOfTimeInSecondSinceStartTime(timeval startTime);

    int readData(int fd, void *usrbuf, size_t n);
    void handle_message(int sock);
    void decode_message(unsigned char *rxBuff);

    void sendArmCommand();
    void sendTakeoffCommand();
    void sendLandCommand();
    void sendChangeToGuidedModeCommand();
    void sendChangeToBrakeModeCommand();
    void sendControlCommand(NAV_CMD_TYPE navCommand);
    void send_message(mavlink_message_t message);

    void writeLog(NAV_CMD_TYPE navCommand, double time);
    void handle_quit( int sig );
    
    uint64_t get_time_usec();

private:
    pthread_t mCommunicationThread;
    bool isThreadRunning;

    int c2mSock, m2cSock;
    int serverPort;
    int serverIP;
    FILE* fileSpeed;

    Control mControl;
    Mavlink_Messages current_messages;

    int system_id;
    int autopilot_id;
    int companion_id;

    bool bIsHasInitialPosition;
    mavlink_set_position_target_local_ned_t current_position;
    double mCurrentHeading;
    
    bool bShouldExit;
};

#endif /* UDPCONNECTION_H_ */

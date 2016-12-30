/****************************************************************************
 *
 *   Copyright (c) 2014 MAVlink Development Team. All rights reserved.
 *   Author: Trent Lukaczyk, <aerialhedgehog@gmail.com>
 *           Jaycee Lock,    <jaycee.lock@gmail.com>
 *           Lorenz Meier,   <lm@inf.ethz.ch>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file mavlink_control.cpp
 *
 * @brief An example offboard control process via mavlink
 *
 * This process connects an external MAVLink UART device to send an receive data
 *
 * @author Trent Lukaczyk, <aerialhedgehog@gmail.com>
 * @author Jaycee Lock,    <jaycee.lock@gmail.com>
 * @author Lorenz Meier,   <lm@inf.ethz.ch>
 *
 */



// ------------------------------------------------------------------------------
//   Includes
// ------------------------------------------------------------------------------

#include "mavlink_control.h"
#include <sys/stat.h>
#include <X11/Xlib.h>

// ------------------------------------------------------------------------------
// Variables
// ------------------------------------------------------------------------------
//const char* SAVE_DATA_PATH = "SaveData";
//const char* GPS_FILE_NAME = "SaveData/gps.txt";
//const char* SPEED_FILE_NAME = "SaveData/speed.txt";
volatile NAV_CMD_TYPE mNavigationCommand;
volatile float velocityX;
volatile float velocityY;
volatile double velocity;
volatile double timeStep;// Save time value of each step
volatile bool bIsDroneStopped;
volatile bool isAutoMode;
volatile bool isLandModeClicked;
timeval timeStepBegin; // Calculate time step for Canndy detection, Hough line, Vanishing , Tracking, SendCMD2Control
timeval visionBegin;
FILE *visionProcessFile;
FILE *visionPositionFile;

const char* SAVE_ORIGINAL_PATH = "SaveData/OriginalFrame";
const char* SAVE_VISION_RESULT_PATH = "SaveData/VisionResult";
const char* SAVE_VISION_TIME_PATH = "SaveData/VisionTime";
const char* SAVE_VISION_POSITION_PATH = "SaveData/VisionPosition";

std::vector<StoredLine> sLines;
// ------------------------------------------------------------------------------
//   TOP
// ------------------------------------------------------------------------------
int
top (int argc, char **argv)
{

	// --------------------------------------------------------------------------
	//   PARSE THE COMMANDS
	// --------------------------------------------------------------------------

	// Default input arguments
#ifdef __APPLE__
	char *uart_name = (char*)"/dev/tty.usbmodem1";
#else
	char *uart_name = (char*)"/dev/ttyUSB0";
#endif
	int baudrate = 57600;

	// do the parse, will throw an int if it fails
	//parse_commandline(argc, argv, uart_name, baudrate);

	// Start connection thread
        if( argc >= 2)
        {
            string frameType(argv[1]);
       
            if (frameType == "1")
            {
                mControl.setCoordinateFrame(MAV_FRAME_LOCAL_NED);
                printf("FRAME: LOCAL_NED\n");
            } 
            else if (frameType == "7")
            {
                mControl.setCoordinateFrame(MAV_FRAME_LOCAL_OFFSET_NED);
                printf("FRAME: LOCAL_OFFSET_NED\n");
            } 
            else if (frameType == "8")
            {
                mControl.setCoordinateFrame(MAV_FRAME_BODY_NED);
                printf("FRAME: BODY_NED\n");
            } 
            else if (frameType == "9")
            {
                mControl.setCoordinateFrame(MAV_FRAME_BODY_OFFSET_NED);
                printf("FRAME: BODY_OFFSET_NED\n");
            }
        }
        
	udpConnection = new UDP_Connection(mControl);
	udpConnection->start();

	
	
        
        //Create Vision Folders
         struct stat st = {0};
        if (stat(SAVE_ORIGINAL_PATH, &st) == -1)
        {
            mkdir(SAVE_ORIGINAL_PATH, 0700);
        }
        else
        {
              system("exec rm -r SaveData/OriginalFrame/*");
        }
         
        struct stat st_vision = {0};
        if (stat(SAVE_VISION_RESULT_PATH, &st_vision) == -1)
        {
            mkdir(SAVE_VISION_RESULT_PATH, 0700);
        }
        else
        {
            system("exec rm -r SaveData/VisionResult/*");
            
        }
        //Create Vision Time
        struct stat st_vision_data = {0};
        if (stat(SAVE_VISION_TIME_PATH, &st_vision_data) == -1)
        {
            mkdir(SAVE_VISION_TIME_PATH, 0700);
        }
        else
        {
            system("exec rm -r SaveData/VisionTime/*");
        }
        visionProcessFile = fopen("SaveData/VisionTime/visionTime.txt", "w");
        if(visionProcessFile == NULL)
        {
            std::cout << "Can not open file visionTime.txt" << std::endl;
            return false;
        }
        else 
        {
            fprintf(visionProcessFile, "Frame       AdaptiveROI     LineDetection       Processing     SendCMD\n");
        }
        //Create Vision Position file
         struct stat st_position_data = {0};
        if (stat(SAVE_VISION_POSITION_PATH, &st_position_data) == -1)
        {
            mkdir(SAVE_VISION_POSITION_PATH, 0700);
        }
        else
        {
            system("exec rm -r SaveData/VisionPosition/*");
        }
        visionPositionFile = fopen("SaveData/VisionPosition/visionPosition.txt", "w");
        if(visionPositionFile == NULL)
        {
            std::cout << "Can not open file visionPosition.txt" << std::endl;
            return false;
        }
        else 
        {
              fprintf(visionPositionFile, "Frame     CurvedRATIO   AngleOfHeading  AngleOfRoad        TIME          POS&CMD         RealCMD          VelocityX           VelocityY\n");
         }
	// Start vision thread
	mVision = new Vision();   
        
        if( argc >= 3)
        {
            string laneType(argv[2]);
            if (laneType == "1")
            {
                mVision->setLaneType(CurvedLane);
                cout<<"CURVED LANE"<<endl;
            }
            else if (laneType == "2")
            {
                mVision->setLaneType(MixedLane);
                cout<<"MIXED LANE"<<endl;
            }
            else
            {
                cout<<"STRAIGHT LANE"<<endl;
            }
        }
        mVision->start();
        
	signal(SIGINT,quit_handler);


	return 0;

}

// ------------------------------------------------------------------------------
//   Parse Command Line
// ------------------------------------------------------------------------------
// throws EXIT_FAILURE if could not open the port
void
parse_commandline(int argc, char **argv, char *&uart_name, int &baudrate)
{

	// string for command line usage
	const char *commandline_usage = "usage: mavlink_serial -d <devicename> -b <baudrate>";

	// Read input arguments
	for (int i = 1; i < argc; i++) { // argv[0] is "mavlink"

		// Help
		if (strcmp(argv[i], "-h") == 0 || strcmp(argv[i], "--help") == 0) {
			printf("%s\n",commandline_usage);
			throw EXIT_FAILURE;
		}

		// UART device ID
		if (strcmp(argv[i], "-d") == 0 || strcmp(argv[i], "--device") == 0) {
			if (argc > i + 1) {
				uart_name = argv[i + 1];

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

		// Baud rate
		if (strcmp(argv[i], "-b") == 0 || strcmp(argv[i], "--baud") == 0) {
			if (argc > i + 1) {
				baudrate = atoi(argv[i + 1]);

			} else {
				printf("%s\n",commandline_usage);
				throw EXIT_FAILURE;
			}
		}

	}
	// end: for each input argument

	// Done!
	return;
}

// ------------------------------------------------------------------------------
//   Quit Signal Handler
// ------------------------------------------------------------------------------
// this function is called when you press Ctrl-C
void
quit_handler( int sig )
{
	printf("\n");
	printf("TERMINATING AT USER REQUEST\n");
	printf("\n");

	bShouldExit = true;

	try {
	        udpConnection->handle_quit(sig);
	    }
	    catch (int error){}

    // Vision
    try {
        mVision->handle_quit(sig);
    }
    catch (int error){}

	// end program here
	exit(0);

}

// ------------------------------------------------------------------------------
//   Main
// ------------------------------------------------------------------------------
const int SCREEN_WIDTH = 120*4;
const int SCREEN_HEIGHT = 90*3;
const int SCREEN_BPP = 32;
    
int
main(int argc, char **argv)
{
    XInitThreads();
    SDL_Event event;
    bShouldExit = false;
    mNavigationCommand = NONE;

    // This program uses throw, wrap one big try/catch here
    try
    {
        //-----------------------------------
        // Run connection and vision thread
        //-----------------------------------
        int result = top(argc,argv);

        // UI
        //Initialize
        if( mSDL.init() == false )
        {
            return 1;
        }

        //Clip the sprite sheet
        mSDL.set_clips();

        //Init buttons
        mSDL.init_buttons(3, 4, SCREEN_WIDTH, SCREEN_HEIGHT);

        //While the user hasn't quit
        while( bShouldExit == false )
        {
            //If there's events to handle
            if( SDL_PollEvent( &event ) )
            {
                //Handle button events
                mSDL.init_Button_Event_Handler(event);

                //If the user has Xed out the window
                if( event.type == SDL_QUIT )
                {
                    //Quit the program
                    bShouldExit = true;
                }
            }

            //Fill the screen white
            mSDL.fillScreen();

            //Show the button
            mSDL.showButtons();

            //Update the screen
            if(  mSDL.update() == -1 )
            {
                return 1;
            }
        }
    }

    catch ( int error )
    {
            fprintf(stderr,"mavlink_control threw exception %i \n" , error);
            return error;
    }

}



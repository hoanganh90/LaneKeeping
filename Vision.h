//------------------------------------------------------------------------
//               HANDLE VISION PART
//------------------------------------------------------------------------

#ifndef VISION_H_
#define VISION_H_

#include <iostream>
#include <stdio.h>
#include "Common.h"
#include "Vision.h"

#ifdef WIN32
#include <windows.h>
#endif
#include <fstream>//cong.anh write to txt file
#include <string>
#ifdef linux
#include <stdio.h>
#endif

#define USE_PPHT
#define MAX_NUM_LINES	200

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/videoio/videoio.hpp"
#include <libavutil/avconfig.h>
#include "lmmin.h"
#include "MSAC.h"
#include "LaneInfo.h"
#include <sys/time.h> 
#include "math.h"
extern "C" {
#ifndef UINT64_C
#define UINT64_C(c) (c ## ULL)
#endif

#include "libavutil/imgutils.h"
#include "libavutil/opt.h"
#include "libavcodec/avcodec.h"
#include "libavutil/mathematics.h"
#include "libavutil/samplefmt.h"
#include "libavformat/avformat.h"
#include "libswscale/swscale.h" // need to add swscale to linker to convert from AVFrame to cv::MAt
//#include "libavutil/frame.h"
//cong.anh
#include <pthread.h>
}

#include "LineSegment.h"
#include "Common.h"
using namespace std;
using namespace cv;

//cong.anh
struct RoiData {
	int index;
	cv::Point2f vp;
	vector<StoredLine> lineSegments;
	cv::Mat subRoi;
        bool isAccepted;
};
struct commandData {
	POSITION pos;
	LaneInfo pLane;
};

typedef enum 
{
    StraightLane,//control drone in a straight lane
    CurvedLane,//control drone in a curved lane
    MixedLane           //control drone in a mixed(both straight and curved) lane
}
LaneType;

//---------------------------------------------
//                Vison class
//---------------------------------------------

class Vision {
public:
    Vision();
    virtual ~Vision();

    void start();
    void run();

    void handle_quit( int sig );

    //MARK: Vision
    void drawRoiImage(cv::Mat image, vector<StoredLine> &lineSegments, int index);
    int onROIFrame(cv::Mat subRoi, cv::Point2f &vanishingPoint,vector<StoredLine> &lineCS, int index);
    void detectLefnRightLanes(vector<StoredLine> &sLines, cv::Point2f &bottomPoint,
		vector<StoredLine> &leftSide, vector<StoredLine> &rightSide);
    void filterLineSegment(std::vector<StoredLine>&lineSegment);
    void processImage(cv::Mat &imgGRAY);
    int onVisionFrame();
    int onVisionFrame_Rpi();
    float calculateCurvedLine(cv::Point2f C0, cv::Point2f C1, cv::Point2f C2,cv::Point2f &centerCirle);
    void sendCMD2StraightControl(POSITION pos, LaneInfo primaryLane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime);
    //Test sendCMDtoCurevedControl: 2016.12.10: not success
    void sendCMD2StraightControl2(LaneInfo primaryLane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime);
    
    void sendCMD2CurvedControl(LaneInfo primarylane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime);
    //Test cendCMD2CurvedControl
    void sendCMD2CurvedControl2(LaneInfo primarylane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime);
    double calculatePeriodOfTime(timeval startTime);
    bool logVisionProcessStep(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
    bool logVisionPosition(int frameNum,LaneInfo primarylane, std::string strPos, std::string navCMD,float curvedRatio, float angleHeading, float angleOfRoad, double deltaTime,float velocityX,float velocityY);
    void setLaneType(LaneType lane);
   
private:
    pthread_t mVisionThread;
    bool bShouldExit;

    int countFrame ;
    bool isFirstFrameDetected;
    cv::Point2f bottomPoint;
    POSITION position;
    //POSITION lastPosition;
    LaneInfo primaryLane;
    bool isEndOfLineProcessLog;
   // bool isEndOfLinePosionLog;
    cv::Point2f C_0;
    cv::Point2f C_1;
    cv::Point2f C_2;
    //cong.anh: Set only 1 Copy
    bool bDataIsNotCopied;
   
 
public:
    struct RoiData args[2];
    int mRoiDataIndex;
    //cong.anh
    bool bIsRotate;
    LaneType laneType;
};

#endif
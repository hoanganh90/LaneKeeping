//------------------------------------------------------------------------
//               HANDLE VISION PART
//------------------------------------------------------------------------

#include <vector>
#include <cmath>
#include <unistd.h>
#include <sys/stat.h>
//#include <raspicam/raspicam_cv.h>
#include "Vision.h"
#include <X11/Xlib.h>

Vision::Vision() {
    // TODO Auto-generated constructor stub
    bShouldExit = false;
    mRoiDataIndex = 0;
    countFrame = 0;
    isFirstFrameDetected = false;
    bIsRotate = false;
   // isEndOfLinePosionLog = false;
    isAutoMode = false;
    isLandModeClicked = false;
    C_0 = cv::Point2f(0,0);
    C_1 = cv::Point2f(0,0);
    C_2 = cv::Point2f(0,0);
    laneType = StraightLane;
    bDataIsNotCopied = true;
}

Vision::~Vision() {
    // TODO Auto-generated destructor stub
}

void Vision::setLaneType(LaneType lane)
{
    laneType = lane;
}

void* runningVisionThread(void *args)
{
    Vision* vision = (Vision*) args;
    vision->run();
    return NULL;
}

void *roi_process_thread(void *arg) {
    Vision* vision = (Vision*) arg;
//	int *index = (int*) arg;
//	onROIFrame(rd[*index].subRoi, rd[*index].vp, rd[*index].lineSegments);
//	cout << "**END roi_thread *****" << endl;

    struct RoiData *roiData = (struct RoiData*) &vision->args[vision->mRoiDataIndex];
    int inRoi = vision->onROIFrame(roiData->subRoi, roiData->vp, roiData->lineSegments, roiData->index);
   // std::cout<<"***After THREAD: VP:"<<roiData->vp<<" Number of Lines: "<< roiData->lineSegments.size()<<std::endl;
    if(inRoi == -1)
    {
      //  cout<<"Cannot process ROI"<<endl;
        roiData->isAccepted = false;
    }
    else
        roiData->isAccepted = true;
    pthread_exit(NULL);
    return NULL;
}


void Vision::start() {

    printf("START READ THREAD \n");

    int result = pthread_create( &mVisionThread, NULL, &runningVisionThread, this );
    if ( result ) throw result;
}

void Vision::run()
{

    //----------------------------
    //     Start Camera
    //----------------------------
    //For Raspberry
    //onVisionFrame_Rpi();
    
    //For Linux laptop
   onVisionFrame();
   

}

void Vision::handle_quit(int sig)
{
    try {

        bShouldExit = true;

        //---------------------------------
        //        Stop Camera
        //---------------------------------


    }
    catch (int error) {
        fprintf(stderr,"Warning, could not stop serial port\n");
    }
}

void Vision::drawRoiImage(cv::Mat image, vector<StoredLine> &lineSegments, int index) {

    for (unsigned int c = 0; c < lineSegments.size(); c++) {
            Point pt1 = lineSegments[c].point1;
            Point pt2 = lineSegments[c].point2;
            line(image, pt1, pt2, CV_RGB(0, 0, 0), 2);
    }
    if (index == 0) {
            //imshow("Output IM1", image);
            imwrite("test1.jpg", image);
    } else {
            //imshow("Output IM2", image);
            imwrite("test2.jpg", image);

    }
}

int Vision::onROIFrame(cv::Mat subRoi, cv::Point2f &vanishingPoint,
		vector<StoredLine> &lineCS, int index) {
    bool verbose = false;
    int width = subRoi.cols;
    int height = subRoi.rows;
    int mode = MODE_LS;
    cv::Size procSize = cv::Size(width, height);
    vector<vector<cv::Point> > lineSegments;
    MSAC msac;
    msac.init(mode, procSize, verbose);
    cv::Mat image, imageDilation, imgCanny;
    int numVps = 1;
    //1. Morphological transform : Dilation
    image = subRoi.clone();
    
 //  cout << "***onROIFrame****" << endl;
    msac.dilation(image, imageDilation, 1, 4);
//	if (index == 0) {
//		imwrite("ROI1_d.jpg", imageDilation);
//	} else
//		imwrite("ROI2_d.jpg", imageDilation);
    //3. Edge Detection using Canny
 //    cout << "***Canny****" << endl;
    cv::Canny(imageDilation, imgCanny, 180, 120, 3);

    //Result
//    imwrite("imageCanny.jpg", imgCanny); //Keep the name of sample imageROI.jpg
    // Hough
    //vector<vector<cv::Point> > lineSegments;
    vector<cv::Point> aux;
#ifndef USE_PPHT
    /*	printf("cong.anh: Using PPHT\n");*/
    vector<Vec2f> lines;
    cv::HoughLines(imgCanny, lines, 1, CV_PI / 180, 200);
    //cong.anh Hough transform
    for (size_t i = 0; i < lines.size(); i++) {
            float rho = lines[i][0];
            float theta = lines[i][1];

            double a = cos(theta), b = sin(theta);
            double x0 = a * rho, y0 = b * rho;

            Point pt1, pt2;
            pt1.x = cvRound(x0 + 1000 * (-b));
            pt1.y = cvRound(y0 + 1000 * (a));
            pt2.x = cvRound(x0 - 1000 * (-b));
            pt2.y = cvRound(y0 - 1000 * (a));

            aux.clear();
            aux.push_back(pt1);
            aux.push_back(pt2);
            lineSegments.push_back(aux);

            //line(outputImg, pt1, pt2, CV_RGB(0, 0, 0), 1, 8);

    }
    //cong.anh
    //cv::imwrite("Hough.bwp",outputImg);
#else

    /*printf("cong.anh: Using HoughLines\n ");*/
    //vector<Vec4i> lines;
    int houghThreshold = 50;
   // printf("Hough transform\n");
    vector<Vec4f> lines;
    cv::HoughLinesP(imgCanny, lines, 1, CV_PI / 180, houghThreshold, 10, 10);

    while (lines.size() > MAX_NUM_LINES) {
            lines.clear();
            houghThreshold += 10;
            cv::HoughLinesP(imgCanny, lines, 1, CV_PI / 180, houghThreshold, 10,
                            10);
    }
  //  cout << "Line size = " << lines.size() << endl;
    if(lines.size() == 0)
        return -1;
    for (size_t i = 0; i < lines.size(); i++) {
            Point pt1, pt2;
            pt1.x = lines[i][0];	// diem dau va diem cuoi cua 1 doan
            pt1.y = lines[i][1];
            pt2.x = lines[i][2];
            pt2.y = lines[i][3];

            // Store into vector of pairs of Points for msac
            aux.clear();
            aux.push_back(pt1);
            aux.push_back(pt2);
            lineSegments.push_back(aux);
    }

#endif
    //cong.anh: Draw the best consume set
    for (size_t i = 0; i < lineSegments.size(); i++) {
            Point pt1, pt2;

            pt1 = lineSegments[i][0];	// diem dau va diem cuoi cua 1 doan
            pt2 = lineSegments[i][1];
            //line(outputImg, pt1, pt2, CV_RGB(214, 37, 152), 2);
    }
    // Multiple vanishing points
    std::vector<cv::Mat> vps;// vector of vps: vps[vpNum], with vpNum=0...numDetectedVps
    std::vector<std::vector<int> > CS;// index of Consensus Set for all vps: CS[vpNum] is a vector containing indexes of lineSegments belonging to Consensus Set of vp numVp
    std::vector<int> numInliers;

    std::vector<std::vector<std::vector<cv::Point> > > lineSegmentsClusters;
    //cong.anh
    if (lineSegments.size() <= 3)
            return -1;
    // cong.anh the list of lines of the next frame
    std::vector<std::vector<cv::Point> > lineCurrent;

    std::vector<std::vector<cv::Point> > lineVP;

    //isVPcheck = true;
      gettimeofday(&timeStepBegin, 0);
       timeStep = calculatePeriodOfTime(timeStepBegin);
    int ransacResult = msac.multipleVPEstimation(lineSegments, lineSegmentsClusters, numInliers,
                    vps, numVps);
    if(ransacResult == -1)
        return -1;
    for (int v = 0; v < vps.size(); v++) {
//            printf("VP is stored in vps: %d (%.3f, %.3f, %.3f)", v,
//                            vps[v].at<float>(0, 0), vps[v].at<float>(1, 0),
//                            vps[v].at<float>(2, 0));
            fflush(stdout);
            double vpNorm = cv::norm(vps[v]);
            if (fabs(vpNorm - 1) < 0.001) {
                    printf("(INFINITE)");
                    fflush(stdout);
            }
            printf("\n");
    }
    for (unsigned int c = 0; c < lineSegmentsClusters.size(); c++) {
            for (unsigned int i = 0; i < lineSegmentsClusters[c].size(); i++) {
                    StoredLine sline;

                    Point pt1 = lineSegmentsClusters[c][i][0];
                    Point pt2 = lineSegmentsClusters[c][i][1];
                    sline.setPoint(pt1, pt2);
                    lineCS.push_back(sline);
            }
    }
    vanishingPoint.x = vps[0].at<float>(0, 0);
    vanishingPoint.y = vps[0].at<float>(1, 0);
  //  cout << "*****VP: " << vanishingPoint << endl;
    if(abs(vanishingPoint.x) > 1000 || abs(vanishingPoint.y) > 1000)
        return -1;
    //cong.anh : For Debug
    //drawRoiImage(imageDilation, lineCS, index);
    return 0;
}

bool sortByDistance(const StoredLine &lp1, const StoredLine &lp2) {
	return lp1.distance < lp2.distance;
}

void Vision::detectLefnRightLanes(vector<StoredLine> &sLines, cv::Point2f &bottomPoint,
		vector<StoredLine> &leftSide, vector<StoredLine> &rightSide) {
	//Detect left&right lane

	for (int i = 0; i < sLines.size(); i++) {

		if (sLines[i].intersection2Bottom.x < bottomPoint.x) {
			StoredLine aux;
			aux.intersection2Bottom = sLines[i].intersection2Bottom;
                        aux.intersection2Middle = sLines[i].intersection2Middle;
			aux.point1 = sLines[i].point1;
			aux.point2 = sLines[i].point2;
			aux.cpoint = sLines[i].cpoint;
			aux.distance = sLines[i].distance;
			aux.RoiPos = sLines[i].RoiPos;
			leftSide.push_back(aux);
		} else if (sLines[i].intersection2Bottom.x > bottomPoint.x) {
			StoredLine aux;
			aux.intersection2Bottom = sLines[i].intersection2Bottom;
                        aux.intersection2Middle = sLines[i].intersection2Middle;
			aux.point1 = sLines[i].point1;
			aux.point2 = sLines[i].point2;
			aux.cpoint = sLines[i].cpoint;
			aux.distance = sLines[i].distance;
			aux.RoiPos = sLines[i].RoiPos;
			rightSide.push_back(aux);
		}
	}
	std::sort(leftSide.begin(), leftSide.end(), sortByDistance);
	std::sort(rightSide.begin(), rightSide.end(), sortByDistance);
}
void Vision::filterLineSegment(std::vector<StoredLine>&lineSegment) {
	for (int i = lineSegment.size() - 1; i >= 0; i--) {
            for (int j = lineSegment.size() - 1; j >= 0; j--) {
                if (j != i) {
                    int delta;
                    delta = abs(
                                    lineSegment[i].intersection2Bottom.x
                                                    - lineSegment[j].intersection2Bottom.x);
                    double res = cv::norm(
                                    lineSegment[i].cpoint - lineSegment[j].cpoint); //Euclidian distance
                    if (delta < 20 || res < 50) {

                            lineSegment.erase(lineSegment.begin() + j);
                            i = i - 1;
                    }
                }
            }
	}
}
void Vision::processImage( cv::Mat &imgGRAY)
 {
        //images
	cv::Mat imgCanny,imageROI,imgDilation,image;
        char input_image[250];
        //3 points of mid-line
        //cv::Point2f C_0(0, 0), C_1(0, 0), C_2(0, 0);
        cv::Point2f tmpC_0(0, 0), tmpC_1(0, 0), tmpC_2(0, 0);
        //size of each ROI
        cv::Point roi1StartPoint, roi2StartPoint;
        int roi1Width, roi1Height, roi2Width, roi2Height;
        int isFoundPrimaryLane = 0;
        //roi1Length = distance (C_0 & C1) - roi2Length = distance (C_1 & C2)
        double roi1Length = 0, roi2Length = 0;        
        //roi2Ratio = delta =  roi2Ratio = roi2Length / (roi1Length + roi2Length)
        double roi2Ratio = 0;
	//if C_0. C_1, C_2 do not satisfy the condition => isEscape = true
        bool isEscaped = false;
        int numLineROI1;//number of detected lines in ROI1 after RANSAC
        cv::Rect roi1Zone,roi2Zone;//roi1Zone + roi2Zone = original ROI
        //Vectors contain detected lines in ROI1 & ROI2 after RANSAC
        vector<StoredLine> lineROI1, lineROI2;
	cv::Point2f vanishingPoint1, vanishingPoint2;
	cv::Point2f topROI, centerBottom, centerTop;
        float lengthC0C1 =0,lengthC1C2=0, curvedRatio = 0;
        float laneThreshold = 0.1;
        MSAC msac;
        //processing ROI1
        std::vector<MatchedLine> cLines;
        vector<StoredLine> leftSideROI1;
	vector<StoredLine> rightSideROI1;
        pthread_t lineSegmentDetection[2];
        //Angle between C_0 - C_1 - C_2: Use in curved line
	float angleOfRoad = 0;
        float angleHeading = 0;
        
        //Drone position in center-bottom point of ROI1
        cv::Point2f dronePos;
//        string filepath ="/home/anh/workspace/LaneKeepingTestResult/Debug/SaveOriginalFrame/";
//        string imagePath = filepath + "Image%04d.jpg";
//        sprintf(input_image,"/home/anh/workspace/LaneKeepingTestResult/Debug/sample4/SaveOriginalFrame/Image%04d.jpg",countFrame);
//        cout << "image file path: " << input_image << endl;
//        image = imread(input_image, 1);
//        //image = imgGRAY;
//        if (!image.data) {
//                return;
//        }
        
        cv::Mat subROI[2];
        args[0].lineSegments.clear();
        args[0].vp = cv::Point(0, 0);
        args[1].lineSegments.clear();
        args[1].vp = cv::Point(0, 0);
        args[0].isAccepted =false;
        args[1].isAccepted =false;
        mRoiDataIndex = 0;
        leftSideROI1.clear();
        rightSideROI1.clear();
        
        //1. ROI Selection
      
        gettimeofday(&timeStepBegin, 0);      
        //Original ROI
        cv::Rect rect(0, imgGRAY.rows * 0.25, imgGRAY.cols, imgGRAY.rows * 0.75);
        imageROI = imgGRAY(rect);
   
        //imageROI = image;
        if (!imageROI.data) {
                return;
        }
    
        //Separate ROI
	//|-----------------------------------
	//|
	//|
	//|ROI2
	//|
	//|
	//|-----------------------------------
	//|
	//|
	//|ROI1
	//|
	//|
	//|-----------------------------------
        //1.Adaptive ROI
       if (!isFirstFrameDetected) {
            //Is first frame : roiRatio = 0.5
            roi2Ratio = 0.5;
            roi1Width = imageROI.cols;
            roi1Height = imageROI.rows * roi2Ratio;

            roi2Width = imageROI.cols;
            roi2Height = imageROI.rows * roi2Ratio;
            roi2StartPoint = cv::Point(0, 0);
            roi1StartPoint = cv::Point(0, roi1Height);
            gettimeofday(&visionBegin, 0);
        } else {
            if (!isEscaped) {
                    roi1Length = cv::norm(C_0 - C_1);
                    roi2Length = cv::norm(C_1 - C_2);
                    if (roi1Length == 0 || roi2Length == 0) {
                            roi2Ratio = 0.5;
                    } else
                            roi2Ratio = roi2Length / (roi1Length + roi2Length);
            } else {
                    roi2Ratio = 0.5;
                    isEscaped = false;
            }
            roi1Width = imageROI.cols;
            roi1Height = (int) (imageROI.rows * roi2Ratio);

            roi2Width = imageROI.cols;
            roi2Height = imageROI.rows - roi1Height;

            roi2StartPoint = cv::Point(0, 0);
            roi1StartPoint = cv::Point(0, imageROI.rows - roi1Height);
//            cout << " delta " << roi2Ratio << " Roi1Height " << roi1Height
//                            << " Roi2Height: " << roi2Height << endl;
        }

        roi1Zone = cv::Rect(roi1StartPoint.x, roi1StartPoint.y, roi1Width,roi1Height);
        roi2Zone= cv::Rect(0, 0, roi2Width, roi2Height);
        subROI[0] = imageROI(roi1Zone);
        subROI[1] = imageROI(roi2Zone);
   
        timeStep = calculatePeriodOfTime(timeStepBegin);
        //Column 1.Save execution time of Adaptive ROI
        //(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
        logVisionProcessStep(countFrame,timeStep,false, false);   
        gettimeofday(&timeStepBegin, 0);
        
        
        //2. Line segment detection
        for (int i = 0; i < 2; i++) {
           // cout << "Go to thread: " << i << endl;
            args[i].index = i;
            args[i].subRoi = subROI[i].clone();
            int width = args[i].subRoi.cols;
            int height = args[i].subRoi.rows;
            cv::Size procSize = cv::Size(width, height);
            //MARK: Init for using thread
            mRoiDataIndex = i;
            pthread_create(&lineSegmentDetection[i], NULL, &roi_process_thread,
                            (void*) this);
	}
	for (int i = 0; i < 2; i++)
		pthread_join(lineSegmentDetection[i], NULL);
   
	topROI.x = imgCanny.size().width / 2;
	topROI.y = 0;
	bottomPoint.x = subROI[0].cols / 2;
	bottomPoint.y = subROI[0].rows;

	lineROI1 = args[0].lineSegments;
	vanishingPoint1 = args[0].vp;

	lineROI2 = args[1].lineSegments;
	vanishingPoint2 = args[1].vp;
        
        std::cout<<"***After THREAD: VP_1:"<<args[0].vp<<" Number of Lines: "<< args[0].lineSegments.size()<< " VP_2 " <<args[1].vp<<" Number of Lines: "<< args[1].lineSegments.size()<<std::endl;
        topROI.x = subROI[0].cols / 2;
        topROI.y = 0;
        //bottomPoint is Drone's position
        bottomPoint.x = subROI[0].cols / 2;
        bottomPoint.y = subROI[0].rows;

        lineROI1 = args[0].lineSegments;
        vanishingPoint1 = args[0].vp;

        lineROI2 = args[1].lineSegments;
        vanishingPoint2 = args[1].vp;

        tmpC_2 = vanishingPoint2;
        numLineROI1 = lineROI1.size();
        if(numLineROI1 == 0)
            return;
        //line tracking algorithm
        if (!isFirstFrameDetected) {
            printf("\n #%d Frame \n", countFrame);
            for (int i = 0; i < lineROI1.size(); i++) {
                //Stored lines
                StoredLine sline;
                sline.setPoint(lineROI1[i].point1, lineROI1[i].point1);
                sLines.push_back(sline);
                isFirstFrameDetected = true;
            }
            msac.distancePoint2StoredLine(sLines, bottomPoint, topROI);
            return;
            }//In next frames, add all found lines into currentLines list for matching with sLines
            else {
             printf("\n #%d Frame \n", countFrame);
                cLines.clear();
                for (int i = 0; i < lineROI1.size(); i++) {
                    MatchedLine lineSeg;
                    lineSeg.point1 = lineROI1[i].point1;
                    lineSeg.point2 = lineROI1[i].point2;
                    cLines.push_back(lineSeg);
                }
                msac.distancePoint2CurrentLine(cLines, bottomPoint,topROI);
            }
        msac.lineMatching(sLines, cLines, bottomPoint);
        
	detectLefnRightLanes(sLines, bottomPoint, leftSideROI1, rightSideROI1);
        if(leftSideROI1.size() == 0 || rightSideROI1.size() == 0 || tmpC_2.x > subROI[1].cols
                                       || tmpC_2.x < 0 || tmpC_2.y < 0 || tmpC_2.y > subROI[1].rows)
        {   
            cout << "No acceptable. Go to escape" << endl;
             //logVisionProcessStep(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
            logVisionProcessStep(countFrame,timeStep,true, false);   
            isEscaped = true;
            return;
        }
        timeStep = calculatePeriodOfTime(timeStepBegin);
        //Column 2.Save execution time of Processing lines in ROI1 - Thread
        //(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
        logVisionProcessStep(countFrame,timeStep,false, false);   
        //3. Processing line
        gettimeofday(&timeStepBegin, 0);
//	cout << "Number of LEFT LINE BEFORE = " << leftSideROI1.size() << endl;
	filterLineSegment(leftSideROI1);
//	cout << "Number of LEFT LINE AFTER = " << leftSideROI1.size() << endl;
//	cout << "Number of RIGHT LINE BEFORE = " << rightSideROI1.size() << endl;
	filterLineSegment(rightSideROI1);
//	cout << "Number of LINE AFTER = " << rightSideROI1.size() << endl;
        
	//2016.11.20: Detect primary lane
        // Detect center line of the primary lane also
	isFoundPrimaryLane = msac.detectPrimaryLane(leftSideROI1, rightSideROI1, centerBottom,centerTop, primaryLane);
        if(isFoundPrimaryLane == -1)
            return;
        tmpC_0 = centerBottom;
        tmpC_1 = centerTop;  
      
        //Save execution time of Estimating the mide line of road
        /*
	 * Output: steering angle and the position Left | Right | Inside the Primary Lane
	 */
        angleHeading = msac.estmateAnglenPosition(primaryLane, bottomPoint,tmpC_1, position);// in Degree
         
        char imageFileName[128];
        snprintf(imageFileName, sizeof(imageFileName), "SaveData/OriginalFrame/Image%04d.jpg", countFrame);
        imwrite(imageFileName, imageROI);
        msac.dilation(imageROI,imgDilation,1, 4);
     	msac.drawImage(imgDilation, leftSideROI1, rightSideROI1, lineROI2,tmpC_0, tmpC_1, tmpC_2, vanishingPoint1, vanishingPoint2, countFrame,roi2Height, roi2Ratio, numLineROI1, position);
        C_0 = tmpC_0;
        C_1 = tmpC_1;
        C_2 = tmpC_2;
        gettimeofday(&timeStepBegin, 0);
        
        
        double visionTime = calculatePeriodOfTime(visionBegin); //Average time for receiving a video frame to vision thread
        double deltaTime = visionTime; 
        //For curved line
        angleOfRoad = msac.angleBetween2LinesAtan2(C_0,C_1,C_1,C_2);//In Degree
        
        timeStep = calculatePeriodOfTime(timeStepBegin);
        //column 3: Save processing line (int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
        logVisionProcessStep(countFrame,timeStep,false, false);   
        gettimeofday(&timeStepBegin, 0);
        lengthC0C1 = cv::norm(C_0 - vanishingPoint1);
        lengthC1C2 = cv::norm(vanishingPoint1 - vanishingPoint2);
        curvedRatio = lengthC1C2/lengthC0C1;
        if(laneType == StraightLane)
        {
             sendCMD2StraightControl(position,primaryLane,curvedRatio, angleHeading, angleOfRoad, bottomPoint,deltaTime);
        }
        else if(laneType == CurvedLane)
        {
            sendCMD2CurvedControl2(primaryLane,curvedRatio, angleHeading, angleOfRoad,bottomPoint,deltaTime);
        }
        else
        {
            if(curvedRatio < laneThreshold)//Straight lane
            {
                sendCMD2StraightControl2(primaryLane,curvedRatio, angleHeading, angleOfRoad,bottomPoint,deltaTime);
            }
            else // Curved lane
            {
                 sendCMD2CurvedControl2(primaryLane,curvedRatio, angleHeading, angleOfRoad,bottomPoint,deltaTime);
            }
        }
        //Save execution time of Output control commands
        timeStep = calculatePeriodOfTime(timeStepBegin);
        //(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
        logVisionProcessStep(countFrame,timeStep,false, false);   

        //Fishing loging for 1 frame
        logVisionProcessStep(countFrame,0,true, false);   
       
}
int Vision::onVisionFrame() {
    //--------------------------
    //Start camera
    //--------------------------
    cout<<"Go to Vision"<<endl;
    cv::VideoCapture capture(0); //Capture using any camera connected to your system
    cv::Mat imageFrame,resizeFrame;
    Size size(640, 480);	//the dst image size 640x480
    if(!capture.isOpened())
          return -1;
    capture.set(CV_CAP_PROP_FRAME_WIDTH,640);
    capture.set(CV_CAP_PROP_FRAME_HEIGHT,480);
    capture.set(CV_CAP_PROP_FORMAT,CV_8UC1);
       
    //----------------------------
    //     Vision Processing
    //----------------------------
    while(!bShouldExit)
    {
        //Start recording log: Write frame number first
        logVisionProcessStep(countFrame,timeStep,false, true);     
        //compute time to covert2GrayScale
        gettimeofday(&timeStepBegin, 0);  
        capture>>imageFrame;
//        transpose(imageFrame, imageFrame);
//        flip(imageFrame, imageFrame,-1);
       
        args[0].lineSegments.clear();
        args[1].lineSegments.clear();
        
        if(!imageFrame.data)
	{
		cout<<"ERROR: Cannot read a frame from RPI"<<endl;
		return -1;
	}
	else
	{
            //imwrite("Input.jpg", imageFrame);
            imshow("Input Image",imageFrame);
            char q = (char) waitKey(1);
            if (q == 27) {
                    printf("\nStopped by user request\n");
                    break;
            }
	}
        cout<<"Come here while loop in vision thread"<<endl;
        //cong.anh
        if(isAutoMode)
        {
            cv::Mat inputGray;
            int mode = MODE_LS;
       
        // ++++++++++++++++++++++++++++++++++++++++
        // Process
        // ++++++++++++++++++++++++++++++++++++++++

            timeStep = calculatePeriodOfTime(timeStepBegin);
            //Save execution time of convert2GrayScale
            //logVisionProcessStep(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum);
            logVisionProcessStep(countFrame,timeStep,false, false);  
            processImage( imageFrame);
            countFrame++;
        }
        if(isLandModeClicked&&bDataIsNotCopied)
        {
            int num;
            FILE *fptr;
            fptr = fopen("Result/count.txt", "a+");
            if (fptr == NULL) {
                    printf("Error: Cannot open count.txt!");
                    exit(1);
            }
            //Read file
            struct stat stCheckSize;
            if (stat("Result/count.txt", &stCheckSize) != 0) {
                    return EXIT_FAILURE;
            }
            fprintf(stdout, "file size: %zd\n", stCheckSize.st_size);
            if(stCheckSize.st_size == 0)
            {
                num = 0;
            }
            else
                fscanf(fptr, "%d", &num);
            num++;
            //Delete old number:
            fclose(fptr);
            fptr = NULL;
            //Write file
            fptr = fopen("Result/count.txt", "w");
            fprintf(fptr, "%d", num);
            //Close file
            fclose(fptr);
            std::string logStr = "Result/" + std::to_string(num);	
            std::string src = "SaveData/";
            std::string tmpStr = "cp -R "+ src + " " + logStr;
            struct stat stCopy = { 0 };
            if (stat(logStr.c_str(), &stCopy) == -1) {
                    mkdir(logStr.c_str(), 0700);
                    system(tmpStr.c_str());

            }
            bDataIsNotCopied = false;  
        }
       
   }
   capture.release();
    return 0;
}
    //Computer vision on RPi
    
int Vision::onVisionFrame_Rpi() {
//    time_t timer_begin,timer_end;
//    raspicam::RaspiCam_Cv Camera;
//    cv::Mat image,resizeImg;
//    //Size size(640,480);
//    //set camera params
//    Camera.set( CV_CAP_PROP_FORMAT, CV_8UC1 );
//    Camera.set(CV_CAP_PROP_FRAME_WIDTH, 640);
//    Camera.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
//    //Open camera
//    cout<<"Opening Camera..."<<endl;
//    if (!Camera.open()) {cerr<<"Error opening the camera"<<endl;return -1;}
//
//    //while(1)
//    while(!bShouldExit)
//    {
//        gettimeofday(&timeStepBegin, 0);   
//        Camera.grab();
//        Camera.retrieve (image);
//        if(isAutoMode)
//        {
//            cv::Mat reverseImg;
//            cv::Point2f pc(image.cols/2,image.rows/2);
//            cv::Mat r = cv::getRotationMatrix2D(pc, -180,1);
//            cv::warpAffine(image,reverseImg, r,image.size());
//            if(!reverseImg.data)
//            {
//                    cout<<"ERROR: Cannot read a frame from Bebop"<<endl;
//                    return -1;
//            }
////            else
////            {
////                resize(reverseImg,resizeImg,size);
////            }//           cout<<"Come here while loop in vision thread\n"<<endl;
//           
//        // ++++++++++++++++++++++++++++++++++++++++
//        // Process
//        // ++++++++++++++++++++++++++++++++++++++++
//            timeStep = calculatePeriodOfTime(timeStepBegin);
//            //Save execution time of convert2GrayScale
//            logVisionProcessStep(countFrame,timeStep);       
//            processImage(reverseImg);
//            countFrame++;
//        }
//         if(isLandModeClicked&&bDataIsNotCopied)
//        {
//            int num;
//            FILE *fptr;
//            fptr = fopen("Result/count.txt", "a+");
//            if (fptr == NULL) {
//                    printf("Error: Cannot open count.txt!");
//                    exit(1);
//            }
//            //Read file
//            struct stat stCheckSize;
//            if (stat("Result/count.txt", &stCheckSize) != 0) {
//                    return EXIT_FAILURE;
//            }
//            fprintf(stdout, "file size: %zd\n", stCheckSize.st_size);
//            if(stCheckSize.st_size == 0)
//            {
//                num = 0;
//            }
//            else
//                fscanf(fptr, "%d", &num);
//            num++;
//            //Delete old number:
//            fclose(fptr);
//            fptr = NULL;
//            //Write file
//            fptr = fopen("Result/count.txt", "w");
//            fprintf(fptr, "%d", num);
//            //Close file
//            fclose(fptr);
//            std::string logStr = "Result/" + std::to_string(num);	
//            std::string src = "SaveData/";
//            std::string tmpStr = "cp -R "+ src + " " + logStr;
//            struct stat stCopy = { 0 };
//            if (stat(logStr.c_str(), &stCopy) == -1) {
//                    mkdir(logStr.c_str(), 0700);
//                    system(tmpStr.c_str());
//
//            }
//            bDataIsNotCopied = false;  
//        }
//    
//    return 0;
//    cout<<"Stop camera..."<<endl;
//    Camera.release();
}


float Vision::calculateCurvedLine(cv::Point2f C0, cv::Point2f C1, cv::Point2f C2,
		cv::Point2f &centerCirle) {
	//Radius
	float radius;
	//1.Slope:
	float slope1, slope2;
	//- The line is perpendicular to line C0 - C1:
	slope1 = (C1.y - C0.y) / (C1.x - C0.x);
	//- The line is perpendicular to line C1 - C2:
	slope2 = (C2.y - C1.y) / (C2.x - C1.x);
	if ((C1 == C0) || (C2 == C1) || (C2 == C0)) {
		return -1;
	} else if ((C1.x == C0.x) || (C2.x == C1.x) || (C1.y == C0.y)
			|| (C1.y == C2.y)) {
		centerCirle = (C0 + C2) / 2;
		return radius = cv::norm(centerCirle - C0);
	}
	if (slope1 == slope2) {
		//This could be coincident: C0 - C1 - C2 on a same line: No Circle
		return -1;
	} else {
		//2. Calculate the position of center point
		centerCirle.x = ((slope1 * slope2) * (C2.y - C0.y)
				+ slope1 * (C1.x + C2.x) - slope2 * (C0.x + C1.x))
				/ (2 * (slope1 - slope2));
		centerCirle.y = (-1 / slope1) * (centerCirle.x - (C0.x + C1.x) / 2)
				+ (C0.y + C1.y) / 2;

		//3. Radius
		radius = cv::norm(centerCirle - C0);
		return radius;
	}

}
    
//    int delta = 0;
//    delta = std::abs(primaryLane.rightLine.intersection2Bottom.x - primaryLane.leftLine.intersection2Bottom.x)/10;
//    int thresholdLeft, thresholdRight;
//    thresholdLeft   = primaryLane.leftLine.intersection2Bottom.x + delta;
//    thresholdRight  = primaryLane.leftLine.intersection2Bottom.x + delta*9;
 void Vision::sendCMD2StraightControl(POSITION pos, LaneInfo primarylane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime)
 {
    cout<<"**STRAIGHT LINE control"<<endl;
    int delta = 0;
    delta = std::abs(primaryLane.leftLine.intersection2Bottom.x - primaryLane.rightLine.intersection2Bottom.x)/10;
    int thresholdLeft, thresholdRight;
    thresholdLeft   = primaryLane.leftLine.intersection2Bottom.x + delta*2;
    thresholdRight  = primaryLane.leftLine.intersection2Bottom.x + delta*8;
    
    timeStep = calculatePeriodOfTime(timeStepBegin);
    if(pos == Left_Lane)
    {
        //if(lastPosition != position)
        mNavigationCommand = Right;
        printf("OUT LEFT %d => Command:%d\n",Left_Lane, Right);
        logVisionPosition(countFrame,primarylane,"OUT LEFT", "GO=>RIGHT", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    else if(pos == Right_Lane)
    {
        //if(lastPosition != position)
        mNavigationCommand = Left;
        printf("OUT Right %d => Command:%d\n",Right_Lane, Left);
        logVisionPosition(countFrame,primarylane,"OUT RIGHT", "GO=>LEFT", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    else// in Lane
    {

	if(dronePos.x < thresholdLeft)
	{
            if(abs(angleHeading) < 0.2)
            {
                mNavigationCommand = Forward;
                std::cout<<"*******Near LEFT, angle < 10degree => go Forward\n"<<angleHeading<<std::endl;
                logVisionPosition(countFrame,primarylane,"NearLEFT&Forward", "Forward", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
            }
            else {
                if(bIsDroneStopped)
                {
                    mNavigationCommand = Clockwise;
                    std::cout<<"*******ESTIMATE LEFT & Drone is stopped => go Clockwise angle\n"<<angleHeading<<std::endl;
                    logVisionPosition(countFrame,primarylane,"LEFT&Stopped", "Clockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                }
                else
                {
                    mNavigationCommand = Stop;
                    std::cout<<"********In lane & Drone is moving \n"<<std::endl;
                    logVisionPosition(countFrame,primarylane,"LEFT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                }
            }
        }
        else if(dronePos.x > thresholdRight){
          
            if(abs(angleHeading) < 0.2)
            {
                mNavigationCommand = Forward;
                std::cout<<"*******Near RIGHT, angle < 10degree => go Forward\n"<<angleHeading<<std::endl;
                logVisionPosition(countFrame,primarylane,"NearRIGHT&Forward", "Forward", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
            }
            else
            {
                if(bIsDroneStopped)
                {
                    mNavigationCommand = CounterClockwise;//LEFT //CounterClockwise
                    std::cout<<"********RIGHT threshold & Drone is stopped => go CounterClockwise Angle:\n"<<angleHeading<<std::endl;
                    logVisionPosition(countFrame,primarylane,"RIGHT&Stopped", "CounterClockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                }
                else
                {
                    mNavigationCommand = Stop;
                    std::cout<<"********In lane & Drone is moving \n"<<std::endl;
                    logVisionPosition(countFrame,primarylane,"RIGHT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                }
            }
        }
        else
        {
            std::cout<<"********CENTER => Forward**********\n"<<std::endl;
            mNavigationCommand = Forward;
            logVisionPosition(countFrame,primarylane,"**CENTER**", "FORWARD", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
        }
    }
}
 void Vision::sendCMD2StraightControl2( LaneInfo primarylane,float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime)
 {
    cout<<"**STRAIGHT LINE control ver 2"<<endl;
    int delta = 0;
    delta = std::abs(primaryLane.leftLine.intersection2Bottom.x - primaryLane.rightLine.intersection2Bottom.x)/10;
    float thresholdLeft = 0, thresholdRight = 0;
    float angleThreshold = 20;
    float realLaneSize = 1.2;
    float laneVision = 0, offsetLeft = 0;
    timeval timeRotateBegin, timeMovingBegin; 
    bool bIsOnceStop = false, bIsPrinted = false;
    float rotateSpeed = 10; //10 deg/s
    float droneMovingSpeed = 0.5; // 0.5 m/s
    double timeStep,estimatedRotatedTime,estimatedMovingTime;
    thresholdLeft   = primaryLane.leftLine.intersection2Bottom.x + delta*2;
    thresholdRight  = primaryLane.leftLine.intersection2Bottom.x + delta*8;
    laneVision = abs(primarylane.rightLine.intersection2Bottom.x - primarylane.leftLine.intersection2Bottom.x);
    
    if(abs(angleHeading) < angleThreshold)
    {
        if(dronePos.x < thresholdLeft)//Threshold Left lane => Go Right
        {
            while(true)
            {
                if(bIsOnceStop)
                {
                    timeStep = calculatePeriodOfTime(timeMovingBegin);
                    if(timeStep > estimatedMovingTime)
                    {
                        mNavigationCommand = Forward;
                        break;       
                    }
                    mNavigationCommand = Right;
                    std::cout<<"****Drone is Straight LEFT\n"<<std::endl;
                    if(!bIsPrinted)
                    {
                        logVisionPosition(countFrame,primarylane,"StraightLEFT", "Right", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                        bIsPrinted = true;
                    }
                }
                else
                {
                    mNavigationCommand = Stop;
                    gettimeofday(&timeMovingBegin, 0);
                    estimatedMovingTime = abs(dronePos.x - primarylane.centerLine.intersection2Bottom.x)/laneVision * realLaneSize /droneMovingSpeed ;
                    std::cout<<"********Drone is Straight LEFT \n"<<std::endl;
                    logVisionPosition(countFrame,primarylane,"StraightLEFT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsOnceStop = true;
                }
            }
        }
        else if(dronePos.x > thresholdRight)//Threshold Right Lane => Go LEFT
        {
           while(true)
            {
                if(bIsOnceStop)
                {
                    timeStep = calculatePeriodOfTime(timeMovingBegin);
                    if(timeStep > estimatedMovingTime)
                    {
                        mNavigationCommand = Forward;
                        break;
                    }
                    mNavigationCommand = Left;
                    std::cout<<"****Drone is Straight RIGHT\n"<<std::endl;
                    if(!bIsPrinted)
                    {
                        logVisionPosition(countFrame,primarylane,"StraightRIGHT", "Left", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                        bIsPrinted = true;
                    }
                }
                else
                {
                    mNavigationCommand = Stop;
                    gettimeofday(&timeMovingBegin, 0);
                    estimatedMovingTime = abs(dronePos.x - primarylane.centerLine.intersection2Bottom.x)/laneVision * realLaneSize /droneMovingSpeed ;
                    std::cout<<"********Drone is Straight RIGHT \n"<<std::endl;
                    logVisionPosition(countFrame,primarylane,"StraightRIGHT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsOnceStop = true;
                }
            }  
        }
        mNavigationCommand = Forward;//Forward
        std::cout<<"****Drone is Forward | Angle:\n"<<angleHeading<<std::endl;
        logVisionPosition(countFrame,primarylane,"InCENTER", "Forward", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    else if(angleHeading < angleThreshold * -1 ) // Heading vector is on the Left of mid-lane
    {
       while (true)
       {
            if(bIsOnceStop)
            {
                timeStep = calculatePeriodOfTime(timeRotateBegin);
                if(timeStep > estimatedRotatedTime)
                {
                   // bIsOnceStop = false;
                    mNavigationCommand = Forward;
                    break;
                }
                mNavigationCommand = Clockwise;//Clockwise
                std::cout<<"****Drone is StraightLEFT => Clockwise Angle:\n"<<angleHeading<<std::endl;
                if(!bIsPrinted)
                {
                    logVisionPosition(countFrame,primarylane,"StraightLEFT&Stopped", "Clockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsPrinted = true;
                }
            }
            else
            {
                mNavigationCommand = Stop;
                gettimeofday(&timeRotateBegin, 0);
                estimatedRotatedTime = abs(angleHeading)/rotateSpeed;
                std::cout<<"********Drone is moving - Straight LEFT \n"<<std::endl;
                logVisionPosition(countFrame,primarylane,"StraightLEFT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                bIsOnceStop = true;
            }
       }
    }
    else if(angleHeading > angleThreshold)// Heading vector is on the Right of mid-lane
    {
       while(true)
       {
            if(bIsOnceStop)
            {
                timeStep = calculatePeriodOfTime(timeRotateBegin);
                if(timeStep > estimatedRotatedTime)
                {
                    //bIsOnceStop = false;
                    mNavigationCommand = Forward;
                    break;
                }
                mNavigationCommand = CounterClockwise;//Clockwise
                std::cout<<"****Drone is StraightRight => CounterClockwise Angle:\n"<<angleHeading<<std::endl;
                if(!bIsPrinted)
                {
                    logVisionPosition(countFrame,primarylane,"StraightRight&Stopped", "CounterClockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsPrinted = true;
                }
            }
            else
            {
                mNavigationCommand = Stop;
                gettimeofday(&timeRotateBegin, 0);
                estimatedRotatedTime = abs(angleHeading)/rotateSpeed;
                std::cout<<"********Drone is moving - Straight LEFT \n"<<std::endl;
                logVisionPosition(countFrame,primarylane,"StraightRight&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                bIsOnceStop = true;
            }
       }
    }
}
 void Vision::sendCMD2CurvedControl(LaneInfo primarylane, float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime){
    cout<<"**CURVED LINE control"<<endl;
    int delta = 0;
    delta = std::abs(primarylane.leftLine.intersection2Bottom.x - primarylane.rightLine.intersection2Bottom.x)/10;
    int thresholdLeft, thresholdRight;
    thresholdLeft   = primarylane.leftLine.intersection2Bottom.x + delta*2;
    thresholdRight  = primarylane.leftLine.intersection2Bottom.x + delta*8;
    
    timeStep = calculatePeriodOfTime(timeStepBegin);
    if(dronePos.x < thresholdLeft)
    {
        mNavigationCommand = Forward;
        std::cout<<"*******Near LEFT => go Forward\n"<<std::endl;
        logVisionPosition(countFrame,primarylane,"NearLEFT>Forward", "Forward", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    else if(dronePos.x > thresholdRight){
        if(bIsDroneStopped)
        {
            mNavigationCommand = CounterClockwise;//CounterClockwise
            std::cout<<"****RIGHT threshold & Drone is stopped => CounterClockwise Angle:\n"<<angleOfRoad<<std::endl;
            logVisionPosition(countFrame,primarylane,"RIGHT&Stopped", "CounterClockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
            bIsRotate = true;
        }
        else
        {
            mNavigationCommand = Stop;
            std::cout<<"********In lane & Drone is moving \n"<<std::endl;
            logVisionPosition(countFrame,primarylane,"RIGHT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
        }  
    }
    else
    {
        std::cout<<"****CENTER => Forward**********\n"<<std::endl;
        mNavigationCommand = Forward;
        logVisionPosition(countFrame,primarylane,"**CENTER**", "FORWARD", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    if(bIsRotate)
    {
        if(abs(angleOfRoad) < 15 )
        {
            std::cout<<"****ROTATE=> Forward**********\n"<<std::endl;
            mNavigationCommand = Forward;
            logVisionPosition(countFrame,primarylane,"*ROTATE*", "FORWARD", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
            bIsRotate = false;
        }
    }
 }
 void Vision::sendCMD2CurvedControl2(LaneInfo primarylane,float curvedRatio, float angleHeading, float angleOfRoad,cv::Point dronePos, double deltaTime){
    cout<<"**CURVED LINE control ver 2"<<endl;
    int minThreshold = 20;
    int maxThreshold = 45;
    int averageThreshold = (minThreshold + maxThreshold)/2;
    timeval timeRotateBegin; 
    bool bIsOnceStop = false,bIsPrinted = false;
    float rotateSpeed = 10; //10 deg/s
    double timeStep,estimatedRotatedTime;
    
    
    if(abs(angleOfRoad) < minThreshold)
    {
        mNavigationCommand = Forward;//Forward
        std::cout<<"****Drone is Curved Forward=> Forward Angle:\n"<<angleOfRoad<<std::endl;
        logVisionPosition(countFrame,primarylane,"InMinThreshold", "Forward", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
    }
    else if((angleOfRoad < minThreshold * -1 && angleOfRoad > maxThreshold * -1)|| angleOfRoad > minThreshold) // - 15 > angle > - 35 => CCW
    {
       while (true)
       {
            if(bIsOnceStop)
            {
                timeStep = calculatePeriodOfTime(timeRotateBegin);
                if(timeStep > estimatedRotatedTime)
                {
                   // bIsOnceStop = false;
                    mNavigationCommand = Forward;
                    break;
                }
                mNavigationCommand = CounterClockwise;//CounterClockwise
                std::cout<<"****Drone is CurvedLEFT => CounterClockwise Angle:\n"<<angleOfRoad<<std::endl;
                if(!bIsPrinted)
                {
                    logVisionPosition(countFrame,primarylane,"CurvedLEFT&Stopped", "CounterClockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsPrinted = true;
                }
            }
            else
            {
                mNavigationCommand = Stop;
                gettimeofday(&timeRotateBegin, 0);
                estimatedRotatedTime = abs(maxThreshold - angleOfRoad)/rotateSpeed;
                std::cout<<"********Drone is moving - Curved LEFT \n"<<std::endl;
                logVisionPosition(countFrame,primarylane,"CurvedLEFT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                bIsOnceStop = true;
            }
       }
    }
    else
    {
       while(true)
       {
            if(bIsOnceStop)
            {
                timeStep = calculatePeriodOfTime(timeRotateBegin);
                if(timeStep > estimatedRotatedTime)
                {
                    //bIsOnceStop = false;
                    mNavigationCommand = Forward;
                    break;
                }
                mNavigationCommand = Clockwise;//Clockwise
                std::cout<<"****Drone is Curved LEFT => Clockwise Angle:\n"<<angleOfRoad<<std::endl;
                if(!bIsPrinted)
                {
                    logVisionPosition(countFrame,primarylane,"CurvedLEFT&Stopped", "Clockwise", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                    bIsPrinted = true;
                }
            }
            else
            {
                mNavigationCommand = Stop;
                gettimeofday(&timeRotateBegin, 0);
                estimatedRotatedTime = (angleOfRoad - minThreshold)/rotateSpeed;
                std::cout<<"********Drone is moving - CurvedLEFT \n"<<std::endl;
                logVisionPosition(countFrame,primarylane,"CurvedLEFT&Moving", "Stop", curvedRatio, angleHeading, angleOfRoad,deltaTime, velocityX,velocityY);
                bIsOnceStop = true;
            }
       }
    }
   
 }
 
double Vision::calculatePeriodOfTime(timeval startTime) //return ms
{
	timeval currentTime;
	gettimeofday(&currentTime, 0);
	return (currentTime.tv_sec + 0.000001*currentTime.tv_usec - startTime.tv_sec - 0.000001*startTime.tv_usec)*1000;
}
bool Vision::logVisionProcessStep(int frameNum,double timeStep, bool isFrameDone, bool isFrameNum)
{	
    if(visionProcessFile == NULL)
    {
        const char* VISION_STEP_FILE_NAME = "/SaveData/VisionTime/visionTime.txt";
        visionProcessFile = fopen(VISION_STEP_FILE_NAME, "w");
        
        if(visionProcessFile == NULL)
        {
                std::cout << "Can not open file" << std::endl;
                return false;
        }
       
    }

    if(visionProcessFile != NULL)
    {	
        if(isFrameNum)
        {
            fprintf(visionProcessFile, "%d  ",frameNum);
            
        }
        else
        {
            if(!isFrameDone)
            {
                fprintf(visionProcessFile, "        %6f", timeStep);
            }
            else
                 fprintf(visionProcessFile, "\n");
        }
        
//        if(timeStep > 0)
//        {
//            fprintf(visionProcessFile, "    %f", timeStep);
//        }
//        else
//        {
//            fprintf(visionProcessFile, "    \n");
//            fprintf(visionProcessFile, "%d",frameNum);
//           
//        }
        fflush(visionProcessFile);
    }
    return true;
}
bool Vision::logVisionPosition(int frameNum,LaneInfo primarylane, std::string strPos, std::string navCMD,float curvedRatio, float angleHeading, float angleOfRoad, double autoTime, float velX, float velY)
{	
    if(visionPositionFile == NULL)
    {
        const char* VISION_POSITION_FILE_NAME = "/SaveData/VisionPosition/visionPosition.txt";
        visionPositionFile = fopen(VISION_POSITION_FILE_NAME, "w");
        
        if(visionPositionFile == NULL)
        {
                std::cout << "Can not open file visionPosition" << std::endl;
                return false;
        }
       
    }

    if(visionPositionFile != NULL)
    {		
        //                                                                                                          1.FrameNO   2. curvedRatio  3.AngleHeading    4.AngleOfRoad          5. TimeStep  6.Position&Command   7. Real CMD     8.VelocityX   9.VelocityY                      
         fprintf(visionPositionFile, "%d        %f      %f      %f      %f      %s      %s      %f      %f\n",frameNum,curvedRatio,angleHeading, angleOfRoad,autoTime, strPos.c_str(),navCMD.c_str(),velX,velY);
       fflush(visionPositionFile);
    }
    return true;
}
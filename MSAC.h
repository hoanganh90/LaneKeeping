#ifndef __MSAC_H__
#define __MSAC_H__

#include <opencv/cv.h>     
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include "Common.h"
#include "errorNIETO.h"
#include "LineSegment.h"
#include "LaneInfo.h"
#include <time.h>
#include <math.h>
#define MODE_LS		0
#define MODE_NIETO	1

class MSAC {
public:
	MSAC(void);
	~MSAC(void);

private:
	// Error Mode
	int __mode;	// Error mode (MODE_LS or MODE_NIETO)

	// Image info
	int __width;
	int __height;

	// RANSAC Options 
	float __epsilon;
	float __P_inlier;
	float __T_noise_squared;
	int __min_iters;
	int __max_iters;
	bool __reestimate;
	bool __verbose;
	bool __update_T_iter;
	bool __notify;
	// Parameters (precalculated)
	int __minimal_sample_set_dimension;	// Dimension of the MSS (minimal sample set)
	int __N_I_best;				// Number of inliers of the best Consensus Set
	float __J_best;				// Cost of the best Consensus Set
	std::vector<int> __MSS;			// Minimal sample set

	// Auxiliar variables
	cv::Mat __a, __an, __b, __bn, __li, __c;
	cv::Mat __vp, __vpAux;
	//cong.anh
	cv::Mat __lineResult;

	// Calibration
	cv::Mat __K;				// Approximated Camera calibration matrix

	// Data (Line Segments)	
	cv::Mat __Li;// Matrix of appended line segments (3xN) for N line segments
	cv::Mat __Mi;				// Matrix of middle points (3xN)
	cv::Mat __Lengths;			// Matrix of lengths (1xN)

	// Consensus set
	std::vector<int> __CS_idx, __CS_best;// Indexes of line segments: 1 -> belong to CS, 0 -> does not belong
	std::vector<int> __ind_CS_best;	// Vector of indexes of the Consensus Set
	double vp_length_ratio;
        
        bool isFindPrimaryLane;
        timeval timeStepBegin; 

public:

	/** Initialisation of MSAC procedure*/
	void init(int mode, cv::Size imSize, bool verbose = false);
	void videoLaneMatching(cv::Mat &Rep_ref, cv::Mat &Count_ref, cv::Mat &lines,
			int MaxLaneNum, int ExLaneNum, int TrackThreshold,
			int countUpperThresh);
	/** Main function which returns, if detected, several vanishing points and a vector of containers of line segments
	 corresponding to each Consensus Set.*/
        double calculatePeriodOfTime(timeval startTime);
	int multipleVPEstimation(
			std::vector<std::vector<cv::Point> > &lineSegments,
			std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters,
			std::vector<int> &numInliers, std::vector<cv::Mat> &vps, int numVps);
	//void KalmanFilter(cv::Point &vanishingPoint);
	/** Draws vanishing points and line segments according to the vanishing point they belong to*/
//cong.anh
	//void drawCS(cv::Mat &im, std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters, std::vector<cv::Mat> &vps);
	void drawCS(cv::Mat &im,
			std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters,
			std::vector<cv::Mat> &vps, cv::Point &bottomPoint,
			std::vector<std::vector<cv::Point> > &lineSegments,
			std::vector<LineSegment>&leftSegments,
			std::vector<LineSegment>&rightSegments, int leftindex,
			int rightIndex, int direction);
	void drawLaneKeeping(cv::Mat &im, std::vector<StoredLine> &acceptedLine,
			LaneInfo primaryLane, int pos);
	//cong.anh 2016.11.09
	void detectCenterLine(LineSegment tmpLine[2], LaneInfo &primaryLane);
        int detectPrimaryLane(std::vector<StoredLine> &leftLines,
			std::vector<StoredLine> &rightLines, cv::Point2f &centerBottom,
			cv::Point2f &centerTop,LaneInfo &primaryLane);
        void detectPrimaryLaneFirstFrame(std::vector<StoredLine> &sLines,
		cv::Point2f &bottomPoint, LaneInfo &primaryLane);
       void drawImage(cv::Mat image, std::vector<StoredLine> &leftLineROI1,
			std::vector<StoredLine> &rightLineROI1,
			std::vector<StoredLine> &lineROI2, cv::Point2f &C_0,
			cv::Point2f &C_1, cv::Point2f &C_2, cv::Point &vanishingPoint1,
			cv::Point &vanishingPoint2, double count,int roi2Height,double roi2Ratio, int numLineROI1, POSITION position);
	/*void laneMatching(std::vector<std::vector<cv::Point> >&currentLines,
	 std::vector<std::vector<cv::Point> >&storedLines,
	 std::vector<int>&counter, cv::Point &bottomPoint);*/
	void lineMatching(std::vector<StoredLine> &storedLines,
			std::vector<MatchedLine> &currentlines,
			cv::Point2f &bottomPoint);
	int errorCA(std::vector<LineSegment>&lineSegments, cv::Point &bottomPoint,
			std::vector<float> &E, bool dir);
	void leftRightClassification(std::vector<StoredLine> &storedLines,
			std::vector<StoredLine> &storedLeftLines,
			std::vector<StoredLine> &storedRightLines,
			cv::Point &vanishingPoint, cv::Point &bottomPoint);
	/*void distancePoint2line(std::vector<std::vector<cv::Point> >&lineSegments,
	 cv::Point &bottomPoint, std::vector<float> &E);*/
	void distancePoint2StoredLine(std::vector<StoredLine> &lineSegments,
		cv::Point2f &bp, cv::Point2f &topROI);
	void distancePoint2CurrentLine(std::vector<MatchedLine>&lineSegments,
			cv::Point2f &bp,  cv::Point2f &topROI);
	void distancePoint2line(std::vector<LineSegment>&lineSegments,
			cv::Point &bottomPoint, std::vector<float> &E);

	void matchingDistance(std::vector<StoredLine> storedLines,
			std::vector<MatchedLine> currentLines,
			std::vector<cv::Point2f> &Match_dis);

	void error2distanceList(std::vector<float> currentDistanceList,
			std::vector<float> storedDistanceList, std::vector<cv::Point2f> &E);
	bool intersection2Bottom(cv::Point2d &point1, cv::Point2d &point2,
			cv::Point2d &result);
	// Finds the intersection of two lines, or returns false.
	// The lines are defined by (o1, p1) and (o2, p2).
	bool intersection(cv::Point pt1_line1, cv::Point pt2_line1,
			cv::Point pt1_line2, cv::Point pt2_line2, cv::Point2f &r);
	void centerLineDetection(std::vector<StoredLine> &storedLines,
			LineSegment &centerLine, int LeftIndex, int rightIndex,
			cv::Point &bottomPoint);
	
	float angleBetween2Lines(cv::Point2f pt1_Line1, cv::Point2f pt2_Line1, cv::Point2f pt1_Line2, cv::Point2f pt2_Line2);
	double estmateAnglenPosition(LaneInfo primaryLane, cv::Point2f &bp, cv::Point2f &refPoint,
			POSITION &position);
        //cong.anh 2016.09.11
        void dilation(cv::Mat &src, cv::Mat &dst, int dilation_element,
		int dilation_size);
        double angleBetween2LinesAcos(cv::Point2f pt1_Line1, cv::Point2f pt2_Line1,
		cv::Point2f pt1_Line2, cv::Point2f pt2_Line2);
        double angleBetween2LinesAtan2(cv::Point pt1_Line1, cv::Point pt2_Line1,
		cv::Point pt1_Line2, cv::Point pt2_Line2);
private:
	/** This function returns a randomly selected MSS*/
	void GetMinimalSampleSet(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
			std::vector<int> &MSS, cv::Mat &vp);

	/** This function returns the Consensus Set for a given vanishing point and set of line segments*/
	float GetConsensusSet(int vpNum, cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
			cv::Mat &vEst, std::vector<float> &E, int *CS_counter);

	/** This is an auxiliar function that formats data into appropriate containers*/
	void fillDataContainers(std::vector<std::vector<cv::Point> > &lineSegments);

	// Estimation functions
	/** This function estimates the vanishing point for a given set of line segments using the Least-squares procedure*/
	void estimateLS(cv::Mat &Li, cv::Mat &Lengths, std::vector<int> &set,
			int set_length, cv::Mat &vEst);

	/** This function estimates the vanishing point for a given set of line segments using the Nieto's method*/
	void estimateNIETO(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
			std::vector<int> &set, int set_length, cv::Mat &vEst);

	// Error functions
	/** This function computes the residuals of the line segments given a vanishing point using the Least-squares method*/
	float errorLS(int vpNum, cv::Mat &Li, cv::Mat &vp, std::vector<float> &E,
			int *CS_counter);

	/** This function computes the residuals of the line segments given a vanishing point using the Nieto's method*/
	float errorNIETO(int vpNum, cv::Mat &Li, cv::Mat &lengthsLS, cv::Mat &Mi,
			cv::Mat &vp, std::vector<float> &E, int *CS_counter);

};

#endif // __MSAC_H__

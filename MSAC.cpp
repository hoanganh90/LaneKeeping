#include "MSAC.h"
#include "errorNIETO.h"
#include "lmmin.h"
#include "opencv2/core/core.hpp"
//cong.anh
#include <opencv2/opencv.hpp>
#include "opencv2/video/tracking.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <time.h>
#include "math.h"
#include "LineSegment.h"
#include "LaneInfo.h"
#include "Vision.h"
//#ifdef DEBUG_MAP	// if defined, a 2D map will be created (which slows down the process)

using namespace std;
using namespace cv;

MSAC::MSAC(void) {
	// Auxiliar variables
	__a = Mat(3, 1, CV_32F);
	__an = Mat(3, 1, CV_32F);
	__b = Mat(3, 1, CV_32F);
	__bn = Mat(3, 1, CV_32F);
	__li = Mat(3, 1, CV_32F);
	__c = Mat(3, 1, CV_32F);

	__vp = cv::Mat(3, 1, CV_32F);
	__vpAux = cv::Mat(3, 1, CV_32F);
}

MSAC::~MSAC(void) {
}
void MSAC::init(int mode, cv::Size imSize, bool verbose) {
	// Arguments
	__verbose = verbose;
	__width = imSize.width;
	__height = imSize.height;
	__verbose = verbose;
	__mode = mode;

	// MSAC parameters
	__epsilon = (float) 1e-6;
	__P_inlier = (float) 0.95;
	__T_noise_squared = (float) 0.001;
	__min_iters = 5;
	__max_iters = INT_MAX;
	__update_T_iter = false;

	// Parameters
	__minimal_sample_set_dimension = 2;

	// Minimal Sample Set
	for (int i = 0; i < __minimal_sample_set_dimension; ++i)
		__MSS.push_back(0);

	// (Default) Calibration	
	__K = Mat(3, 3, CV_32F);
	__K.setTo(0);
	__K.at<float>(0, 0) = (float) __width;
	__K.at<float>(0, 2) = (float) __width / 2;
	__K.at<float>(1, 1) = (float) __height;
	__K.at<float>(1, 2) = (float) __height / 2;
	__K.at<float>(2, 2) = (float) 1;

}

// COMPUTE VANISHING POINTS
void MSAC::fillDataContainers(
		std::vector<std::vector<cv::Point> > &lineSegments) {
	int numLines = lineSegments.size();
	if (__verbose)
		printf("Line segments: %d\n", numLines);

	// Transform all line segments
	// __Li = [l_00 l_01 l_02; l_10 l_11 l_12; l_20 l_21 l_22; ...]; where li=[l_i0;l_i1;l_i2]^T is li=an x bn; 
	__Li = Mat(numLines, 3, CV_32F);
	__Mi = Mat(numLines, 3, CV_32F);
	__Lengths = Mat(numLines, numLines, CV_32F);
	__Lengths.setTo(0);

	// Fill data containers (__Li, __Mi, __Lenghts)
	double sum_lengths = 0;
	for (int i = 0; i < numLines; i++) {
		// Extract the end-points					
		Point p1 = lineSegments[i][0];
		Point p2 = lineSegments[i][1];
		__a.at<float>(0, 0) = (float) p1.x; // x1
		__a.at<float>(1, 0) = (float) p1.y; // y1
		__a.at<float>(2, 0) = 1;
		__b.at<float>(0, 0) = (float) p2.x; // x2
		__b.at<float>(1, 0) = (float) p2.y; //y2
		__b.at<float>(2, 0) = 1;

		if (__mode == MODE_NIETO)
			__c = 0.5 * (__a + __b);
		//tinh chieu dai cua moi doan
		double length = sqrt(
				(__b.at<float>(0, 0) - __a.at<float>(0, 0))
						* (__b.at<float>(0, 0) - __a.at<float>(0, 0))
						+ (__b.at<float>(1, 0) - __a.at<float>(1, 0))
								* (__b.at<float>(1, 0) - __a.at<float>(1, 0)));
		sum_lengths += length;
		__Lengths.at<float>(i, i) = (float) length;
		//	printf("CA Line segment value: Lane: %d, %f \n",i,(float)__Lengths.at<float>(i,i));
		if (__mode == MODE_LS) {
			// Normalize into the sphere
			__an = __K.inv() * __a;
			__bn = __K.inv() * __b;
		} else // __mode == MODE_NIETO requires not to calibrate into the sphere
		{
			__an = __a;
			__bn = __b;
		}

		// Compute the general form of the line
		__li = __an.cross(__bn);
		cv::normalize(__li, __li);

		// Insert line into appended array
		__Li.at<float>(i, 0) = __li.at<float>(0, 0);
		__Li.at<float>(i, 1) = __li.at<float>(1, 0);
		__Li.at<float>(i, 2) = __li.at<float>(2, 0);

		if (__mode == MODE_NIETO) {
			// Store mid-Point too
			__Mi.at<float>(i, 0) = __c.at<float>(0, 0);
			__Mi.at<float>(i, 1) = __c.at<float>(1, 0);
			__Mi.at<float>(i, 2) = __c.at<float>(2, 0);
		}
	}
	__Lengths = __Lengths * ((double) 1 / sum_lengths);

}
/*
void MSAC::KalmanFilter(cv::Point &vanishingPoint) {
	//cv::KalmanFilter kf(4,2,0);
	Mat state(2, 1, CV_32F); //phi, delta_phi
	Mat processNoise(2, 1, CV_32F);
	Mat measure = Mat::zeros(1, 1, CV_32F);
	char code = (char) -1;
//	kf.transitionMatrix = *(Mat_<float>(4, 4) << 1,0,1,0,   0,1,0,1,  0,0,1,0,  0,0,0,1);
	cv::KalmanFilter kf(4, 2, 0);
	kf.transitionMatrix =
			(Mat_<float>(4, 4) << 1, 0, 1, 0, 0, 1, 0, 1, 0, 0, 1, 0, 0, 0, 0, 1);
	Mat_<float> measurement(2, 1);
	measurement.setTo(Scalar(0));
	kf.statePre.at<float>(0) = vanishingPoint.x;
	kf.statePre.at<float>(1) = vanishingPoint.y;
	kf.statePre.at<float>(2) = 0;
	kf.statePre.at<float>(3) = 0;
//Dont know
	setIdentity(kf.measurementMatrix);
	setIdentity(kf.processNoiseCov, Scalar::all(1e-4));
	setIdentity(kf.measurementNoiseCov, Scalar::all(10));
	setIdentity(kf.errorCovPost, Scalar::all(.1));

	vector<Point> mousev, kalmanv, aux;
	mousev.clear();
	kalmanv.clear();
	printf("CA: Kalman filter 2\n");
	//First predict, to update the internal statePre variable
	Mat prediction = kf.predict();
	Point predictPt(prediction.at<float>(0), prediction.at<float>(1));

	//get VP position
	measurement(0) = vanishingPoint.x;
	measurement(1) = vanishingPoint.y;

	//the update phase
	printf("CA: Kalman filter 3\n");
	Mat estimated = kf.correct(measure);
	Point statePt(estimated.at<float>(0), estimated.at<float>(1));
	Point measPt(measurement(0), measurement(1));

	mousev.push_back(measPt);
	kalmanv.push_back(statePt);
	printf("CA: Kalman filter 4\n");
	for (int i = 0; i < kalmanv.size(); i++) {
		std::cout << "Kalman point: " << "[" << i << "]" << kalmanv[i] << endl;
	}

}
*/
//cong.anh 2016.09.11
void MSAC::dilation(cv::Mat &src, cv::Mat &dst, int dilation_element,
		int dilation_size) {
    //cout << "***Dilation****" << endl;
	int dilation_type;
	if (dilation_element == 0)
		dilation_type = MORPH_RECT;
	else if (dilation_element == 1)
		dilation_type = MORPH_ELLIPSE;
	Mat element = getStructuringElement(dilation_type,
			Size(2 * dilation_size + 1, 2 * dilation_size + 1),
			Point(dilation_size, dilation_size));
	dilate(src, dst, element);

}
double MSAC::angleBetween2LinesAtan2(cv::Point pt1_Line1, cv::Point pt2_Line1,
		cv::Point pt1_Line2, cv::Point pt2_Line2) {
	cv::Point2d vectorA, vectorB;
	vectorA = pt1_Line2 - pt1_Line1;
	vectorB = pt2_Line2 - pt2_Line1;
	double cross, dot, angle;
	cross = vectorA.x * vectorB.y - vectorA.y * vectorB.x;
	dot = vectorA.x * vectorB.x + vectorA.y * vectorB.y;
	angle = atan2(cross, dot);
	// atan2(y, x) or atan2(sin, cos)
	return angle/3.14*180;
}
double MSAC::angleBetween2LinesAcos(cv::Point2f pt1_Line1, cv::Point2f pt2_Line1,
		cv::Point2f pt1_Line2, cv::Point2f pt2_Line2) {
	cv::Point2f leftVector, rightVector;
	leftVector.x = pt1_Line1.x - pt2_Line1.x;
	leftVector.y = pt1_Line1.y - pt2_Line1.y;
	rightVector.x = pt1_Line2.x - pt2_Line2.x;
	rightVector.y = pt1_Line2.y - pt2_Line2.y;

	float len1 = sqrt(
			leftVector.x * leftVector.x + leftVector.y * leftVector.y);
	float len2 = sqrt(
			rightVector.x * rightVector.x + rightVector.y * rightVector.y);

	float dot = leftVector.x * rightVector.x + leftVector.y * rightVector.y;

	float a = dot / (len1 * len2);
	if (a >= 1)
		return 0;
	else if (a <= -1)
		return M_PI;
	else
		return acos(a);
	//return value is angle in rad
}
double MSAC::calculatePeriodOfTime(timeval startTime) //return ms
{
	timeval currentTime;
	gettimeofday(&currentTime, 0);
	return (currentTime.tv_sec + 0.000001*currentTime.tv_usec - startTime.tv_sec - 0.000001*startTime.tv_usec)*1000;
}
int MSAC::multipleVPEstimation(
		std::vector<std::vector<cv::Point> > &lineSegments,
		std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters,
		std::vector<int> &numInliers, std::vector<cv::Mat> &vps, int numVps) {
// Make a copy of lineSegments because it is modified in the code (it will be restored at the end of this function)
	std::vector<std::vector<cv::Point> > lineSegmentsCopy = lineSegments;
	std::vector<std::vector<cv::Point> > lineSegmentsCurrent;
// Loop over maximum number of vanishing points
	int number_of_inliers = 0;
	for (int vpNum = 0; vpNum < numVps; vpNum++) {
		// Fill data structures
		fillDataContainers(lineSegments);
		int numLines = lineSegments.size();
		if (numLines == 0)
			return -1;
		//printf("CA: multipleVPEstimation - Number of lines: %d\n", numLines);
		if (__verbose)
			printf("VP %d-----\n", vpNum);

		// Break if the number of elements is lower than minimal sample set
		if (numLines < 3 || numLines < __minimal_sample_set_dimension) {
			if (__verbose)
				printf("Not enough line segments to compute vanishing point\n");
			break;
		}

		// Vector containing indexes for current vp
		std::vector<int> ind_CS;

		__N_I_best = __minimal_sample_set_dimension; //2 // Number of inliers of the best Consensus Set
		__J_best = FLT_MAX;	// Cost of the best Consensus Set

		int iter = 0;
		int T_iter = INT_MAX;
		int no_updates = 0;
		int max_no_updates = INT_MAX;

		// Define containers of CS (Consensus set): __CS_best to store the best one, and __CS_idx to evaluate a new candidate
		__CS_best = vector<int>(numLines, 0);
		__CS_idx = vector<int>(numLines, 0);

		// Allocate Error matrix
		vector<float> E = vector<float>(numLines, 0);

		// MSAC
		if (__verbose) {
			if (__mode == MODE_LS)
				printf("Method: Calibrated Least Squares\n");
			if (__mode == MODE_NIETO)
				printf("Method: Nieto\n");

			//printf("Start MSAC\n");
		}
		/*printf("Start RANSAC\n");*/
		//int count = 0;
		// RANSAC loop
                timeval timeStepBegin;
                gettimeofday(&timeStepBegin, 0);
		while ((iter <= __min_iters)
				|| ((iter <= T_iter) && (iter <= __max_iters)
						&& (no_updates <= max_no_updates))) {

                      //  printf("Anh RANSAC: %d iteration\n", iter);
			iter++;

			if (iter >= __max_iters)
				break;
                        timeStep = calculatePeriodOfTime(timeStepBegin);
                        if(timeStep >=3000)
                            return -1;
			// Hypothesize ------------------------
			// Select MSS
			if (__Li.rows < (int) __MSS.size())
				break;
			GetMinimalSampleSet(__Li, __Lengths, __Mi, __MSS, __vpAux);	// output __vpAux is calibrated

			// Test --------------------------------
			// Find the consensus set and cost
			int N_I = 0;
			float J = GetConsensusSet(vpNum, __Li, __Lengths, __Mi, __vpAux, E,
					&N_I);		// the CS is indexed in CS_idx

			// Update ------------------------------
			// If the new cost is better than the best one, update
			if (N_I >= __minimal_sample_set_dimension && (J < __J_best)
					|| ((J == __J_best) && (N_I > __N_I_best))) {
				//std::cout << "Find a better value\n";
				__notify = true;

				__J_best = J;
				__CS_best = __CS_idx;

				__vp = __vpAux;	// Store into __vp (current best hypothesis): __vp is therefore calibrated

				if (N_I > __N_I_best)
					__update_T_iter = true;

				__N_I_best = N_I;

				if (__update_T_iter) {
					// Update number of iterations
					double q = 0;
					if (__minimal_sample_set_dimension > __N_I_best) {
						// Error!
						perror(
								"The number of inliers must be higher than minimal sample set");
					}
					if (numLines == __N_I_best) {
						q = 1;
					} else {
						q = 1;
						for (int j = 0; j < __minimal_sample_set_dimension; j++)
							q *= (double) (__N_I_best - j)
									/ (double) (numLines - j);
					}
					// Estimate the number of iterations for RANSAC
					if ((1 - q) > 1e-12) {
						T_iter = (int) ceil(
								log((double) __epsilon)
										/ log((double) (1 - q)));
//						std::cout
//								<< "Estimate the number of iterations for RANSAC T_iter:"
//								<< T_iter << endl;
					} else
						T_iter = 0;
				}
			} else {
				__notify = false;
				//	std::cout << "Not good enough\n";
			}
			// Verbose
			if (__verbose && __notify) {
				int aux = max(T_iter, __min_iters);
				printf("Iteration = %5d/%9d. ", iter, aux);
				printf("Inliers = %6d/%6d (cost is J = %8.4f)\n", __N_I_best,
						numLines, __J_best);

				if (__verbose)
					printf("MSS Cal.VP = (%.3f,%.3f,%.3f)\n",
							__vp.at<float>(0, 0), __vp.at<float>(1, 0),
							__vp.at<float>(2, 0));
			}

			// Check CS length (for the case all line segments are in the CS)
			if (__N_I_best == numLines) {
				if (__verbose)
					printf(
							"All line segments are inliers. End MSAC at iteration %d.\n",
							iter);
				break;
			}
		}

		// Reestimate ------------------------------
		if (__verbose) {
			printf("Number of iterations: %d\n", iter);
			printf("Final number of inliers = %d/%d\n", __N_I_best, numLines);
		}

		// Fill ind_CS with __CS_best
		//cong.anh get the best consume set
		//	std::vector<std::vector<cv::Point> > lineSegmentsCurrent;
		for (int i = 0; i < numLines; i++) {
			if (__CS_best[i] == vpNum) {
				int a = i;
				ind_CS.push_back(a);
				lineSegmentsCurrent.push_back(lineSegments[i]);

			}
		}

		if (__J_best > 0
				&& ind_CS.size()
						> (unsigned int) __minimal_sample_set_dimension) // if J==0 maybe its because all line segments are perfectly parallel and the vanishing point is at the infinity
						{
			if (__verbose) {
				printf("Reestimating the solution... ");
				fflush(stdout);
			}
			if (__mode == MODE_LS)
				estimateLS(__Li, __Lengths, ind_CS, __N_I_best, __vp);
			else if (__mode == MODE_NIETO) //default method (more loops)
				estimateNIETO(__Li, __Lengths, __Mi, ind_CS, __N_I_best, __vp);	// Output __vp is calibrated
			else
				perror(
						"ERROR: mode not supported, please use {LS, LIEB, NIETO}\n");

			if (__verbose)
				printf("done!\n");

			// Uncalibrate
			if (__verbose)
				printf("Cal.VP = (%.3f,%.3f,%.3f)\n", __vp.at<float>(0, 0),
						__vp.at<float>(1, 0), __vp.at<float>(2, 0));
			//std::cout << "cong.anh 1: come here 1 __K = \n" << __K << endl;
			__vp = __K * __vp;
			if (__vp.at<float>(2, 0) != 0) {
				//std::cout << "cong.anh come here 2\n"; //=> it was un-calibrated??? => calibrate
				__vp.at<float>(0, 0) /= __vp.at<float>(2, 0);
				__vp.at<float>(1, 0) /= __vp.at<float>(2, 0);
				__vp.at<float>(2, 0) = 1;
			} else {
				//std::cout << "cong.anh come here 3\n";
				// Since this is infinite, it is better to leave it calibrated
				__vp = __K.inv() * __vp;
			}
			if (__verbose)
				printf("VP = (%.3f,%.3f,%.3f)\n", __vp.at<float>(0, 0),
						__vp.at<float>(1, 0), __vp.at<float>(2, 0));
			// Copy to output vector
			vps.push_back(__vp);
		} else if (fabs(__J_best - 1) < 0.000001) {
			if (__verbose) {
				printf(
						"The cost of the best MSS is 0! No need to reestimate\n");
				printf("Cal. VP = (%.3f,%.3f,%.3f)\n", __vp.at<float>(0, 0),
						__vp.at<float>(1, 0), __vp.at<float>(2, 0));
			}

			// Uncalibrate
			__vp = __K * __vp;
			if (__vp.at<float>(2, 0) != 0) {
				__vp.at<float>(0, 0) /= __vp.at<float>(2, 0);
				__vp.at<float>(1, 0) /= __vp.at<float>(2, 0);
				__vp.at<float>(2, 0) = 1;

				if (__verbose)
					printf("VP = (%.3f,%.3f,%.3f)\n", __vp.at<float>(0, 0),
							__vp.at<float>(1, 0), __vp.at<float>(2, 0));
			} else {
				// Calibrate
				__vp = __K.inv() * __vp;
			}
			// Copy to output vector
				std::cout << "CA __vp=" << __vp << endl;			//ko vao dau
			vps.push_back(__vp);
		}

		// Fill lineSegmentsClusters containing the indexes of inliers for current vps
		if (__N_I_best > 2) {
			while (ind_CS.size()) {
				lineSegments.erase(
						lineSegments.begin() + ind_CS[ind_CS.size() - 1]);
				ind_CS.pop_back();
			}
		}

		// Fill current CS
		lineSegmentsClusters.push_back(lineSegmentsCurrent);
		/*	for (int i = 0; i < lineSegmentsCurrent.size(); i++) {
		 std::cout << "lineSegmentsCurrent[" << i << "]: "
		 << lineSegmentsCurrent[i] << endl;
		 }*/
		// Fill numInliers
		numInliers.push_back(__N_I_best);
	}

//__Li.at<float>(__MSS[0],0);
//__Li = __K * __Li;

// Restore lineSegments
	lineSegments = lineSegmentsCopy;
        
}
// RANSAC
void MSAC::GetMinimalSampleSet(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
		std::vector<int> &MSS, cv::Mat &vp) {
	int N = Li.rows;

// Generate a pair of samples
	while (N <= (MSS[0] = rand() / (RAND_MAX / (N - 1))))
		; //cong.anh get a random pair value
	while (N <= (MSS[1] = rand() / (RAND_MAX / (N - 1))))
		;
	//std::cout << "Mss[0] = " << MSS[0] << " Mss[1]= " << MSS[1] << "\n" << endl;
//std::cout << "CA GetMinimalSampleSet\n";
// Estimate the vanishing point and the residual error
	if (__mode == MODE_LS)
		estimateLS(Li, Lengths, MSS, 2, vp);
	else if (__mode == MODE_NIETO)
		estimateNIETO(Li, Mi, Lengths, MSS, 2, vp);
	else
		perror("ERROR: mode not supported. Please use {LS, LIEB, NIETO}\n");
}

float MSAC::GetConsensusSet(int vpNum, cv::Mat &Li, cv::Mat &Lengths,
		cv::Mat &Mi, cv::Mat &vp, std::vector<float> &E, int *CS_counter) {
// Compute the error of each line segment of LSS with respect to v_est
// If it is less than the threshold, add to the CS
	for (unsigned int i = 0; i < __CS_idx.size(); i++)
		__CS_idx[i] = -1;

	float J = 0;

	if (__mode == MODE_LS)
		J = errorLS(vpNum, Li, vp, E, CS_counter);
	else if (__mode == MODE_NIETO)
		J = errorNIETO(vpNum, Li, Lengths, Mi, vp, E, CS_counter);
	else
		perror("ERROR: mode not supported, please use {LS, LIEB, NIETO}\n");

	return J;
}

// Estimation functions
void MSAC::estimateLS(cv::Mat &Li, cv::Mat &Lengths, std::vector<int> &set,
		int set_length, cv::Mat &vp) {
	//printf("Estimate the vanishing point for a given set of line segments\n");
	if (set_length == __minimal_sample_set_dimension) //__minimal_sample_set_dimension = 2
			{
		// Just the cross product
		//printf("CA DATA IS CALIBRATED in MODE_LS\n");
		cv::Mat ls0 = Mat(3, 1, CV_32F);
		cv::Mat ls1 = Mat(3, 1, CV_32F);

		ls0.at<float>(0, 0) = Li.at<float>(set[0], 0);
		ls0.at<float>(1, 0) = Li.at<float>(set[0], 1);
		ls0.at<float>(2, 0) = Li.at<float>(set[0], 2);

		ls1.at<float>(0, 0) = Li.at<float>(set[1], 0);
		ls1.at<float>(1, 0) = Li.at<float>(set[1], 1);
		ls1.at<float>(2, 0) = Li.at<float>(set[1], 2);

		vp = ls0.cross(ls1); //intersaction

		cv::normalize(vp, vp);

		return;
	} else if (set_length < __minimal_sample_set_dimension) {
		perror("Error: at least 2 line-segments are required\n");
		return;
	}

// Extract the line segments corresponding to the indexes contained in the set
	cv::Mat li_set = Mat(3, set_length, CV_32F);
	cv::Mat Lengths_set = Mat(set_length, set_length, CV_32F);
	Lengths_set.setTo(0);
//cong.anh
	int count_li_set = 0;
// Fill line segments info
	for (int i = 0; i < set_length; i++) {
		li_set.at<float>(0, i) = Li.at<float>(set[i], 0);
		li_set.at<float>(1, i) = Li.at<float>(set[i], 1);
		li_set.at<float>(2, i) = Li.at<float>(set[i], 2);
		count_li_set++;
		Lengths_set.at<float>(i, i) = Lengths.at<float>(set[i], set[i]);
	}
	//printf(" Least squares solution \n");
// Least squares solution
// Generate the matrix ATA (a partir de LSS_set=A)
	cv::Mat L = li_set.t();
	cv::Mat Tau = Lengths_set;
	cv::Mat ATA = Mat(3, 3, CV_32F);
	ATA = L.t() * Tau.t() * Tau * L;
	//printf("L size = %d\n", count_li_set);
// Obtain eigen decomposition
	cv::Mat w, v, vt;
	cv::SVD::compute(ATA, w, v, vt);
	//printf("CA v_rows = %d v_col = %d\n", v.rows, v.cols);
// Check eigenvecs after SVDecomp
	if (v.rows < 3)
		return;

// print v, w, vt...
//std::cout << "w=" << w << endl;
//std::cout << "v=" << v << endl;
//std::cout << "vt" << vt << endl;

// Assign the result (the last column of v, corresponding to the eigenvector with lowest eigenvalue)
	vp.at<float>(0, 0) = v.at<float>(0, 2);
	vp.at<float>(1, 0) = v.at<float>(1, 2);
	vp.at<float>(2, 0) = v.at<float>(2, 2);

	cv::normalize(vp, vp);
	//std::cout << "CA vp=" << vp << endl;
	return;
}
void MSAC::estimateNIETO(cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
		std::vector<int> &set, int set_length, cv::Mat &vp) {
	printf(
			"Anh: Estimates the vanishing point for a given set of line segments using the Nieto's method\n");
	if (set_length == __minimal_sample_set_dimension) {
		// Just the cross product
		// DATA IS NOT CALIBRATED for MODE_NIETO
		cv::Mat ls0 = Mat(3, 1, CV_32F);
		cv::Mat ls1 = Mat(3, 1, CV_32F);

		ls0.at<float>(0, 0) = Li.at<float>(set[0], 0);
		ls0.at<float>(1, 0) = Li.at<float>(set[0], 1);
		ls0.at<float>(2, 0) = Li.at<float>(set[0], 2);

		ls1.at<float>(0, 0) = Li.at<float>(set[1], 0);
		ls1.at<float>(1, 0) = Li.at<float>(set[1], 1);
		ls1.at<float>(2, 0) = Li.at<float>(set[1], 2);

		vp = ls0.cross(ls1);

		// Calibrate (and normalize) vp
		vp = __K.inv() * vp;
		cv::normalize(vp, vp);
		return;
	} else if (set_length < __minimal_sample_set_dimension) {
		perror("Error: at least 2 line-segments are required\n");
		return;
	}

// Extract the line segments corresponding to the indexes contained in the set
	cv::Mat li_set = Mat(3, set_length, CV_32F);
	cv::Mat Lengths_set = Mat(set_length, set_length, CV_32F);
	cv::Mat mi_set = Mat(3, set_length, CV_32F);
	Lengths_set.setTo(0);

// Fill line segments info
	for (int i = 0; i < set_length; i++) {
		li_set.at<float>(0, i) = Li.at<float>(set[i], 0);
		li_set.at<float>(1, i) = Li.at<float>(set[i], 1);
		li_set.at<float>(2, i) = Li.at<float>(set[i], 2);

		Lengths_set.at<float>(i, i) = Lengths.at<float>(set[i], set[i]);

		mi_set.at<float>(0, i) = Mi.at<float>(set[i], 0);
		mi_set.at<float>(1, i) = Mi.at<float>(set[i], 1);
		mi_set.at<float>(2, i) = Mi.at<float>(set[i], 2);
	}

#ifdef DEBUG_MAP
	double dtheta = 0.01;
	double dphi = 0.01;

	int numTheta = (int)CV_PI/(2*dtheta);
	int numPhi = (int)2*CV_PI/dphi;
	cv::Mat debugMap(numTheta, numPhi, CV_32F);
	debugMap.setTo(0);

	data_struct dataTest(li_set, Lengths_set, mi_set, __K);
	double *fvecTest = new double[set_length];
	int *infoTest;
	int aux = 0;
	infoTest = &aux;

// Image limits
	cv::Mat pt0 = Mat(3,1,CV_32F);
	cv::Mat pt3 = Mat(3,1,CV_32F);
	pt0.at<float>(0,0) = 0; pt0.at<float>(1,0) = 0; pt0.at<float>(2,0) = 1;
	pt3.at<float>(0,0) = __width; pt3.at<float>(1,0) = __height; pt3.at<float>(2,0) = 1;

	cv::Mat pt0C = __K.inv()*pt0; cv::normalize(pt0C, pt0C);
	cv::Mat pt3C = __K.inv()*pt3; cv::normalize(pt3C, pt3C);

	double theta0 = acos(pt0C.at<float>(2,0));
	double phi0 = atan2(pt0C.at<float>(1,0), pt0C.at<float>(0,0));
	printf("\nPt0(sph): (%.2f, %.2f)\n", theta0, phi0);

	double theta3 = acos(pt3C.at<float>(2,0));
	double phi3 = atan2(pt3C.at<float>(1,0), pt3C.at<float>(0,0));
	printf("Pt3(sph): (%.2f, %.2f)\n", theta3, phi3);

	double paramTest [] = {0, 0};
	double maxE = 0, minE = FLT_MAX;
	for(int t=0; t<numTheta; t++)
	{
		double theta = dtheta*t;
		for(int p=0; p<numPhi; p++)
		{
			double phi = dphi*p - CV_PI;
			paramTest[0] = theta;
			paramTest[1] = phi;

			evaluateNieto(paramTest, set_length, (const void*)&dataTest, fvecTest, infoTest);

			for(int m=0; m<set_length; m++)
			debugMap.at<float>(t,p) += fvecTest[m];

			if(debugMap.at<float>(t,p) < minE)
			minE = debugMap.at<float>(t,p);
			else if(debugMap.at<float>(t,p) > maxE)
			maxE = debugMap.at<float>(t,p);
		}
	}
	cv::Mat debugMapIm(numTheta, numPhi, CV_8UC1);
	double scale = 255/(maxE-minE);

	cv::convertScaleAbs(debugMap, debugMapIm, scale);

	delete[] fvecTest;

	cv::imshow("DebugMap", debugMapIm);
	cv::waitKey(0);

#endif

// Lev.-Marq. solution
	int m_dat = set_length;
//int num_par = 3;
	int num_par = 2;

// The starting point is the provided vp which is already calibrated
	if (__verbose) {
		printf("\nInitial Cal.VP = (%.3f,%.3f,%.3f)\n", vp.at<float>(0, 0),
				vp.at<float>(1, 0), vp.at<float>(2, 0));
		cv::Mat vpUnc = Mat(3, 1, CV_32F);
		vpUnc = __K * vp;
		if (vpUnc.at<float>(2, 0) != 0) {
			vpUnc.at<float>(0, 0) /= vpUnc.at<float>(2, 0);
			vpUnc.at<float>(1, 0) /= vpUnc.at<float>(2, 0);
			vpUnc.at<float>(2, 0) = 1;
		}
		printf("Initial VP = (%.3f,%.3f,%.3f)\n", vpUnc.at<float>(0, 0),
				vpUnc.at<float>(1, 0), vpUnc.at<float>(2, 0));
	}

// Convert to spherical coordinates to move on the sphere surface (restricted to r=1)
	double x = (double) vp.at<float>(0, 0);
	double y = (double) vp.at<float>(1, 0);
	double z = (double) vp.at<float>(2, 0);
	double r = cv::norm(vp);
	double theta = acos(z / r);
	double phi = atan2(y, x);

	if (__verbose)
		printf("Initial Cal.VP (Spherical) = (%.3f,%.3f,%.3f)\n", theta, phi,
				r);

//double par[] = {(double)vp.at<float>(0,0), (double)vp.at<float>(1,0), (double)vp.at<float>(2,0)};
	double par[] = { theta, phi };

	lm_control_struct control = lm_control_double;
	control.epsilon = 1e-5;	// less than 1�
	if (__verbose)
		control.printflags = 2; //monitor status (+1) and parameters (+2), (4): residues at end of fit, (8): residuals at each step
	else
		control.printflags = 0;
	lm_status_struct status;
	data_struct data(li_set, Lengths_set, mi_set, __K);

	/*lmmin(num_par, par, m_dat, &data, evaluateNieto, &control, &status,
			lm_printout_std);*/

	if (__verbose)
		printf("Converged Cal.VP (Spherical) = (%.3f,%.3f,%.3f)\n", par[0],
				par[1], r);

// Store into vp
// 1) From spherical to cartesian
	theta = par[0];
	phi = par[1];
	x = r * cos(phi) * sin(theta);
	y = r * sin(phi) * sin(theta);
	z = r * cos(theta);

	vp.at<float>(0, 0) = (float) x;
	vp.at<float>(1, 0) = (float) y;
	vp.at<float>(2, 0) = (float) z;

}
// Error functions
float MSAC::errorLS(int vpNum, cv::Mat &Li, cv::Mat &vp, std::vector<float> &E,
		int *CS_counter) {
	cv::Mat vn = vp;
	double vn_norm = cv::norm(vn);

	cv::Mat li;
	li = Mat(3, 1, CV_32F);
	double li_norm = 0;
	float di = 0;

	float J = 0;
	for (int i = 0; i < Li.rows; i++) {
		li.at<float>(0, 0) = Li.at<float>(i, 0);
		li.at<float>(1, 0) = Li.at<float>(i, 1);
		li.at<float>(2, 0) = Li.at<float>(i, 2);
		//ca
		//	std::cout<<"CA li = "<<li<<endl;
		//	std::cout<<" vn "<<vn<<endl;
		li_norm = cv::norm(li); // esto lo podria precalcular antes  I could precompute this before
		//	std::cout<<" li_norm "<<li_norm;
		di = (float) vn.dot(li);
		//	std::cout<<" di \n"<<di<<endl;
		di /= (float) (vn_norm * li_norm);//khoang cach tu 1 diem den 1 duong thang

		E[i] = di * di;

		/* Add to CS if error is less than expected noise */
		if (E[i] <= __T_noise_squared) {
			__CS_idx[i] = vpNum;		// set index to 1
			(*CS_counter)++;

			// Torr method
			J += E[i];
		} else {
			J += __T_noise_squared;
		}
	}

	J /= (*CS_counter);

	return J;
}

//cong.anh errorCA(std::vector<std::vector<cv::Point> >&lineSegments,std::vector<cv::Point> &bottomPoint, std::vector<float> &E);
void MSAC::lineMatching(std::vector<StoredLine> &storedLines,
		std::vector<MatchedLine> &currentlines,
		cv::Point2f &bottomPoint) {

	cv::Point bp = bottomPoint;
	//Match_dis: store the minimal error between current&store distance list
	std::vector<cv::Point2f> Match_dis;
	int Threshold = 20;
	int FrameFound = 5;
	int FrameLost = 5;
	int CountUpperThresh = 10;
	//cout<<"laneMatching\n"<<endl;

	//match absolute distance between 2 list
	matchingDistance(storedLines, currentlines, Match_dis);
	//Compare with threshold
	for (int i = 0; i < Match_dis.size(); i++) {

            if (Match_dis[i].x <= Threshold) {// found a acceptable line => update the stored lines, count ++

                    int currentIndex = Match_dis[i].y;// get the index of the acceptable line in current lines
                    storedLines[i].Count++;
                    storedLines[i].point1 = currentlines[currentIndex].point1;
                    storedLines[i].point2 = currentlines[currentIndex].point2;
                    storedLines[i].distance = currentlines[currentIndex].distance;
                    storedLines[i].intersection2Bottom = currentlines[currentIndex].intersection2Bottom;
                    storedLines[i].intersection2Middle = currentlines[currentIndex].intersection2Middle;
                    storedLines[i].bIsUpdated = true;

            } else if (Match_dis[i].x > Threshold || Match_dis[i].x != 10000) {

                    storedLines[i].Count--;
                    storedLines[i].bIsUpdated = true;
                    //	printf(" update: i =%d, storedLine.IsPrimary = %d , currentIndex = %d\n",i, storedLines[i].bIsPrimary,currentIndex);

            }
            if (storedLines[i].Count >= CountUpperThresh) {
                    storedLines[i].Count = CountUpperThresh;
            } else if (storedLines[i].Count < 0)
                    storedLines[i].Count = 0;
	}
        //cout<<"After Comparing threshold\n"<<endl;
	for (int i = 0; i < currentlines.size(); i++) {

            if (currentlines[i].isMatched == false) {
                    //Stored lines
                    StoredLine sline;
                    int lastItem = storedLines.size();
                    sline.ID = lastItem;		//????
                    sline.setPoint(currentlines[i].point1, currentlines[i].point2);
                    sline.bIsUpdated = true;
                    sline.Count = 1;
                    sline.distance = currentlines[i].distance;
                    sline.intersection2Bottom = currentlines[i].intersection2Bottom;
                    sline.intersection2Middle = currentlines[i].intersection2Middle;
                    storedLines.push_back(sline);
            }
	}
        //cout<<"Update store list\n"<<endl;
	for (int i = storedLines.size(); i >= 0; i--) {
		if (storedLines[i].Count == 0) {
			storedLines.erase(storedLines.begin() + i);
		}
	}

}
void MSAC::leftRightClassification(std::vector<StoredLine> &storedLines,
		std::vector<StoredLine> &storedLeftLines,
		std::vector<StoredLine> &storedRightLines, cv::Point &vanishingPoint,
		cv::Point &bottomPoint) {/*
		 vector<cv::Point> aux;
		 for (int i = 0; i < storedLines.size(); i++) {

		 if (((storedLines[i].point1.x > vanishingPoint.x)
		 && (storedLines[i].point2.x > bottomPoint.x))
		 || ((storedLines[i].point2.x > vanishingPoint.x)
		 && (storedLines[i].point1.x > bottomPoint.x))) {
		 aux.clear();
		 aux.push_back(storedLines[i].point1);
		 aux.push_back(storedLines[i].point2);
		 storedRightLines.push_back(aux);
		 } else if (((storedLines[i].point1.x < vanishingPoint.x)
		 && (storedLines[i].point2.x < bottomPoint.x))
		 || ((storedLines[i].point2.x < vanishingPoint.x)
		 && (storedLines[i].point1.x < bottomPoint.x))) {
		 aux.clear();
		 aux.push_back(storedLines[i].point1);
		 aux.push_back(storedLines[i].point2);
		 storedLeftLines.push_back(aux);

		 }

		 }*/
}
/*
 * errorCA
 * 1. find intersection from line to the bottom
 * 2. find the distance from bottom point to the intersection
 * Output:THe index of the item which has the shortest distance from center bottom point to line segments
 */
int MSAC::errorCA(std::vector<LineSegment>&lineSegments, cv::Point &bottomPoint,
		std::vector<float> &E, bool dir) {
	cv::Point bp = bottomPoint;

	float minValue = 10000, minIndex = 0;
	for (int i = 0; i < lineSegments.size(); i++) {

		cv::Point pt1, pt2, bottomY;
                cv::Point2f r;
		bottomY.x = 0;
		bottomY.y = bottomPoint.y;
		double dx, dy;
		pt1.x = lineSegments[i].point1.x;	// diem dau va diem cuoi cua 1 doan
		pt1.y = lineSegments[i].point1.y;
		pt2.x = lineSegments[i].point2.x;
		pt2.y = lineSegments[i].point2.y;

		float param = -1;
		//cong.anh compute the distance from a point to a line
		/*float A = bp.x - pt1.x;
		 float B = bp.y - pt1.y;
		 float C = pt1.x - pt2.x;
		 float D = pt2.y - pt1.y;

		 float dot = B * C + A * D;
		 float length = C * C + D * D;


		 if (length != 0) //in case of 0 length line
		 param = cv::abs(dot) / cv::sqrt(length);*/

		intersection(pt1, pt2, bottomPoint, bottomY, r);
		if ((dir == true && r.x >= bottomPoint.x)
				|| (dir == false && r.x <= bottomPoint.x)) {
			dx = r.x - bottomPoint.x;
			dy = r.y - bottomPoint.y;

			E[i] = sqrt(dx * dx + dy * dy);
			if (E[i] < minValue) {
				minIndex = i;
				minValue = E[i];
			}
		}

	}
	return minIndex;

}
/*
 void MSAC::distancePoint2line(std::vector<std::vector<cv::Point> >&lineSegments,
 cv::Point &bp, std::vector<float> &E) {
 //cong.anh compute the distance from a point to a line
 cv::Point pt1, pt2;
 //vector<float> E = vector<float>(lineSegments.size(), 0);
 for (int i = 0; i < lineSegments.size(); i++) {
 pt1.x = lineSegments[i][0].x;		// diem dau va diem cuoi cua 1 doan
 pt1.y = lineSegments[i][0].y;
 pt2.x = lineSegments[i][1].x;
 pt2.y = lineSegments[i][1].y;

 float A = bp.x - pt1.x;
 float B = bp.y - pt1.y;
 float C = pt2.x - pt1.x;
 float D = pt2.y - pt1.y;

 float dot = A * C + B * D;
 float length = C * C + D * D;
 float param = -1;
 if (length != 0) //in case of 0 length line
 param = cv::abs(dot) / cv::sqrt(length);
 float xx, yy;
 if (param < 0) {
 xx = pt1.x;
 yy = pt1.y;
 } else if (param > 1) {
 xx = pt2.x;
 yy = pt2.y;
 } else {
 xx = pt1.x + param * C;
 yy = pt1.y + param * C;
 }
 float dx = bp.x - xx;
 float dy = bp.y - yy;
 E[i] = cv::sqrt(dx * dx + dy * dy);
 }
 }*/

void MSAC::distancePoint2line(std::vector<LineSegment> &lineSegments,
		cv::Point &bp, std::vector<float> &E) {
	//cong.anh compute the distance from a point to a line
	cv::Point pt1, pt2;
	//vector<float> E = vector<float>(lineSegments.size(), 0);
	for (int i = 0; i < lineSegments.size(); i++) {
		pt1 = lineSegments[i].point1;		// diem dau va diem cuoi cua 1 doan
		pt2 = lineSegments[i].point2;
		float A = bp.x - pt1.x;
		float B = bp.y - pt1.y;
		float C = pt1.x - pt2.x;
		float D = pt2.y - pt1.y;

		float dot = B * C + A * D;
		float length = C * C + D * D;
		float param = -1;
		if (length != 0) //in case of 0 length line
			E[i] = cv::abs(dot) / cv::sqrt(length);
	}
}
void MSAC::distancePoint2CurrentLine(std::vector<MatchedLine> &lineSegments,
		cv::Point2f &bp,  cv::Point2f &topROI) {
	//cong.anh compute the distance from a point to a line
	cv::Point pt1, pt2;
	//vector<float> E = vector<float>(lineSegments.size(), 0);
	bool b;
	for (int i = 0; i < lineSegments.size(); i++) {
		pt1 = lineSegments[i].point1;		// diem dau va diem cuoi cua 1 doan
		pt2 = lineSegments[i].point2;
		float A = bp.x - pt1.x;
		float B = bp.y - pt1.y;
		float C = pt1.x - pt2.x;
		float D = pt2.y - pt1.y;

		float dot = B * C + A * D;
		float length = C * C + D * D;
		float param = -1;
		if (length != 0) //in case of 0 length line
			lineSegments[i].distance = cv::abs(dot) / cv::sqrt(length);
		b = intersection(pt1, pt2, bp, cv::Point(0, bp.y),
				lineSegments[i].intersection2Bottom);
                b = intersection(pt1, pt2, topROI, cv::Point(0, topROI.y),
				lineSegments[i].intersection2Middle);

		lineSegments[i].distance = abs(
				bp.x - lineSegments[i].intersection2Bottom.x);
	}
}
void MSAC::distancePoint2StoredLine(std::vector<StoredLine> &lineSegments,
		cv::Point2f &bp, cv::Point2f &topROI) {
	//cong.anh compute the distance from a point to a line
	cv::Point pt1, pt2;
	bool b;
	for (int i = 0; i < lineSegments.size(); i++) {
            pt1 = lineSegments[i].point1;		// diem dau va diem cuoi cua 1 doan
            pt2 = lineSegments[i].point2;
            /*float A = bp.x - pt1.x;
             float B = bp.y - pt1.y;
             float C = pt1.x - pt2.x;
             float D = pt2.y - pt1.y;

             float dot = B * C + A * D;
             float length = C * C + D * D;
             float param = -1;
             if (length != 0) //in case of 0 length line
             lineSegments[i].distance = cv::abs(dot) / cv::sqrt(length);*/

            b = intersection(pt1, pt2, bp, cv::Point(0, bp.y),
                            lineSegments[i].intersection2Bottom);
            b = intersection(pt1, pt2, topROI, cv::Point(0, topROI.y),
                            lineSegments[i].intersection2Middle);

            lineSegments[i].distance = abs(
                            bp.x - lineSegments[i].intersection2Bottom.x);	
	}
}


void MSAC::matchingDistance(std::vector<StoredLine> storedLines,
		std::vector<MatchedLine> currentLines,
		std::vector<cv::Point2f> &Match_dis) {
	//cout<<"matchingDistance - sLine.size ="<<storedLines.size()<<" cLines.size = "<<currentLines.size() <<endl;
	//Init a vector which store the difference between 2 distance

	float E[currentLines.size()];
	float delta[currentLines.size()];
	/*
	 * Matching
	 */
	for (int i = 0; i < storedLines.size(); i++) {
            float minValue = 10000;
            float minDistance = 70;
            int minIndex = 0;

            for (int j = 0; j < currentLines.size(); j++) {

                if (currentLines[j].isMatched == false) {
                    //Distance between 2 lines in the bottom
                  //  cout<<"Come here check matching"<<endl;
                    delta[j] = abs(storedLines[i].intersection2Bottom.x - currentLines[j].intersection2Bottom.x);
                    //Error of distance to the center-bottom point between 2 lines
                    E[j] =  abs(storedLines[i].distance - currentLines[j].distance);
//                    	printf("[%d] E[%d] = %f delta[%d] = %d\n", i, j, E[j], j,
//                     delta[j]);
                     
                    if (E[j] < minValue && delta[j] < minDistance) {
                            minIndex = j;
                            minValue = E[j];
                    }
                  //  cout<<" Matching Distance minIndex: "<<minIndex<<" , minValue = "<< minValue<<endl;
                }
            }
            currentLines[minIndex].isMatched = true;
            Point2f p;
            p.x = minValue; // Minimun value
            p.y = minIndex; // index of currentLine which match to the storedline
            Match_dis.push_back(p);

	}
	/*for (int i = 0; i < Match_dis.size(); i++) {
	 printf("Match_dis[%d]:  MinVal %f MinInd = %f\n", i, Match_dis[i].x,
	 Match_dis[i].y);
	 }*/
}
/*
 * Input : a List of distance from bottom point to current lines(currentDistanceList)+ a List of distance from bottom point to stored line(storedDistanceList)
 * TODO: From each element in storedDistanceList, find the minimal difference with every elements in currentDistanceList
 * Match_dis[i] = min(|storedDistanceList - currentDistanceList|)
 */

void MSAC::error2distanceList(std::vector<float> currentDistanceList,
		std::vector<float> storedDistanceList,
		std::vector<cv::Point2f> &Match_dis) {
	printf("come here  error2distanceList\n");

//Init a vector which store the difference between 2 distance
	float E[currentDistanceList.size()];
	for (int i = 0; i < storedDistanceList.size(); i++) {
		float minValue = 10000;
		float minIndex = currentDistanceList.size();
		for (int j = currentDistanceList.size() - 1; j != minIndex; j--) {
			E[j] = cv::abs(storedDistanceList[i] - currentDistanceList[j]);
			if (E[j] < minValue) {
				minIndex = j;
				minValue = E[j];
			}
		}
		Point p;
		p.x = minValue; // Minimun value
		p.y = minIndex; // index of currentLine which match to the storedline
		Match_dis.push_back(p);
	}

}
bool intersection2Bottom(cv::Point2d &point1, cv::Point2d &point2,
		cv::Point2d &result) {
	int x1, y1, x2, y2;
	float m1, m2, c1, c2;
	x1 = point1.x;
	y1 = point1.y;
	x2 = point2.x;
	y2 = point2.y;

	cv::Point bottomPoint1, bottomPoint2;
//	cv::Point2d intersection;
	bottomPoint1.x = 0;
	bottomPoint1.y = 10;

	bottomPoint2.x = 320;
	bottomPoint2.y = 10;

	float dx, dy;

//Line 1
	dx = x2 - x1;
	dy = y2 - y1;
	m1 = dy / dx;
	c1 = y1 - m1 * x1; //intercep c = y - mx

//Line 2
	x1 = bottomPoint1.x;
	y1 = bottomPoint1.y;
	x2 = bottomPoint2.x;
	y2 = bottomPoint2.y;

	dx = x2 - x1;
	dy = y2 - y1;
	m2 = dy / dx;
	c2 = y2 - m2 * x2; // which is same as y2 - slope * x2
	if (m1 - m2 == 0) {
		std::cout << "No intersection between 2 lines";
		return false;
	} else {
		result.x = (c2 - c1) / (m1 - m2);
		result.y = m1 * result.x + c1;
	}
	return true;
}
bool MSAC::intersection(cv::Point pt1_line1, cv::Point pt2_line1,
		cv::Point pt1_line2, cv::Point pt2_line2, cv::Point2f &r) {
	cv::Point x = pt1_line2 - pt1_line1;
	cv::Point d1 = pt2_line1 - pt1_line1;
	cv::Point d2 = pt2_line2 - pt1_line2;

	float cross = d1.x * d2.y - d1.y * d2.x;
	if (abs(cross) < /*EPS*/1e-8)
		return false;

	double t1 = (x.x * d2.y - x.y * d2.x) / cross;
	r = pt1_line1 + d1 * t1;
	return true;
}
float MSAC::errorNIETO(int vpNum, cv::Mat &Li, cv::Mat &Lengths, cv::Mat &Mi,
		cv::Mat &vp, std::vector<float> &E, int *CS_counter) {
	float J = 0;
	float di = 0;

	cv::Mat lineSegment = Mat(3, 1, CV_32F);
	float lengthLineSegment = 0;
	cv::Mat midPoint = Mat(3, 1, CV_32F);

// The vp arrives here calibrated, need to uncalibrate (check it anyway)
	cv::Mat vn = cv::Mat(3, 1, CV_32F);
	double vpNorm = cv::norm(vp);
	if (fabs(vpNorm - 1) < 0.001) {
		// Calibrated -> uncalibrate
		vn = __K * vp;
		if (vn.at<float>(2, 0) != 0) {
			vn.at<float>(0, 0) /= vn.at<float>(2, 0);
			vn.at<float>(1, 0) /= vn.at<float>(2, 0);
			vn.at<float>(2, 0) = 1;
		}
	}

	for (int i = 0; i < Li.rows; i++) {
		lineSegment.at<float>(0, 0) = Li.at<float>(i, 0);
		lineSegment.at<float>(1, 0) = Li.at<float>(i, 1);
		lineSegment.at<float>(2, 0) = Li.at<float>(i, 1);

		lengthLineSegment = Lengths.at<float>(i, i);

		midPoint.at<float>(0, 0) = Mi.at<float>(i, 0);
		midPoint.at<float>(1, 0) = Mi.at<float>(i, 1);
		midPoint.at<float>(2, 0) = Mi.at<float>(i, 2);

		//di = distanceNieto(vn, lineSegment, lengthLineSegment, midPoint);

		E[i] = di * di;

		/* Add to CS if error is less than expected noise */
		if (E[i] <= __T_noise_squared) {
			__CS_idx[i] = vpNum;		// set index to 1
			(*CS_counter)++;

			// Torr method
			J += E[i];
		} else {
			J += __T_noise_squared;
		}

		J += E[i];
	}

	J /= (*CS_counter);

	return J;
}
/*
 * centerLineDetection:
 * 1.Identify the list of lanes
 * 2. Locate the center line of the primary lane
 * 3. Calculate the angle between the
 */
void MSAC::centerLineDetection(std::vector<StoredLine> &storedLines,
		LineSegment &centerLine, int LeftIndex, int rightIndex,
		cv::Point &bottomPoint) {
	bool b1, b2, b3, b4;
	//cv::Point interBottom1, interMiddle1, interBottom2, interMiddle2;
	cv::Point middlePoint1, middlePoint2, bottomPoint1;
	cv::Point2f middleLeft, middleRight, bottomLeft, bottomRight;

	middlePoint1.x = 0;
	middlePoint1.y = bottomPoint.y / 2;
	middlePoint2.x = bottomPoint.x;
	middlePoint2.y = bottomPoint.y / 2;
	bottomPoint1.x = 0;
	bottomPoint1.y = bottomPoint.y;

	b1 = intersection(storedLines[LeftIndex].point1,
			storedLines[LeftIndex].point2, middlePoint1, middlePoint2,
			middleLeft);
	b2 = intersection(storedLines[LeftIndex].point1,
			storedLines[LeftIndex].point2, bottomPoint1, bottomPoint,
			bottomLeft);

	b3 = intersection(storedLines[rightIndex].point1,
			storedLines[rightIndex].point2, middlePoint1, middlePoint2,
			middleRight);
	b4 = intersection(storedLines[rightIndex].point1,
			storedLines[rightIndex].point2, bottomPoint1, bottomPoint,
			bottomRight);

	centerLine.point1 = (middleLeft + middleRight) / 2;

	centerLine.point2 = (bottomLeft + bottomRight) / 2;
}
void MSAC::drawCS(cv::Mat &im,
		std::vector<std::vector<std::vector<cv::Point> > > &lineSegmentsClusters,
		std::vector<cv::Mat> &vps, cv::Point &bottomPoint,
		std::vector<std::vector<cv::Point> > &lineSegments,
		std::vector<LineSegment>&leftSegments,
		std::vector<LineSegment>&rightSegments, int leftindex, int rightIndex,
		int direction) {
	vector<cv::Scalar> colors;
	colors.push_back(cv::Scalar(0, 0, 255)); // First is RED
	colors.push_back(cv::Scalar(0, 255, 0)); // Second is GREEN
	colors.push_back(cv::Scalar(255, 0, 0)); // Third is BLUE

	cv::Point pt1_line1, pt2_line1, pt1_line2, pt2_line2, point1_middleLine,
			point2_middleLine, point1_bottomline, point2_bottomLine;
	cv::Point intersaction1_middle, intersaction2_middle, intersaction1_bottom,
			intersaction2_bottom;

	point1_middleLine.x = 0;
	point1_middleLine.y = im.size().height / 2;

	point2_middleLine.x = im.size().width;
	point2_middleLine.y = im.size().height / 2;

	point1_bottomline.x = 0;
	point1_bottomline.y = im.size().height;

	point2_bottomLine.x = im.size().width;
	point2_bottomLine.y = im.size().height;

	pt1_line1 = leftSegments[leftindex].point1; // Point 1 leftLine1
	pt2_line1 = leftSegments[leftindex].point2; //Point 2 leftline2

	pt1_line2 = rightSegments[rightIndex].point1; // Point 1 rightLine 2
	pt2_line2 = rightSegments[rightIndex].point2; // Point 2 rightLine 2

	//bool point1, point2, point3, point4;


	Point pts[4] = { intersaction1_middle, intersaction1_bottom,
			intersaction2_bottom, intersaction2_middle };
	/*	std::cout << "Left intersection middle" << intersaction1_middle
	 << " Left intersaction bottom " << intersaction1_bottom
	 << "Right intersaction middle " << intersaction2_middle
	 << "Right intersaction bottom " << intersaction2_bottom << endl;*/

// Paint vps
	for (unsigned int vpNum = 0; vpNum < vps.size(); vpNum++) {
		if (vps[vpNum].at<float>(2, 0) != 0) {
			Point2f vp(vps[vpNum].at<float>(0, 0),
					vps[vpNum].at<float>(1, 0) + im.rows / 2);

			// Paint vp if inside the image
			if (vp.x >= 0 && vp.x < im.cols && vp.y >= 0 && vp.y < im.rows) {
				//cv2.circle(im,vp,4,colors[vpNum],2);
				//cv2.circle(im, vp, 3, CV_RGB(0,0,0), -1);
				circle(im, vp, 4, colors[vpNum], 2);
				circle(im, vp, 3, CV_RGB(0, 0, 0), -1);

			}
		}
	}
// Paint line segments
	/*for (unsigned int c = 0; c < lineSegmentsClusters.size(); c++) {
	 for (unsigned int i = 0; i < lineSegmentsClusters[c].size(); i++) {
	 Point pt1 = lineSegmentsClusters[c][i][0];
	 Point pt2 = lineSegmentsClusters[c][i][1];

	 line(im, pt1, pt2, colors[0], 1);
	 }
	 }
	 */
	/*	//Paint Left segment
	 for (unsigned int i = 0; i < leftSegments.size(); i++) {
	 Point pt1 = leftSegments[i][0];
	 Point pt2 = leftSegments[i][1];

	 line(im, pt1, pt2, CV_RGB(255, 217, 0), 1);
	 }
	 //Paint Right segments
	 for (unsigned int i = 0; i < rightSegments.size(); i++) {
	 Point pt1 = rightSegments[i][0];
	 Point pt2 = rightSegments[i][1];

	 line(im, pt1, pt2, CV_RGB(255, 0, 0), 1);
	 }*/
	/*Point pt1x = lineSegments[__MSS[0]][0];
	 Point pt1y = lineSegments[__MSS[0]][1];

	 Point pt2x = lineSegments[__MSS[1]][0];
	 Point pt2y = lineSegments[__MSS[1]][1];*/

	Point pt1x = leftSegments[leftindex].point1; // Point 1 Line 1
	Point pt1y = leftSegments[leftindex].point2; // Point 2 Line 1

	Point pt2x = rightSegments[rightIndex].point1; // Point 1 Line 2
	Point pt2y = rightSegments[rightIndex].point2; // Point 2 Line 2

//Paint Left and Right

	cv::fillConvexPoly(im, pts, 4, Scalar(0, 255, 0));
	Point vp;
	vp.x = vps[0].at<float>(0, 0);
	vp.y = vps[0].at<float>(1, 0) + im.rows / 2;

	cv::line(im, pt1x, pt1y, (0, 255, 0), 5);
	cv::line(im, pt2x, pt2y, (0, 255, 0), 5);
	cv::line(im, vp, bottomPoint, (0, 255, 0), 5);
//	std: cout << "Print line:[0] " << "__MSS[0] = " << __MSS[0] << " value "
//			<< lineSegments[__MSS[0]] << "\n Print line:[1] " << "__MSS[1] = "
//			<< __MSS[1] << lineSegments[__MSS[1]] << "\n" << endl;

	if (direction == 1) {

		cv::putText(im, "Right", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(0, 200, 200));
	} else if (direction == 2) {
		cv::putText(im, "Left", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(0, 200, 200));
	} else {
		cv::putText(im, "Forward", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(0, 200, 200));
	}

}
void MSAC::drawLaneKeeping(cv::Mat &im, std::vector<StoredLine> &acceptedLine,
		LaneInfo primaryLane, int pos) {
	//all accept lanes
	for (int i = 0; i < acceptedLine.size(); i++) {
		Point pt1 = acceptedLine[i].point1;
		Point pt2 = acceptedLine[i].point2;

		line(im, pt1, pt2, CV_RGB(255, 0, 0), 2); //red
	}

	// Draw the main lane
	printf("drawLaneKeeping\n");
	Point p1_leftLane, p2_leftLane, p1_rightLane, p2_rightLane, p1_centerLine, p2_centerLine;
	p1_leftLane = primaryLane.leftLine.intersection2Middle;
	p2_leftLane = primaryLane.leftLine.intersection2Bottom;

	p1_rightLane = primaryLane.rightLine.intersection2Middle;
	p2_rightLane = primaryLane.rightLine.intersection2Bottom;

	p1_centerLine = primaryLane.centerLine.intersection2Middle;
	p2_centerLine = primaryLane.centerLine.intersection2Bottom;

	// primary lane
	//line(im, p1_leftLane, p2_leftLane, CV_RGB(255, 217, 0), 2); // yellow
	//line(im, p1_rightLane, p2_rightLane, CV_RGB(255, 217, 0), 2); //yellow

	// center line
	//line(im, p1_centerLine, p2_centerLine, CV_RGB(0, 255, 0), 2);
	/*
	 * Position
	 */
	if (pos == 1) {

		cv::putText(im, "Left", Point(50, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(19, 17, 17));
	} else if (pos == 2) {
		cv::putText(im, "Right", Point(370, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(19, 17, 17));
	} else {
		cv::putText(im, "Forward", Point(320, 100), FONT_HERSHEY_SIMPLEX, 1,
				Scalar(19, 17, 17));
	}
}
void MSAC::detectPrimaryLaneFirstFrame(std::vector<StoredLine> &sLines,
		cv::Point2f &bottomPoint, LaneInfo &primaryLane) {
	//Detect left&right lane

	float minLeft = 10000, minIndexLeft = 0;
	float minRight = 10000, minIndexRight = 0;
	for (int i = 0; i < sLines.size(); i++) {
		sLines[i].bIsPrimary = false;

		if (sLines[i].intersection2Bottom.x > bottomPoint.x) {
			/*LineSegment aux;
			 aux.setPoint(results[i].point1, results[i].point2);
			 rightSegments.push_back(aux);*/
			if (sLines[i].distance < minLeft) {
				minIndexLeft = i;
				minLeft = sLines[i].distance;
			}
		} else if (sLines[i].intersection2Bottom.x < bottomPoint.x) {
			/*	LineSegment aux;
			 aux.setPoint(results[i].point1, results[i].point2);
			 leftSegments.push_back(aux);*/
			if (sLines[i].distance < minRight) {
				minIndexRight = i;
				minRight = sLines[i].distance;
			}
		}

	}
	/*
	 * Tracking the primary lane's condition
	 */
	int delta = cv::abs(
			sLines[minIndexLeft].intersection2Bottom.x
					- sLines[minIndexRight].intersection2Bottom.x);
	if (delta > 30) {
		sLines[minIndexLeft].bIsPrimary = true;
		sLines[minIndexRight].bIsPrimary = true;
              
                isFindPrimaryLane = true;
	}
}
int MSAC::detectPrimaryLane(std::vector<StoredLine> &leftLines,
		std::vector<StoredLine> &rightLines, cv::Point2f &centerBottom,
		cv::Point2f &centerTop, LaneInfo &primaryLane) {
	//Primary lane
	// Left side: leftLines[0]
	// Right side: rightLines[0]
	/*
	 * Tracking the primary lane's condition
	 */
	float delta = abs(leftLines[0].intersection2Bottom.x
					- rightLines[0].intersection2Bottom.x);
	if (delta > 20) {
		leftLines[0].bIsPrimary = true;
		rightLines[0].bIsPrimary = true;
                primaryLane.leftLine.point1 = leftLines[0].point1;
                primaryLane.leftLine.point2 = leftLines[0].point2;
                primaryLane.leftLine.intersection2Bottom = leftLines[0].intersection2Bottom;
                primaryLane.leftLine.intersection2Middle = leftLines[0].intersection2Middle;
                
                primaryLane.rightLine.point1 = rightLines[0].point1;
                primaryLane.rightLine.point1 = rightLines[0].point2;
                primaryLane.rightLine.intersection2Bottom = rightLines[0].intersection2Bottom;
                primaryLane.rightLine.intersection2Middle = rightLines[0].intersection2Middle;
                centerBottom = (leftLines[0].intersection2Bottom
			+ rightLines[0].intersection2Bottom) / 2;
                centerTop = (leftLines[0].intersection2Middle
			+ rightLines[0].intersection2Middle) / 2;
                primaryLane.centerLine.intersection2Bottom = centerBottom;
                primaryLane.centerLine.intersection2Middle = centerTop;
                cout<<"Primary lane Left"<<primaryLane.leftLine.intersection2Bottom<<" Right: "<<primaryLane.rightLine.intersection2Bottom<<endl;
                
	} else
		return -1;
        return 0;
	
}
void MSAC::detectCenterLine(LineSegment tmpLine[2], LaneInfo &primaryLane) {
//	printf("CA detectCenterLine: intesec1 = %d , intersec2 = %d\n",
//			tmpLine[0].intersection2Bottom.x, tmpLine[1].intersection2Bottom.x);
	if (tmpLine[0].intersection2Bottom.x >= tmpLine[1].intersection2Bottom.x) {
		//printf("come here 1\n");
		//tmpLine[0] is Right, tmpLine[1] is Left
		primaryLane.setLine(tmpLine[1], tmpLine[0]);
	} else {
		//tmpLine[0] is Left, tmpLine[1] is Right
		//printf("come here 2\n");
		primaryLane.setLine(tmpLine[0], tmpLine[1]);
	}
//	cout << "Primary lane left.x " << primaryLane.leftLine.intersection2Bottom.x
//			<< " rightBottom = " << primaryLane.rightLine.intersection2Bottom.x
//			<< endl;
	//centerLine - middle lane
	primaryLane.centerLine.intersection2Middle.x =
			(tmpLine[0].intersection2Middle.x + tmpLine[1].intersection2Middle.x)
					/ 2;
	primaryLane.centerLine.intersection2Middle.y =
			(tmpLine[0].intersection2Middle.y + tmpLine[1].intersection2Middle.y)
					/ 2;

	//centerline - bottom lane
	primaryLane.centerLine.intersection2Bottom.x =
			(tmpLine[0].intersection2Bottom.x + tmpLine[1].intersection2Bottom.x)
					/ 2;
	primaryLane.centerLine.intersection2Bottom.y =
			(tmpLine[0].intersection2Bottom.y + tmpLine[1].intersection2Bottom.y)
					/ 2;
	intersection(primaryLane.leftLine.intersection2Bottom,primaryLane.leftLine.intersection2Middle, primaryLane.rightLine.intersection2Bottom,primaryLane.rightLine.intersection2Middle,primaryLane.intersecLeftRight);

}
void MSAC::drawImage(cv::Mat image, std::vector<StoredLine> &leftLineROI1,std::vector<StoredLine> &rightLineROI1,std::vector<StoredLine> &lineROI2, cv::Point2f &C_0, cv::Point2f &C_1,	cv::Point2f &C_2, cv::Point2f &vanishingPoint1,cv::Point2f &vanishingPoint2, double count, int roi2Height,double roi2Ratio, int numLineROI1, POSITION position) {
	int height;
	height = image.rows / 2;
	cv::Point2f centerCircle;
	float radius;
        String posStr;
	// Draw lineROI1
	for (unsigned int c = 0; c < leftLineROI1.size(); c++) {
		cv::Point pt1, pt2;
		pt1.y = leftLineROI1[c].point1.y + roi2Height;
		pt2.y = leftLineROI1[c].point2.y + roi2Height;
		pt1.x = leftLineROI1[c].point1.x;
		pt2.x = leftLineROI1[c].point2.x;
		line(image, pt1, pt2, CV_RGB(0, 0, 0), 2);
	}
	//Draw the Right lines of ROI1
	for (unsigned int c = 0; c < rightLineROI1.size(); c++) {
		cv::Point pt1, pt2;
		pt1.y = rightLineROI1[c].point1.y + roi2Height;
		pt2.y = rightLineROI1[c].point2.y + roi2Height;
		pt1.x = rightLineROI1[c].point1.x;
		pt2.x = rightLineROI1[c].point2.x;
		line(image, pt1, pt2, CV_RGB(0, 0, 0), 2);
	}
	//Draw lineROi2
//	for (unsigned int c = 0; c < lineROI2.size(); c++) {
//		Point pt1 = lineROI2[c].point1;
//		Point pt2 = lineROI2[c].point2;
//		line(image, pt1, pt2, CV_RGB(0, 0, 0), 2);
//	}
	//Draw C_0 - C_1 - C_2
	C_0.y = C_0.y + roi2Height;
	C_1.y = C_1.y + roi2Height;
	line(image, C_0, C_1, CV_RGB(0, 0, 0), 2);
	line(image, C_1, C_2, CV_RGB(0, 0, 0), 2);

	//Draw vanishing points VP1
	vanishingPoint1.y = vanishingPoint1.y + image.rows / 2;
	if (vanishingPoint1.x >= 0 && vanishingPoint1.x < image.cols
			&& vanishingPoint1.y >= 0 && vanishingPoint1.y < image.rows) {
		//cv2.circle(im,vp,4,colors[vpNum],2);
		//cv2.circle(im, vp, 3, CV_RGB(0,0,0), -1);
		circle(image, vanishingPoint1, 4, CV_RGB(255, 255, 255), 2);
		circle(image, vanishingPoint1, 3, CV_RGB(0, 0, 0), -1);
		cv::putText(image, "VP1", vanishingPoint1,
		CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1, 8, false);

	}
	//Draw vanishing points Vp2
	if (vanishingPoint2.x >= 0 && vanishingPoint2.x < image.cols
			&& vanishingPoint2.y >= 0 && vanishingPoint2.y < image.rows) {
		//cv2.circle(im,vp,4,colors[vpNum],2);
		//cv2.circle(im, vp, 3, CV_RGB(0,0,0), -1);
		circle(image, vanishingPoint2, 4, CV_RGB(255, 255, 255), 2);
		circle(image, vanishingPoint2, 3, CV_RGB(0, 0, 0), -1);
		cv::putText(image, "VP2", vanishingPoint2,
		CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1, 8, false);
	}
	cout << "Index: " << (int) count << " C_0 : " << C_0 << " C_1: " << C_1
			<< " C_2: " << C_2 << " delta: " << roi2Ratio << " VP1: "
			<< vanishingPoint1 << " VP2: " << vanishingPoint2 << endl;
	std::string roi2RatioStr = "Delta " + std::to_string(roi2Ratio);

	std::string VP1PosStr = "Vanishing point 1: ("
			+ std::to_string(vanishingPoint1.x) + ", "
			+ std::to_string(vanishingPoint1.y) + ")";
	std::string VP2PosStr = "Vanishing point 2: ("
			+ std::to_string(vanishingPoint2.x) + ", "
			+ std::to_string(vanishingPoint2.y) + ")";
	std::string C0PosStr = "C0: (" + std::to_string(C_0.x) + ", "
			+ std::to_string(C_0.y) + ")";
	std::string C1PosStr = "C1: (" + std::to_string(C_1.x) + ", "
			+ std::to_string(C_1.y) + ")";
	std::string C2PosStr = "C2: (" + std::to_string(C_2.x) + ", "
			+ std::to_string(C_2.y) + ")";

	std::string ROI1PosStr = "ROI1: (Height = " + std::to_string(roi2Height)
			+ ")";
	std::string ROI2PosStr = "ROI2: (Height = "
			+ std::to_string(image.rows - roi2Height) + ")";

	std::string numberlineROI1 = "Detected lines = "
						+ std::to_string(numLineROI1);
	std::string numberFilterlineROI1 = "After filter: "
				+ std::to_string(leftLineROI1.size() + rightLineROI1.size());

	std::string numberlineROI2 = "Detected lines = "
					+ std::to_string(lineROI2.size());

        if(position == 0)
            posStr = "In Lane";
        else if(position == 1)
            posStr = "OUT LEFT";
        else
            posStr = "OUT RIGHT";
        std::string posDroneStr = "POSITION = "+ posStr;
	//Show delta
	cv::putText(image, roi2RatioStr, cv::Point(100, 150),
	CV_FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255), 1, 8, false);
	//Show VP1 position
	cv::putText(image, VP1PosStr, cv::Point(100, 170), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	//Show VP2 position
	cv::putText(image, VP2PosStr, cv::Point(100, 190), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	//Show C_0 position
	cv::putText(image, C0PosStr, cv::Point(100, 210), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	//Show C_1 position
	cv::putText(image, C1PosStr, cv::Point(100, 230), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	//Show C_2 position
	cv::putText(image, C2PosStr, cv::Point(100, 250), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);

	//ROI1
	cv::putText(image, ROI1PosStr, cv::Point(450, 170), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	cv::putText(image, numberlineROI1, cv::Point(450, 190), CV_FONT_HERSHEY_SIMPLEX,
				0.5, cv::Scalar(255), 1, 8, false);
	cv::putText(image, numberFilterlineROI1, cv::Point(450, 210), CV_FONT_HERSHEY_SIMPLEX,
					0.5, cv::Scalar(255), 1, 8, false);
	//ROI2
	cv::putText(image, ROI2PosStr, cv::Point(450, 50), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	cv::putText(image, numberlineROI2, cv::Point(450, 70), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
        //Drone's Position
        cv::putText(image, posDroneStr, cv::Point(320, 300), CV_FONT_HERSHEY_SIMPLEX,
			0.5, cv::Scalar(255), 1, 8, false);
	//2016.11.12 cong.anh: Draw eclipse
	//Calculate R and centerCircle :refer: http://www.regentsprep.org/regents/math/geometry/gcg6/RCir.htm
	if (!image.data) {
		cout << "ERROR: Cannot read a frame" << endl;
		return;
	} else {
		char imageFileName[128];
		snprintf(imageFileName, sizeof(imageFileName),
				"SaveData/VisionResult/ImageResult%04d.jpg", (int) count);
		imwrite(imageFileName, image);
//                imshow("Output", image);
//                char q = (char) waitKey(1);
//
//		if (q == 27) {
//			printf("\nStopped by user request\n");
//			return;
//		}
	}
}
float MSAC::angleBetween2Lines(cv::Point2f pt1_Line1, cv::Point2f pt2_Line1, cv::Point2f pt1_Line2, cv::Point2f pt2_Line2)
{
	cv::Point2f leftVector, rightVector;
	leftVector.x = pt1_Line1.x - pt2_Line1.x;
	leftVector.y = pt1_Line1.y - pt2_Line1.y;
	rightVector.x = pt1_Line2.x - pt2_Line2.x;
	rightVector.y = pt1_Line2.y - pt2_Line2.y;

	float len1 = sqrt(leftVector.x * leftVector.x + leftVector.y*leftVector.y);
	float len2 = sqrt(rightVector.x * rightVector.x + rightVector.y*rightVector.y);

	float dot = leftVector.x * rightVector.x + leftVector.y * rightVector.y;

	float a = dot/(len1*len2);
	if(a>= 1)
		return 0;
	else if(a<=-1)
		return M_PI;
	else
		return acos(a);
        //return value is angle in rad
}

float MSAC::estmateAnglenPosition(LaneInfo primaryLane, cv::Point2f &bp, cv::Point2f &refPoint,
		POSITION &pos) {
	/*
	 * Angle
	 */
	cv::Point2f tmpBp = cv::Point2f(bp.x, 0);
	float angle = angleBetween2Lines(bp,refPoint,bp,tmpBp);
        if(bp.x < primaryLane.centerLine.intersection2Bottom.x)
        {
            angle = angle * -1;
        }
	/*
	 * Position
	 */
	
	if (bp.x < primaryLane.leftLine.intersection2Bottom.x) {
		pos = Left_Lane;	// on the Left of the main lane
	} else if (bp.x > primaryLane.rightLine.intersection2Bottom.x) {
		pos = Right_Lane; // on the right of the main lane
	} else {
		pos = Center_lane; //inside the main lane
	}
	return angle;

}

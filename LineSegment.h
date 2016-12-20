/*
 * LineSegment.h
 *
 *  Created on: Aug 3, 2016
 *      Author: anh
 */

#ifndef LINESEGMENT_H_
#define LINESEGMENT_H_
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
class LineSegment {

public:
	cv::Point point1;
	cv::Point point2;
	cv::Point2f intersection2Bottom;
	cv::Point2f intersection2Middle;
	//cv::Point intersectionLeftRight;
	float distance;
        //2016.11.09 cong.anh
        cv::Point cpoint;//center point of each line
        int RoiPos;
	void setPoint(cv::Point pt1, cv::Point pt2);
	void setPoint(cv::Point pt1, cv::Point pt2, cv::Point inter2Bottom, cv::Point inter2Middle,float distance2Bottom);
	
	LineSegment();
	LineSegment(cv::Point p1, cv::Point p2);
};

class StoredLine: public LineSegment {

public:
	/*cv::Point point1;
	 cv::Point point2;*/
	int ID;
	bool bIsUpdated;
	int Count;
	bool bIsPrimary;
	StoredLine();
	StoredLine(int id, cv::Point p1, cv::Point p2, int count = 1);

};

class MatchedLine: public LineSegment {
public:
	bool isMatched;

	MatchedLine();

};

#endif /* LINESEGMENT_H_ */
/*
 * Line.h
 *
 *  Created on: Aug 3, 2016
 *      Author: anh
 */


/*
 * LaneInfo.h
 *
 *  Created on: Aug 10, 2016
 *      Author: anh
 */

#ifndef LANEINFO_H_
#define LANEINFO_H_
#include "LineSegment.h"

class LaneInfo {
public:
	int Id;
	cv::Point2f intersecLeftRight;
	LineSegment leftLine, rightLine, centerLine;
	LaneInfo();
	LaneInfo(LineSegment left, LineSegment right);
	void setLine(LineSegment left, LineSegment right);
};

#endif /* LANEINFO_H_ */

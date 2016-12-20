/*
 * LaneInfo.cpp
 *
 *  Created on: Aug 10, 2016
 *      Author: anh
 */

#include "LaneInfo.h"
#include "LineSegment.h"
#include "MSAC.h"

LaneInfo::LaneInfo() {
intersecLeftRight = cv::Point(0,0);
}
LaneInfo::LaneInfo(LineSegment left, LineSegment right) {
	leftLine = left;
	rightLine = right;
}
void LaneInfo::setLine(LineSegment left, LineSegment right) {
	//Left side of the main lane
	leftLine.intersection2Bottom = left.intersection2Bottom;
	leftLine.intersection2Middle = left.intersection2Middle;
	leftLine.point1 = left.point1;
	leftLine.point2 = left.point2;

	//right side of the main lane
	rightLine.intersection2Bottom = right.intersection2Bottom;
	rightLine.intersection2Middle = right.intersection2Middle;

	rightLine.point1 = right.point1;
	rightLine.point2 = right.point2;
}


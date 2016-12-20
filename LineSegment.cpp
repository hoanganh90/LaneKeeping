/*
 * LineSegment.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: anh
 */

#include "LineSegment.h"

/*
 * Line.cpp
 *
 *  Created on: Aug 3, 2016
 *      Author: anh
 */

LineSegment::LineSegment() {

}
LineSegment::LineSegment(cv::Point p1, cv::Point p2) {
	point1 = p1;
	point2 = p2;
}

void LineSegment::setPoint(cv::Point pt1, cv::Point pt2) {
	point1 = pt1;
	point2 = pt2;
        cpoint = (pt1 + pt2)/2;
}
void LineSegment::setPoint(cv::Point pt1, cv::Point pt2, cv::Point inter2Bottom, cv::Point inter2Middle, float distance2Bottom) {
	point1 = pt1;
	point2 = pt2;
	intersection2Bottom = inter2Bottom;
	intersection2Middle = inter2Middle;
	distance = distance2Bottom;
}
StoredLine::StoredLine() :
		LineSegment() {
	bIsUpdated = false;
	bIsPrimary = false;
	Count = 1;

}

StoredLine::StoredLine(int id, cv::Point p1, cv::Point p2, int count) :
		LineSegment(p1, p2) {
	ID = id;
	Count = count;
	bIsUpdated = false;
}

MatchedLine::MatchedLine() :
		LineSegment() {

	isMatched = false;
	//distance = 0;
}
;


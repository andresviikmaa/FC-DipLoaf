#include "kdNode2D.h"


static int cmpX(const void* a, const void* b)
{
	return (int)((*(BallLocation*)a).fieldCoords.x - (*(BallLocation*)b).fieldCoords.x);
}

static int cmpY(const void* a, const void* b)
{
	return (*(BallLocation*)a).fieldCoords.y - (*(BallLocation*)b).fieldCoords.y;
}

kdNode2D::kdNode2D(BallLocation* pointList, int pointLength, int depth)
{
	if (pointLength == 1) {
		left = NULL;
		right = NULL;
		p = pointList;
		std::cout << "construct 0: " << p->id << std::endl;
		return;
	}

	// Odd depth = Y, even depth = X
	if (depth & 1)
		qsort(pointList, pointLength, sizeof(BallLocation), cmpY);
	else
		qsort(pointList, pointLength, sizeof(BallLocation), cmpX);

	const unsigned int halfLength = pointLength >> 1;
	p = &pointList[halfLength];
	std::cout << "construct 1: " << p->id << std::endl;
	for (size_t i = 0; i < halfLength; i++){
		std::cout << pointList[i].id << " ";
	}
	std::cout <<" ["<< p->id << "] ";
	for (int i = halfLength+1; i < pointLength; i++){
		std::cout << pointList[i].id << " ";
	}
	std::cout << std::endl;
	left = new kdNode2D(pointList, halfLength, depth + 1);
	right = pointLength == halfLength+1 ? NULL : new kdNode2D(pointList + halfLength + 1, pointLength - halfLength -1, depth + 1);
}

std::pair<unsigned, BallLocation*>  kdNode2D::nearest(const cv::Point &point, int depth)
{
	std::cout << "looking from " << p->id << " " << std::endl;
	/* End of tree. */
	/*
	if (!left && !right)   
	{
		cv::Point2d r = p->fieldCoords;
		r.x -= point.x;
		r.y -= point.y;
		if (p->isUpdated) {
			std::cout << "point " << p->id << " is used" << std::endl;
			return{ (unsigned)(r.dot(r)), NULL }; // throw std::runtime_error("backrack");
		}
		std::cout << "found point " << p->id << "" << std::endl;
		//used = true;
		return { (unsigned)(r.dot(r)), p };
	}
	*/
	const int tmp = (int)(depth == 0 ? p->fieldCoords.x - point.x : p->fieldCoords.y - point.y);
	const int side = tmp < 0; /* Prefer the left. */

	/* Switch depth. */
	depth ^= 1;
	std::pair<unsigned, BallLocation*> left = { 0, NULL };
	std::pair<unsigned, BallLocation*> right = { 0, NULL };

	/* Search the near side of the tree. */
	if (sons[side]){
		std::cout << "look left " << sons[side]->p->id << "" << std::endl;
		left = sons[side]->nearest(point, depth);
		/* Radius intersects a kd tree boundary? */
		if (left.first < (tmp * tmp) && left.second != NULL)
		{
			std::cout << "found left point0 " << left.second->id << "" << std::endl;
			return left;
		}
	}
	/* Yes; look at the points on the other side. */
	if (sons[side ^ 1]){
		std::cout << "look right " << sons[side ^ 1]->p->id << "" << std::endl;
		right = sons[side ^ 1]->nearest(point, depth);
	}
	if (right.second == NULL && left.second == NULL){ // both sides are used, return itself
		cv::Point2d r = cv::Point2d(p->fieldCoords.x, p->fieldCoords.y);
		r.x -= point.x;
		r.y -= point.y;
		if (p->isUpdated) {
			std::cout << "point " << p->id << " is used" << std::endl;
			return{ (unsigned)(r.dot(r)), NULL }; // throw std::runtime_error("backrack");
		}
		std::cout << "found point " << p->id << "" << std::endl;
		return{ (unsigned)(r.dot(r)), p };
	} 
	else if (right.second == NULL){
		std::cout << "found left point " << left.second->id << "" << std::endl;
		return left;
	}
	else if (left.second == NULL){
		std::cout << "found right point " << right.second->id << "" << std::endl;
		return right;
	} 
	else if (left.first < right.first){
		std::cout << "found left2 point " << left.second->id << "" << std::endl;
		return left;
	}
	else {
		std::cout << "found right2 point " << right.second->id << "" << std::endl;
		return right;
	}
}
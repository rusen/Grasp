/*
 * Path.h
 *
 *  Created on: 28 Aug 2017
 *      Author: rusi
 */

#ifndef SRC_UTIL_PATH_H_
#define SRC_UTIL_PATH_H_

#include "util/Path.h"
#include "util/Waypoint.h"

namespace Grasp {

class Path {
public:
	Path(int count = 0);
	~Path();
	int getSteps() const;
	void setSteps(int steps);
	int steps = 1500;
	Waypoint * waypoints;

	// Linear interpolation
	Waypoint Interpolate(int step);

	int getWaypointCount() const {
		return waypointCount;
	}

	void setWaypointCount(int waypointCount = 0) {
		this->waypointCount = waypointCount;
	}


private:
	int waypointCount = 0;
};

} /* namespace Grasp */

#endif /* SRC_UTIL_PATH_H_ */

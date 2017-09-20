/*
 * Waypoint.h
 *
 *  Created on: 28 Aug 2017
 *      Author: rusi
 */

#ifndef SRC_UTIL_WAYPOINT_H_
#define SRC_UTIL_WAYPOINT_H_

#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Grasp {

class Waypoint {
public:

	glm::vec3 pos;	// position of wrist.
	glm::quat quat; // Orientation of wrist.
	int jointCount;	// Number of joints in hand
	float * jointAngles;	// Joint angles at this waypoint.

	Waypoint();
	~Waypoint();
	Waypoint (const Waypoint &obj); // copy
	void print();
	void SetInitialPose (const double * pose); // copy
};

} /* namespace Grasp */

#endif /* SRC_UTIL_WAYPOINT_H_ */

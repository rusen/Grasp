/*
 * GraspResult.h
 *
 *  Created on: 26 Dec 2017
 *      Author: rusi
 */

#ifndef INCLUDE_PLANNER_GRASPRESULT_H_
#define INCLUDE_PLANNER_GRASPRESULT_H_

#include <glm/gtc/quaternion.hpp>
#include <stdio.h>

namespace Grasp {

class GraspResult {
public:
	double successProbability = 0;
	double x1 = 0, r1 = 0, x2 = 0, r2=0;
	int counter = 0, successCounter = 0;
	int viewId = 0;
	int wpCount = 0;
	float likelihood = 0;
	int graspType = 0;
	glm::vec3 gazeDir, camPos; // centerPos is center of gravity of point cloud.
	GraspResult(){}
	virtual ~GraspResult(){}

	// File IO
	void read(FILE * &fp, bool likelihoodFlag);
	void write(FILE * &fp);
	void print();

};

} /* namespace Grasp */

#endif /* INCLUDE_PLANNER_GRASPRESULT_H_ */

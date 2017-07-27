/*
 * GraspPlanner.h
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#ifndef INCLUDE_CONTROLLER_GRASPPLANNER_H_
#define INCLUDE_CONTROLLER_GRASPPLANNER_H_

#include "mujoco.h"
#include <controller/HandControllerInterface.h>
#include <controller/MPLHandController.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <algorithm>    // std::copy
#include <vector>

namespace Grasp {

enum state { initial, approaching, atPreGraspLocation, atFinalApproach, readyToGrasp, grasping, lifting, done };
enum handType {MPL};
enum approachType {initialApproach, finalApproach};

class GraspPlanner {
public:
	GraspPlanner();
	virtual ~GraspPlanner();

	// Compute trajectory.
	void ComputeTrajectory();

	// Follow computed trajectory.
	bool FollowTrajectory(const mjModel* m, mjData* d, approachType type);

	// Perform grasp.
	void PerformGrasp(const mjModel* m, mjData* d, HandControllerInterface * handController);

private:
	state graspState = initial;
	bool startFlag = false;

	// Steps to perform approach.
	int counter = 0;
	int approachCounterLimit = 1000;
	int finalApproachCounterLimit = 500;

	// Allocate space for trajectories.
	glm::vec3 *initialApproachPos = NULL;
	glm::quat *initialApproachQuat = NULL;
	glm::vec3 *finalApproachPos = NULL;
	glm::quat *finalApproachQuat = NULL;

	// Info for bezier curve.
	glm::vec3 initDir, finalDir;
	glm::vec3 p1, p2;

	// Hand controller allocation.
	MPLHandController controller;
};


} /* namespace Grasp */

#endif /* INCLUDE_CONTROLLER_GRASPPLANNER_H_ */

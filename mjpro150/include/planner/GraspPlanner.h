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
#include <controller/DLRHandController.h>
#include <sensor/simulate.h>
#include <sensor/camera.h>
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

enum state { initial, atDataApproach, collectingData, approaching, atPreGraspLocation, atFinalApproach, readyToGrasp, grasping, lifting, done };
enum handType {MPL};
enum approachType {initialApproach, preApproach, finalApproach};

class GraspPlanner {
public:

	int camSize[2] = {480, 640};
	unsigned char depthBuffer[640*480*3];
	cv::Mat pointCloud;

	// Simulator allocation
	Simulate* Simulator = NULL;

	GraspPlanner();
	virtual ~GraspPlanner();

	// Compute trajectory.
	void ComputeTrajectory();

	// Follow computed trajectory.
	bool FollowTrajectory(const mjModel* m, mjData* d, approachType type);

	// Perform grasp.
	void PerformGrasp(const mjModel* m, mjData* d);

private:
	state graspState = initial;
	bool startFlag = false;
	bool finishFlag = false;
	bool fastSim = false; // If true, we're in fast simulation mode -  no unnecessary work.

	// Steps to perform approach.
	int counter = 0;
	int approachCounterLimit = 1000;
	int finalApproachCounterLimit = 500;

	// Allocate space for trajectories.
	glm::vec3 *initialApproachPos = NULL;
	glm::quat *initialApproachQuat = NULL;
	glm::vec3 *preApproachPos = NULL;
	glm::quat *preApproachQuat = NULL;
	glm::vec3 *finalApproachPos = NULL;
	glm::quat *finalApproachQuat = NULL;

	// Info for bezier curve.
	glm::vec3 initDir, finalDir;
	glm::vec3 p1, p2;
	glm::vec3 cameraPos, gazeDir;

	// Hand controller allocation.
	DLRHandController controller;

};


} /* namespace Grasp */

#endif /* INCLUDE_CONTROLLER_GRASPPLANNER_H_ */

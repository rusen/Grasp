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
#include "util/Path.h"
#include <stdio.h>      /* printf, NULL */
#include <stdlib.h>     /* srand, rand */
#include <time.h>       /* time */
#include <iostream>
#include <algorithm>    // std::copy
#include <vector>
#include <thread>

namespace Grasp {

enum state { collectingData, planning, pregrasp, checkingCollision, grasping, lifting, stand, reset, done };
enum handType {MPL};

class GraspPlanner {
public:

	int camSize[2] = {480, 640};
	unsigned char rgbBuffer[640*480*3];
	unsigned char depthBuffer[640*480*3];
	float minPointZ = -100; // minimum Z coordinate of allowed points in the point cloud
	std::vector<std::vector<float>> convHullPoints;
	float yOffset = 2; // For collision detection, we try hand positions on a separate part of the table, separated by a y offset.
	char fileId [10]; // Unique file id
	char logFile [1000]; // Log file
	char debugLogFile [1000]; // Log file
	char modelFile [1000]; // Model file (binary)
	char pointFile [1000]; // Point cloud data file (.pcd)
	char rgbFile [1000];  // RGB file (.png)
	char depthFile [1000];  // RGB file (.png)
	char resultFile[1000]; // Grasp success/diagnostics file (.gd)
	char trajectoryFile[1000]; // Trajectory file (.trj)
	char dropboxFolder[1000]; // Dropbox folder for this run
	FILE * trjFP = NULL;
	int numberOfGrasps = 0;
	int collisionPoints = 50, collisionCounter = 0;
	bool collisionSet = false, collisionRun = true, hasCollided = false;
	float* data = NULL;

	// Upload and timeout time
	time_t uploadTime = 0;
	time_t trajectoryTimeout = 600; // seconds. If trajectory doesn't arrive within this timeframe, operation aborted.

	// Steps to perform approach.
	int counter = 0;
	int graspCounter = 0;

	// Simulator allocation
	Simulate* Simulator = NULL;

	GraspPlanner(const char * dropboxFolder);
	virtual ~GraspPlanner();

	// Reset simulation to initial configuration
	void SetFrame(const mjModel* m, mjData* d);

	// Compute trajectory.
	void ComputeTrajectory();

	// Gets the trajectory from the data trajectory file,
	// and assigns it as next trajectory.
	void ReadTrajectory();

	// CheckCollision checks the collision of
	// the next trajectory with the table.
	// Returns true if grasp succeeds ( no collision), returns false if fails.
	bool CheckCollision();

	// Follow computed trajectory.
	bool FollowTrajectory(const mjModel* m, mjData* d, float yOffset);

	// Perform grasp.
	void PerformGrasp(const mjModel* m, mjData* d, mjtNum * stableQpos, mjtNum * stableQvel, mjtNum * stableCtrl, mjvScene *scn, mjrContext *con);

	state getGraspState() const;
	void setGraspState(state graspState);

private:
	state graspState = collectingData; //initial for full scenario
	state prevState = collectingData; // Previous state
	bool startFlag = false;
	bool finishFlag = false;
	bool uploadFlag = false;
	std::thread * dataThread = NULL;

	// Approach waypoints.
	Waypoint capturePos; // Position of the wrist in capture mode.
	Path *finalApproach = NULL; // Path to the final approach.

	// Info for camera capture.
	glm::vec3 cameraPos, gazeDir;

	// Hand controller allocation.
	DLRHandController controller;
};

} /* namespace Grasp */

#endif /* INCLUDE_CONTROLLER_GRASPPLANNER_H_ */

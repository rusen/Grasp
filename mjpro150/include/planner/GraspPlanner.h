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
#include <planner/GraspResult.h>
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

#define RF (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))
#define PI 3.14159265

namespace Grasp {

enum state { collectingData, planning, pregrasp, checkingCollision, grasping, lifting, stand, reset, done };
enum handType {MPL};

class GraspPlanner {
public:
	// All relevant variables come here
	int camSize[2] = {480, 640};
	unsigned char rgbBuffer[640*480*3];
	unsigned char depthBuffer[640*480*3];
	float minPointZ = -0.34; // minimum Z coordinate of allowed points in the point cloud
	int minPointCount = 50;
	std::vector<std::vector<float>> convHullPoints;
	float yOffset = 2; // For collision detection, we try hand positions on a separate part of the table, separated by a y offset.
	char fileId [10]; // Unique file id
	char baseFolder [1000]; // Tmp base folder
	char videoFolder [1000]; // Tmp base folder
	char dataFile[1000];
	char logFile [1000]; // Log file
	char debugLogFile [1000]; // Log file
	char modelPrefix [1000]; // Model file prefix
	char pointFile [1000]; // Point cloud data file (.pcd)
	char rgbFile [1000];  // RGB file (.png)
	char depthFile [1000];  // RGB file (.png)
	char resultFile[1000]; // Grasp success/diagnostics file (.gd)
	char trajectoryFile[1000]; // Trajectory file (.trj)
	char dropboxFolder[1000]; // Dropbox folder for this run
	int baseType = 0; // Type of base for current object
	FILE * trjFP = NULL;

	// Grasp-related variables.
	int numberOfGrasps = 0;
	int numberOfMaximumGrasps = 1000;
	int numberOfTrials = 1;
	int varItr = 0;
	int collisionPoints = 50, collisionCounter = 0;
	bool collisionSet = false, collisionRun = true, hasCollided = false;
	bool testFlag = false;
	bool reSimulateFlag = false;
	float* data = NULL;
	int collisionState[1000];
	std::ofstream *logStream = NULL;

	// Grasp output array
	GraspResult* resultArr = NULL;
	std::vector<std::vector<float>> graspParams;
	std::vector<int> validViews;
	int validViewPixels = 250;

	// Grasp state variables
	state graspState = collectingData; //initial for full scenario
	state prevState = collectingData; // Previous state

	// Info for camera capture.
	glm::vec3 cameraPos, gazeDir;
	std::vector <glm::vec3> cameraPosArr, gazeDirArr;
	int numberOfAngles = 20;

	// Upload and timeout time
	time_t uploadTime = 0;
	time_t trajectoryTimeout = 1200; // seconds. If trajectory doesn't arrive within this timeframe, operation aborted.

	// Random seed (for srand)
	time_t randSeed = 0;

	// Steps to perform approach.
	int counter = 0;
	bool singleGraspExecution = false;
	int singleExecutedGrasp = 0;
	int graspCounter = 0;
	int stableCounter = 0;
	int stableLimit = 25;
	int numberOfNoncollidingGrasps = 0;

	// Simulator allocation
	Simulate* Simulator = NULL;

	GraspPlanner(const char * dropboxFolder, bool testFlag, bool reSimulateFlag, const char * existingId, char * dataFileName, int executedGrasp=-1);
	virtual ~GraspPlanner();

	// Reset simulation to initial configuration
	void SetFrame(const mjModel* m, mjData* d);

	// Compute trajectory.
	void ComputeTrajectory();

	// Gets the trajectory from the data trajectory file,
	// and assigns it as next trajectory.
	void ReadTrajectories( int numberOfGrasps );

	// Gets the trajectory from the data trajectory file,
	// and assigns it as next trajectory.
	void ReadPreMadeTrajectories( int numberOfGrasps );

	// CheckCollision checks the collision of
	// the next trajectory with the table.
	// Returns true if grasp succeeds ( no collision), returns false if fails.
	bool CheckCollision();

	// Follow computed trajectory.
	bool FollowTrajectory(const mjModel* m, mjData* d, float yOffset);

	// Perform grasp.
	void PerformGrasp(const mjModel* &m, mjData* &d, mjtNum * stableQpos, mjtNum * stableQvel, mjtNum * stableCtrl);

	state getGraspState() const;
	void setGraspState(state graspState);

private:
	bool startFlag = false;
	bool finishFlag = false;
	bool uploadFlag = false;
	std::thread * dataThread = NULL;

	// Approach waypoints.
	Waypoint capturePos; // Position of the wrist in capture mode.
	Path **finalApproachArr = NULL; // Path to the final approach.
	int trajectoryCounter = 0;

	// Hand controller allocation.
	DLRHandController controller;
};

} /* namespace Grasp */

#endif /* INCLUDE_CONTROLLER_GRASPPLANNER_H_ */

/*
 * Path.cpp
 *
 *  Created on: 28 Aug 2017
 *      Author: rusi
 */

#include "util/Path.h"
#include <math.h>       /* floor */
#include <iostream>

namespace Grasp {

Path::Path(int count) {
	// Initialize the relevant structures.
	waypointCount = count;
	waypoints = new Waypoint[waypointCount];
}

Path::~Path() {
	delete[] waypoints;
}

int Path::getSteps() const {
	return steps;
}

void Path::setSteps(int steps) {
	this->steps = steps;
}

Waypoint Path::Interpolate(int step){

	// Find relevant waypoints and return the interpolation.
	float stepSize = ((float) steps) / (waypointCount-1);
//	std::cout<<"Step Size: "<<stepSize<<std::endl;
	float val = (float) step / stepSize;
	int startPoint = floor(val);
	int endPoint = ceil(val);

	// Sanity checks.
	if (startPoint < 0)
		startPoint = 0;
	if (endPoint > waypointCount)
		endPoint = waypointCount;

	// Weights for combination
	float endWeight = (val - startPoint);
	float startWeight = endPoint - val;

	if (startPoint == endPoint)
		return waypoints[startPoint];
	else
	{

		Waypoint point;

		// Interpolate two closest waypoints.
		point.pos = waypoints[startPoint].pos * startWeight +
				waypoints[endPoint].pos * endWeight;
		point.quat = glm::slerp(waypoints[startPoint].quat, waypoints[endPoint].quat, endWeight);

		// Interpolate hand joint positions as well.
		for (int i = 0; i<waypoints[startPoint].jointCount; i++){
			point.jointAngles[i] = waypoints[startPoint].jointAngles[i] * startWeight +
					waypoints[endPoint].jointAngles[i] * endWeight;
		}
		return point;
	}
}

std::vector<float> Path::getGraspParams(Eigen::Vector3f gazeDir, Eigen::Vector3f camPos){
	// Get up, right vectors as well.
    // Find right and up vectors
	gazeDir.normalize();
    Eigen::Vector3f tempUp(0, 0, 1), normRight, viewUp;
    normRight = gazeDir.cross(tempUp);
    normRight.normalize();
    viewUp = normRight.cross(gazeDir);
    viewUp.normalize();

    // Create transformation matrices
	Eigen::Matrix4f cameraTraM, cameraRotM;
	cameraRotM.setZero(), cameraTraM.setZero();
	Eigen::Matrix3f camFrame;
	camFrame.setZero();

	// Translation
	cameraTraM(0,0) = 1, cameraTraM(1,1) = 1, cameraTraM(2,2) = 1, cameraTraM(3,3) = 1;
	cameraTraM(0,3) = -camPos(0), cameraTraM(1,3) = -camPos(1), cameraTraM(2,3) = -camPos(2);

	// Rotation
	cameraRotM(0,0) = normRight(0), cameraRotM(0,1) = normRight(1), cameraRotM(0,2) = normRight(2);
	cameraRotM(1,0) = viewUp(0), cameraRotM(1,1) = viewUp(1), cameraRotM(1,2) = viewUp(2);
	cameraRotM(2,0) = gazeDir(0), cameraRotM(2,1) = gazeDir(1), cameraRotM(2,2) = gazeDir(2);
	cameraRotM(3,3) = 1;
	camFrame(0,0) = cameraRotM(0,0), camFrame(0,1) = cameraRotM(0,1), camFrame(0,2) = cameraRotM(0,2);
	camFrame(1,0) = cameraRotM(1,0), camFrame(1,1) = cameraRotM(1,1), camFrame(1,2) = cameraRotM(1,2);
	camFrame(2,0) = cameraRotM(2,0), camFrame(2,1) = cameraRotM(2,1), camFrame(2,2) = cameraRotM(2,2);

	// Final transformation matrices/quats
	Eigen::Quaternionf tQuat = Eigen::Quaternionf(camFrame).inverse();
	Eigen::Matrix4f tM = cameraRotM * cameraTraM;

	// Interpolate waypoints to 10 separate points.
	int limit = 10;
	std::vector<float> outputData;
	for (int i = 0; i<limit * 27 + 10; i++)
		outputData.push_back(0);

	// Set grasp type in data
	outputData[graspType] = 1;

	int currentOffset = 0;
	for (int k = 0; k<limit; k++){
		int cnt = round(k * (((double)steps) / (limit-1)));
		Grasp::Waypoint wp = Interpolate(cnt);

		// Convert to view coordinates.
		Eigen::Vector4f pos(wp.pos[0], wp.pos[1], wp.pos[2], 1);

		// Transform wrist point
		pos = tM * pos;
		Eigen::Quaternionf wristQuat = Eigen::Quaternionf(wp.quat.w, wp.quat.x, wp.quat.y, wp.quat.z);
		wristQuat = tQuat * wristQuat;

		// Write back the data
		outputData[currentOffset + k * 27 + 10] = pos[0];
		outputData[currentOffset + k * 27 + 11] = pos[1];
		outputData[currentOffset + k * 27 + 12] = pos[2];
		outputData[currentOffset + k * 27 + 13] = wristQuat.w();
		outputData[currentOffset + k * 27 + 14] = wristQuat.x();
		outputData[currentOffset + k * 27 + 15] = wristQuat.y();
		outputData[currentOffset + k * 27 + 16] = wristQuat.z();

		for (int i = 0; i<20; i++)
			outputData[currentOffset + k * 27 + 17 + i] = wp.jointAngles[i];
	}

	return outputData;
}


} /* namespace Grasp */

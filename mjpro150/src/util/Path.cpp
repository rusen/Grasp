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

std::vector<float> Path::getGraspParams(glm::vec3 gazeDir, glm::vec3 camPos){
	// Get up, right vectors as well.
    // Find right and up vectors
    glm::vec3 tempUp(0, 0, 1), normRight, viewUp;
    normRight = normalize(glm::cross(gazeDir, tempUp));
    viewUp = normalize(glm::cross(normRight, gazeDir));

    // Find camera transformation matrix (without projection)
	glm::mat4 viewM = glm::lookAt(camPos, camPos + gazeDir, viewUp);

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
		glm::vec4 pos;
		pos[0] = wp.pos[0];
		pos[1] = wp.pos[1];
		pos[2] = wp.pos[2];
		pos[3] = 1;

		pos = viewM * pos;

		// Get rotation matrix
		glm::mat4 rotM = glm::toMat4(wp.quat);

		// Convert to the camera coordinate system.
		rotM = viewM * rotM;

		glm::quat newQuat = glm::toQuat(rotM);
		outputData[currentOffset + k * 27 + 10] = pos[0];
		outputData[currentOffset + k * 27 + 11] = pos[1];
		outputData[currentOffset + k * 27 + 12] = pos[2];
		outputData[currentOffset + k * 27 + 13] = newQuat.w;
		outputData[currentOffset + k * 27 + 14] = newQuat.x;
		outputData[currentOffset + k * 27 + 15] = newQuat.y;
		outputData[currentOffset + k * 27 + 16] = newQuat.z;

		for (int i = 0; i<20; i++)
			outputData[currentOffset + k * 27 + 17 + i] = wp.jointAngles[i];
	}

	return outputData;
}


} /* namespace Grasp */

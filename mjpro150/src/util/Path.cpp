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

/*
Waypoint Path::Interpolate(int step, bool reconstruct){

	// Find relevant waypoints and return the interpolation.
	float stepSize = ((float) steps) / (waypointCount-1);
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
*/

Waypoint Path::Interpolate(int step, bool reconstruct){

	// Find relevant waypoints and return the interpolation.
	float stepSize = ((float) steps) / (waypointCount-1);
	float val = (float) step / stepSize;
	int startPoint = floor(val);
	int endPoint = ceil(val);

	// Sanity checks.
	if (startPoint < 0)
		startPoint = 0;
	if (endPoint > waypointCount)
		endPoint = waypointCount;

	// If start/end endpoints are the same, we just return the relevant waypoint.
	if (startPoint == endPoint)
	{
		return waypoints[startPoint];
	}

	// Weights for combination
	float endWeight = (val - startPoint);
	float startWeight = endPoint - val;

	Waypoint point1, point2, point, *pp;

	// If reconstruct option is on, it means we're trying to find
	// the original waypoints, and simple interpolation will not
	// work here. We attempt to find earlier/later waypoints
	// (if possible), and extrapolate from there instead.
	if (reconstruct)
	{
		for (int itr = 0; itr<2; itr++)
		{
			int curStartPoint = startPoint - 1;
			int curEndPoint = endPoint - 1;
			pp = &point1;
			if (itr)
			{
				curStartPoint = startPoint + 1;
				curEndPoint = endPoint + 1;
				pp = &point2;
			}
			float curEndWeight = (val - curStartPoint);
			float curStartWeight = curEndPoint - val;

			// Interpolate two closest waypoints.
			pp->pos = waypoints[curStartPoint].pos * curStartWeight +
					waypoints[curEndPoint].pos * curEndWeight;
			pp->quat = glm::slerp(waypoints[curStartPoint].quat, waypoints[curEndPoint].quat, curEndWeight);

			// Interpolate hand joint positions as well.
			for (int i = 0; i<waypoints[curStartPoint].jointCount; i++){
				pp->jointAngles[i] = waypoints[curStartPoint].jointAngles[i] * curStartWeight +
						waypoints[curEndPoint].jointAngles[i] * curEndWeight;
			}
		}

//		std::cout<<"Both points found!"<<std::endl;
		for (int ktr = 0; ktr<3; ktr++)
			point.pos[ktr] = (point1.pos[ktr] + point2.pos[ktr])/2;
		point.quat = point1.quat;
		// Interpolate hand joint positions as well.
		for (int i = 0; i<waypoints[startPoint].jointCount; i++)
			point.jointAngles[i] = (point1.jointAngles[i] + point2.jointAngles[i])/2;
	}
	else{
		// Interpolate two closest waypoints.
		point.pos = waypoints[startPoint].pos * startWeight +
				waypoints[endPoint].pos * endWeight;
		point.quat = glm::slerp(waypoints[startPoint].quat, waypoints[endPoint].quat, endWeight);

		// Interpolate hand joint positions as well.
		for (int i = 0; i<waypoints[startPoint].jointCount; i++){
			point.jointAngles[i] = waypoints[startPoint].jointAngles[i] * startWeight +
					waypoints[endPoint].jointAngles[i] * endWeight;
		}
	}
	return point;
}


std::vector<float> Path::getGraspParams(Eigen::Vector3f gazeDir, Eigen::Vector3f camPos, int wpCount){

	// Obtain transformation matrices
	Transformation trans(gazeDir, camPos, true);
	Transformation transInv(gazeDir, camPos, false);

	// Interpolate waypoints to 10 separate points.
	int limit = 10;
	std::vector<float> outputData;
	for (int i = 0; i<limit * 27 + 10; i++)
		outputData.push_back(0);

	// Set start offset for outputting parameters.
	int startOffset = ceil(((double)steps) / (wpCount+1));
	int range = steps - startOffset;

	// Set grasp type in data
	outputData[graspType] = 1;

	int currentOffset = 0;
	for (int k = 0; k<limit; k++){
		int cnt = round(k * (((double)range) / (limit-1))) + startOffset;
		Grasp::Waypoint wp = Interpolate(cnt, false);

		// Convert to view coordinates.
		Eigen::Vector4f pos(wp.pos[0], wp.pos[1], wp.pos[2], 1);

		// Transform wrist point
		pos = trans.tM * pos;
		Eigen::Vector4f pos2 = transInv.tM * pos;
		Eigen::Matrix4f id = transInv.tM * trans.tM;
		Eigen::Quaternionf wristQuat = Eigen::Quaternionf(wp.quat.w, wp.quat.x, wp.quat.y, wp.quat.z);
		wristQuat = trans.tQuat * wristQuat;

		// Write back the data
		outputData[currentOffset + k * 27 + 10] = pos[0];
		outputData[currentOffset + k * 27 + 11] = pos[1];
		outputData[currentOffset + k * 27 + 12] = pos[2];
		outputData[currentOffset + k * 27 + 13] = wristQuat.w();
		outputData[currentOffset + k * 27 + 14] = wristQuat.x();
		outputData[currentOffset + k * 27 + 15] = wristQuat.y();
		outputData[currentOffset + k * 27 + 16] = wristQuat.z();

		for (int i = 0; i<20; i++)
		{
			outputData[currentOffset + k * 27 + 17 + i] = wp.jointAngles[i];
		}
	}

	return outputData;
}


} /* namespace Grasp */

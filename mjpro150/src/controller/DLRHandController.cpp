/*
 * DLRHandController.cpp
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#include <controller/DLRHandController.h>
#include <iostream>
#include <cmath>        // std::abs

namespace Grasp{

DLRHandController::DLRHandController() {
	// TODO Auto-generated constructor stub
	jointCount = DLR_JOINT_COUNT;
}

DLRHandController::~DLRHandController() {
	// TODO Auto-generated destructor stub

}

void DLRHandController::SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q){
	// Assign position
	d->mocap_pos[0] = pos[0];
	d->mocap_pos[1] = pos[1];
	d->mocap_pos[2] = pos[2];

	// Assign orientation
	d->mocap_quat[0] = q[2];
	d->mocap_quat[1] = -q[1];
	d->mocap_quat[2] = q[0];
	d->mocap_quat[3] = q[3];

}

bool DLRHandController::Grasp(const mjModel* m, mjData* d, graspType type){

	// Decide which joint modifications to use.
	double * curPoses = NULL;
	switch (type)
	{
		case initialGrasp:
			curPoses = initialPoses;
			break;
		case finalGrasp:
			curPoses = finalPoses;
			break;
	}

	// Move joints to initial positions.
	for (int i = 0; i < jointCount; i++)
	{
		if (std::abs(curPoses[i]) > 0.001)
		{
			d->ctrl[i] += curPoses[i]/counterLimit;
		}
	}

	// Increase counter and finish operation if needed.
	counter++;
	if (counter > counterLimit)
	{
		counter = 0;
		return true;
	}
	else
		return false;
}


}

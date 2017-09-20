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

void DLRHandController::SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q, float* jointAngles){

	q = q * glm::quat(glm::vec3(-1.5708, 0, -0.7854));

	// Assign position
	d->mocap_pos[0] = pos[0];
	d->mocap_pos[1] = pos[1];
	d->mocap_pos[2] = pos[2];

//	std::cout<<d->mocap_quat[0]<<d->mocap_quat[1]<<d->mocap_quat[2]<<d->mocap_quat[3]<<std::endl;

	d->mocap_quat[0] = q.w;
	d->mocap_quat[1] = q.x;
	d->mocap_quat[2] = q.y;
	d->mocap_quat[3] = q.z;

	// Move joints to initial positions.
	for (int i = 0; i < jointCount; i++)
	{
		d->ctrl[i] = jointAngles[i];
	}
}

}

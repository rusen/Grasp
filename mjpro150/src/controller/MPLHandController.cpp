/*
 * MPLHandController.cpp
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#include <controller/MPLHandController.h>
#include <iostream>
#include <cmath>        // std::abs

namespace Grasp{

MPLHandController::MPLHandController() {
	// TODO Auto-generated constructor stub
	jointCount = 13;
}

MPLHandController::~MPLHandController() {
	// TODO Auto-generated destructor stub

}

void MPLHandController::SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q, float * jointAngles){
	// Assign position
	d->mocap_pos[0] = pos[0];
	d->mocap_pos[1] = pos[1];
	d->mocap_pos[2] = pos[2];

	// Assign orientation
	d->mocap_quat[0] = q[2];
	d->mocap_quat[1] = -q[1];
	d->mocap_quat[2] = q[0];
	d->mocap_quat[3] = q[3];

//	std::cout<<d->mocap_quat[0]<<" "<<d->mocap_quat[1]<<" "<<d->mocap_quat[2]<<" "<<d->mocap_quat[3]<<std::endl;

}

}

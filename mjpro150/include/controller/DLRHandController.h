/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef GRASP_CONTROLLER_DLRHANDCONTROLLER_H_
#define GRASP_CONTROLLER_DLRHANDCONTROLLER_H_

#include "HandControllerInterface.h"
#include "stdio.h"
#define DLR_JOINT_COUNT 20

namespace Grasp{

class DLRHandController: public HandControllerInterface {

public:
	DLRHandController();
	~DLRHandController();

	// Set the pose of the hand.
	void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q, float * jointAngles);
};


} //namespace grasp
#endif /* GRASP_CONTROLLER_DLRHANDCONTROLLER_H_ */

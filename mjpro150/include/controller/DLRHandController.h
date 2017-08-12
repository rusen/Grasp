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

namespace Grasp{

class DLRHandController: public HandControllerInterface {

public:
	DLRHandController();
	~DLRHandController();

	void ComputeTrajectory();

	// Set the pose of the hand.
	void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q);

	// Set Initial pose of the hand.
	bool Grasp(const mjModel* m, mjData* d, graspType type);

protected:

private:
	double initialPoses[15] = {0, 0.25, 0, 0, 0.25, 0, 0, 0.25, 0, 0, 0.25, 0, 0, 0.25, 0};
	double finalPoses[15] = {0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1, 0, 1, 1};
};


} //namespace grasp
#endif /* GRASP_CONTROLLER_DLRHANDCONTROLLER_H_ */

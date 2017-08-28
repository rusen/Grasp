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

	void ComputeTrajectory();

	// Set the pose of the hand.
	void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q);

	// Set Initial pose of the hand.
	bool Grasp(const mjModel* m, mjData* d, graspType type);

protected:

private:
	double initialPoses[DLR_JOINT_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
//	double initialPoses[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
	double finalPoses[DLR_JOINT_COUNT] = {0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1, 0, 1, 1, 1};
//	double finalPoses[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
};


} //namespace grasp
#endif /* GRASP_CONTROLLER_DLRHANDCONTROLLER_H_ */

/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef GRASP_CONTROLLER_MPLHANDCONTROLLER_H_
#define GRASP_CONTROLLER_MPLHANDCONTROLLER_H_

#include "HandControllerInterface.h"
#include "stdio.h"

namespace Grasp{

class MPLHandController: public HandControllerInterface {

public:
	MPLHandController();
	~MPLHandController();

	void ComputeTrajectory();

	// Set the pose of the hand.
	void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q, float* jointAngles);

protected:


private:
	double initialPoses[13] = {0, 0, -0.25, 1.6, 0.8, 0, 0, 0.34, 0, 0, 0, 0.34, 0};
	double finalPoses[13] = {0, 0, 0, 0, 0, 1, 0.2, 0, 1.2, 1.2, 1.2, 0, 1.2};
};


} //namespace grasp
#endif /* SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_ */

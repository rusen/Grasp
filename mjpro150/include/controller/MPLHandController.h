/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_
#define SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_

#include "HandControllerInterface.h"
#include "stdio.h"

namespace Grasp{

class MPLHandController: public HandControllerInterface {

public:
	MPLHandController();
	~MPLHandController();

	void ComputeTrajectory();

	// Set the pose of the hand.
	void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q);

	// Set Initial pose of the hand.
	bool Grasp(const mjModel* m, mjData* d, graspType type);

	int getCounter() const;
	void setCounter(int counter = 0);

protected:


private:
	int counter = 0;
	double initialPoses[13] = {0, 0, -0.25, 1.6, 0.8, 0, 0, 0.34, 0, 0, 0, 0.34, 0};
	double finalPoses[13] = {0, 0, 0, 0, 0, 1, 0.2, 0, 1.2, 1.2, 1.2, 0, 1.2};
};


} //namespace grasp
#endif /* SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_ */

/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_
#define GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_

#include "mujoco.h"

namespace Grasp{

class HandControllerInterface {

public:
	HandControllerInterface();
	virtual ~HandControllerInterface();

	// Trajectory
	mjtNum** ComputeTrajectory(const mjModel* m, const mjData* d);

	// Grip functions
	virtual void GraspFirm(const mjModel* m, mjData* d);

	// Getters/setters
	int getState() const;
	void setState(int state = 1);

protected:
	int state = 1;

};


} //namespace grasp
#endif /* GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_ */

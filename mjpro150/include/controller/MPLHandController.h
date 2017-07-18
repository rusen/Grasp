/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_
#define SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_

#include "HandControllerInterface.h"

namespace Grasp{

class MPLHandController: public HandControllerInterface {

public:
	MPLHandController();
	~MPLHandController();

	void ComputeTrajectory();

	// Close the hand
	void GraspFirm(const mjModel* m, mjData* d);

protected:


private:


};


} //namespace grasp
#endif /* SAMPLE_CONTROLLER_MPLHANDCONTROLLER_H_ */

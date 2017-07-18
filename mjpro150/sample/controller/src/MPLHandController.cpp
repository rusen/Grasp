/*
 * MPLHandController.cpp
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#include "MPLHandController.h"

namespace Grasp{

MPLHandController::MPLHandController() {
	// TODO Auto-generated constructor stub

}

MPLHandController::~MPLHandController() {
	// TODO Auto-generated destructor stub
}




// A full hand grasp.
void MPLHandController::GraspFirm(const mjModel* m, mjData* d){
	if (d->ctrl[4] < 0.99)
		for (int itr = 3; itr < 13; itr++){
			d->ctrl[itr] += 0.001;
		}
	else
		state = 4;
}

}

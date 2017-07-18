/*
 * HandControllerInterface.cpp
 *
 *  Created on: 14 Jul 2017
 *      Author: rusi
 */

#include "HandControllerInterface.h"
namespace Grasp{
HandControllerInterface::HandControllerInterface() {
	// TODO Auto-generated constructor stub

}

HandControllerInterface::~HandControllerInterface() {
	// TODO Auto-generated destructor stub
}

void HandControllerInterface::GraspFirm(const mjModel* m, mjData* d){


}

int HandControllerInterface::getState() const {
	return this->state;
}

void HandControllerInterface::setState(int state) {
	this->state = state;
}

}

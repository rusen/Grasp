/*
 * HandControllerInterface.cpp
 *
 *  Created on: 14 Jul 2017
 *      Author: rusi
 */

#include <controller/HandControllerInterface.h>
namespace Grasp{
HandControllerInterface::HandControllerInterface() {
	// TODO Auto-generated constructor stub

}

HandControllerInterface::~HandControllerInterface() {
	// TODO Auto-generated destructor stub
}

int HandControllerInterface::getCounter() const {
	return counter;
}

void HandControllerInterface::setCounter(int counter) {
	this->counter = counter;
}

}

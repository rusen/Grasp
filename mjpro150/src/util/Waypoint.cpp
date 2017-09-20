/*
 * Waypoint.cpp
 *
 *  Created on: 28 Aug 2017
 *      Author: rusi
 */

#include "util/Waypoint.h"
#include <iostream>

namespace Grasp {

Waypoint::Waypoint() {
	jointCount = 20;
	jointAngles = new float[jointCount];
}

Waypoint::~Waypoint() {
	// TODO Auto-generated destructor stub
	delete[] jointAngles;
}

Waypoint::Waypoint (const Waypoint &obj) {
	jointCount = obj.jointCount;
	jointAngles = new float[jointCount];
	pos = obj.pos;
	quat = obj.quat;

	for (int i = 0; i<jointCount; i++)
		jointAngles[i] = obj.jointAngles[i];
}

void Waypoint::print(){
	std::cout<<"Printing Waypoint"<<std::endl;
	std::cout<<" ******** "<<std::endl;
	std::cout<<"JOINT COUNT:"<<jointCount<<std::endl;
	std::cout<<"WRIST POS:"<<pos[0]<<" "<<pos[1]<<" "<<pos[2]<<std::endl;
	std::cout<<"WRIST QUAT:"<<quat[0]<<" "<<quat[1]<<" "<<quat[2]<<" "<<quat[3]<<std::endl;
	std::cout<<"Fingers:";
	for (int i = 0; i<jointCount; i++){
		std::cout<<jointAngles[i]<<" ";
	}
	std::cout<<std::endl<<" ******** "<<std::endl;

}

void Waypoint::SetInitialPose (const double * pose){
	for (int i = jointCount; i<jointCount; i++){
		jointAngles[i] = (float) pose[i];
	}
}

} /* namespace Grasp */

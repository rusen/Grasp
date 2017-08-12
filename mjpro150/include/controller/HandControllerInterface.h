/*
 * MPLHandController.h
 *
 *  Created on: 7 Jul 2017
 *      Author: rusi
 */

#ifndef GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_
#define GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_

#include "mujoco.h"
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Grasp{

enum graspType {initialGrasp, finalGrasp};

class HandControllerInterface {

public:
	HandControllerInterface();
	virtual ~HandControllerInterface();

	// Set pose of the hand
	virtual void SetPose(const mjModel* m, mjData* d, glm::vec3 pos, glm::quat q)=0;

	// Grip function
	virtual bool Grasp(const mjModel* m, mjData* d, graspType type)=0;

	// Getters/setters
	int getState() const;
	void setState(int state = 1);

	int getCounter() const;
	void setCounter(int counter = 0);

protected:
	int counter = 0;
	int jointCount = 0;
	int counterLimit = 300;
};


} //namespace grasp
#endif /* GRASP_CONTROLLER_HANDCONTROLLERNTERFACE_H_ */

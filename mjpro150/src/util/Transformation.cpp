#include "util/Path.h"
#include <math.h>       /* floor */
#include <iostream>

namespace Grasp {

Transformation::Transformation(Eigen::Vector3f gazeDir, Eigen::Vector3f camPos, bool forward){
	// Get up, right vectors as well.
    // Find right and up vectors
	gazeDir.normalize();
    Eigen::Vector3f tempUp(0, 0, 1), normRight, viewUp;
    normRight = gazeDir.cross(tempUp);
    normRight.normalize();
    viewUp = normRight.cross(gazeDir);
    viewUp.normalize();

    // Create transformation matrices
	Eigen::Matrix4f cameraTraM, cameraRotM;
	cameraRotM.setZero(), cameraTraM.setZero();
	Eigen::Matrix3f camFrame;
	camFrame.setZero();

	// Translation
	cameraTraM(0,0) = 1, cameraTraM(1,1) = 1, cameraTraM(2,2) = 1, cameraTraM(3,3) = 1;
	cameraTraM(0,3) = -camPos(0), cameraTraM(1,3) = -camPos(1), cameraTraM(2,3) = -camPos(2);

	// Rotation
	cameraRotM(0,0) = normRight(0), cameraRotM(0,1) = normRight(1), cameraRotM(0,2) = normRight(2);
	cameraRotM(1,0) = viewUp(0), cameraRotM(1,1) = viewUp(1), cameraRotM(1,2) = viewUp(2);
	cameraRotM(2,0) = gazeDir(0), cameraRotM(2,1) = gazeDir(1), cameraRotM(2,2) = gazeDir(2);
	cameraRotM(3,3) = 1;
	camFrame(0,0) = cameraRotM(0,0), camFrame(0,1) = cameraRotM(0,1), camFrame(0,2) = cameraRotM(0,2);
	camFrame(1,0) = cameraRotM(1,0), camFrame(1,1) = cameraRotM(1,1), camFrame(1,2) = cameraRotM(1,2);
	camFrame(2,0) = cameraRotM(2,0), camFrame(2,1) = cameraRotM(2,1), camFrame(2,2) = cameraRotM(2,2);

	// Final transformation matrices/quats
	Eigen::Quaternionf trQuat = Eigen::Quaternionf(camFrame).inverse();
	trQuat.normalize();
	Eigen::Matrix4f trM = cameraRotM * cameraTraM;

	// If the calculation needs to be backwards, take inverse
	if (!forward)
	{
		tQuat = trQuat.inverse();
		tM = trM.inverse();
	}
	else
	{
		tM = trM;
		tQuat = trQuat;
	}
}

Transformation::~Transformation(){}

}

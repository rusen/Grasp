/*
 * GraspPlanner.cpp
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#include <planner/GraspPlanner.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <thread>

namespace Grasp {

void copyArray(glm::vec3 d, double*s, int c)
{
	for (int i = 0; i<c; i++)
		d[i] = s[i];
}

GraspPlanner::GraspPlanner() {

    // Create and start kinect simulator.
	CameraInfo camInfo;
    Simulator = new Simulate(camInfo, "text", "text2");
//    std::thread camThread(getDepthData, window, m, d, Simulator);

	// Allocate space for trajectory arrays.
	initialApproachPos = new glm::vec3[approachCounterLimit];
	initialApproachQuat = new glm::quat[approachCounterLimit];
	finalApproachPos = new glm::vec3[finalApproachCounterLimit];

	// TODO:: Start from random initial position. This'll change!
    srand (time(NULL));
 //   glm::vec3 initPosition = glm::vec3(((double)(rand()%100) - 50)/200, 0, 0.5 + ((double)(rand()%100) - 50)/200);
    glm::vec3 initPosition = glm::vec3(0, -1, 1);

    initialApproachPos[0] = initPosition;
    float num1 = ((float) (rand()%360))/(2*3.1416);
    float num2 = ((float) (rand()%360))/(2*3.1416);
    float num3 = ((float) (rand()%360))/(2*3.1416);
//    initDir = glm::vec3(num1, num2, num3);
    initDir = glm::vec3(0, 0, 1);
    initialApproachQuat[0] = glm::quat(initDir);

	// TODO: For now, we specify the pre-grasp location. This will be obviously changed.
	glm::vec3 finalPosition = glm::vec3(0.03, -0.11, 0.275);
	finalDir = glm::vec3(0.0f, 0.0f, 0.85f);
	initialApproachPos[approachCounterLimit-1] = finalPosition;
	initialApproachQuat[approachCounterLimit-1] = glm::quat(finalDir);

	// TODO: Specify pre-grasp to grasp points here as well.
	finalApproachPos[0] = glm::vec3(0.03, -0.11, 0.275);
	finalApproachPos[finalApproachCounterLimit-1] = glm::vec3(0.03, -0.11, 0.170);

}

GraspPlanner::~GraspPlanner() {
	delete initialApproachPos;
	delete initialApproachQuat;
	delete finalApproachPos;
}

glm::vec3 getPt( glm::vec3 n1 , glm::vec3 n2 , float perc )
{
    glm::vec3 diff = n2 - n1;
    return n1 + ( diff * perc );
}

void GraspPlanner::ComputeTrajectory(){
	// First, we need to interpolate positions.
	// For now, we do not use any kinematics, so we will work with Bezier curves for a realistic path.
	// Middle points in the bezier curve are assigned randomly.

    float num1 = ((float) (rand()%100))/500 - 0.05f;
    float num2 = ((float) (rand()%100))/500 - 0.05f;
    float num3 = ((float) (rand()%100))/500 + 0.4f;
    p1 = glm::vec3{num1, num2, num3};

    num1 = ((float) (rand()%100))/500 - 0.05f;
	num2 = ((float) (rand()%100))/500 - 0.05f;
    num3 = ((float) (rand()%100))/500 + 0.4f;
    p2 = glm::vec3{num1, num2, num3};

	// We have the initial and end points of a trajectory.
	// Now, we're off to interpolating.

	glm::vec3 p0 = initialApproachPos[0];
	glm::vec3 p3 = initialApproachPos[approachCounterLimit-1];

	for( int itr = 1 ; itr < (approachCounterLimit-1) ; itr ++ )
	{
		float i = ((float)itr)/approachCounterLimit;

	    // The Green Lines
	    glm::vec3 a = getPt( p0 , p1 , i );
	    glm::vec3 b = getPt( p1 , p2 , i );
	    glm::vec3 c = getPt( p2 , p3 , i );

	    // The Blue Line
	    glm::vec3 m = getPt( a , b , i );
	    glm::vec3 n = getPt( b , c , i );

	    // The Black Dot
	    glm::vec3 p = getPt( m , n , i );
	    initialApproachPos[itr] = p;
	}

	// Secondly, we will interpolate the rotations.
	for (int i = 0; i<approachCounterLimit; i++){
		initialApproachQuat[i] = glm::slerp(initialApproachQuat[0],
				initialApproachQuat[approachCounterLimit-1], (float)i/((float)approachCounterLimit-1));
	}

	// TODO: Final approach is vastly simplified, and this will need to change.
	for (int i = 1; i<(finalApproachCounterLimit-1); i++){
		float ratio1 = ((float) i)/((float) (finalApproachCounterLimit-1));
		float ratio2 = ((float) (finalApproachCounterLimit - (i + 1)))/((float) (finalApproachCounterLimit-1));
		finalApproachPos[i] = finalApproachPos[0]*ratio2 + finalApproachPos[finalApproachCounterLimit-1]*ratio1;
	}
}

bool GraspPlanner::FollowTrajectory(const mjModel* m, mjData* d, approachType type){

	// Set new positions for the hand.
	if (type == initialApproach){
		controller.SetPose(m, d, initialApproachPos[counter], initialApproachQuat[counter]);
	}
	else
	{
		d->mocap_pos[0] = finalApproachPos[counter][0];
		d->mocap_pos[1] = finalApproachPos[counter][1];
		d->mocap_pos[2] = finalApproachPos[counter][2];
	}

	// Increase counter for moving.
	counter++;
	int limit = 0;
	if (type == initialApproach)
		limit = approachCounterLimit;
	else
		limit = finalApproachCounterLimit;

	// If total number of steps have been performed, we move on.
	if (counter>=limit)
		return true;
	else return false;

}

// Main function that plans a grasp from initial position to the final trajectory.
// It's a state driven function since it is called in every simulation step.
void GraspPlanner::PerformGrasp(const mjModel* m, mjData* d, HandControllerInterface* handController){

	/*
	// Print positions of the bodies.
	for (int j=0;j<3;j++){
		std::cout<<d->xpos[(m->nbody-1) * 3 + j]<<" ";
	}
	std::cout<<std::endl;
	*/

	bool success = false;
	switch (graspState)
	{
	case initial:
		// Set initial position of the hand.
		if (!startFlag)
		{
			startFlag = true;
			controller.SetPose(m, d, initialApproachPos[0], initialApproachQuat[0]);
		}

		success = handController->Grasp(m, d, initialGrasp);
		if (success){
			ComputeTrajectory();
			startFlag = false;
			graspState = collectingData;
		}
		break;
	case collectingData:
		// Data collection from the kinect camera. We start a new thread with the camera acquisition function, and wait for it to finish.
		if (!startFlag)
		{
			startFlag = true;
	//		CollectData(Simulator, m, d, camSize);
			std::thread camThread(CollectData, Simulator, m, d, camSize);
		}

		// Processing done, move on.
		if (finishFlag)
		{
			finishFlag = false;
			startFlag = false;
			graspState = approaching;
		}

		break;
	case approaching:
		success = FollowTrajectory(m, d, initialApproach);
			if (success)
				graspState = atPreGraspLocation;
			break;
		break;
	case atPreGraspLocation: // A good point to put a pause and wait for user input.
		graspState = atFinalApproach;
		counter = 0;
		break;
	case atFinalApproach:
		success = FollowTrajectory(m, d, finalApproach);
		if (success)
			graspState = readyToGrasp;
		break;
	case readyToGrasp:
		graspState = grasping;
		counter = 0;
		break;
	case grasping:
		success = handController->Grasp(m, d, finalGrasp);
		if (success)
			graspState = lifting;
			break;
	case lifting:
		d->mocap_pos[2] += 0.001;
		counter++;
		if (counter > 500)
			graspState = done;
		break;
	case done:
		break;
	}

}

} /* namespace Grasp */

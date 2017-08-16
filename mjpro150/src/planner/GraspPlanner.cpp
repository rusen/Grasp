/*
 * GraspPlanner.cpp
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#include "mujoco.h"
#include <planner/GraspPlanner.h>
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Grasp {

void copyArray(glm::vec3 d, double*s, int c)
{
	for (int i = 0; i<c; i++)
		d[i] = s[i];
}

GraspPlanner::GraspPlanner() {

    // Create and start kinect simulator.
	CameraInfo camInfo;
    Simulator = new Simulate();

	// Allocate space for trajectory arrays.
	initialApproachPos = new glm::vec3[approachCounterLimit];
	initialApproachQuat = new glm::quat[approachCounterLimit];
	preApproachPos = new glm::vec3[approachCounterLimit];
	preApproachQuat = new glm::quat[approachCounterLimit];
	finalApproachPos = new glm::vec3[finalApproachCounterLimit];

	// TODO:: Start from random initial position. This'll change!
    srand (time(NULL));
    glm::vec3 initPosition = glm::vec3(((double)(rand()%100) - 50)/200,
    		((double)(rand()%100) - 50)/200, 0.75 + ((double)(rand()%100) - 50)/200);

    // Set initial direction and pose.
    initialApproachPos[0] = initPosition;
    float num1 = (((float) (rand()%360)) / 360) * (2*3.1416);
    float num2 = (((float) (rand()%360)) / 360) * (2*3.1416);
    float num3 = (((float) (rand()%360)) / 360) * (2*3.1416);
    initDir = glm::vec3(num1, num2, num3);
    initialApproachQuat[0] = glm::quat(initDir);

    // Calculate a location and orientation for placing the depth cam.
    float radialR = (((float) (rand()%360))/360) * (2*3.1416);
    glm::vec3 handPositionNoise(((float) (rand()%100) - 50)/400, ((float) (rand()%100) - 50)/400, ((float) (rand()%100) - 50)/400);
    glm::vec3 gazePosition = {((float) (rand()%100) - 50)/500, ((float) (rand()%100) - 50)/500, -0.5};
    glm::vec3 handPosition(cos(radialR) * 0.5, sin(radialR) * 0.5, 0.4);
    handPosition += handPositionNoise;

    // Calculate gaze dir.
    gazeDir = normalize(gazePosition - handPosition);

    // Next, we find the view-right vector (right hand vector)
    glm::vec3 tempUp(0, 0, 1), rightDir, upDir;
    rightDir = normalize(glm::cross(gazeDir, tempUp));
    upDir = normalize(glm::cross(rightDir, gazeDir));

    // Calculate transformation matrix
    glm::mat3 RM(rightDir[0], rightDir[1], rightDir[2],
    			 gazeDir[0], gazeDir[1], gazeDir[2],
				 upDir[0], upDir[1], upDir[2]);

    // Find camera pos.
    cameraPos = handPosition + 0.2f * upDir;

    // Fill in the endpoint of initial approach, and start point of pre-grasp approach.
    glm::quat rotation3 = glm::toQuat(RM);
    initialApproachPos[approachCounterLimit-1] = handPosition;
    initialApproachQuat[approachCounterLimit-1] = glm::inverse(rotation3);
    preApproachPos[0] = handPosition;
    preApproachQuat[0] = initialApproachQuat[approachCounterLimit-1];

	// TODO: For now, we specify the pre-grasp location. This will be obviously changed.
	glm::vec3 finalPosition = glm::vec3(0.03, -0.11, 0.275);
	finalDir = glm::vec3(0.85f, 0.0f, 0.0f);
	preApproachPos[approachCounterLimit-1] = finalPosition;
	preApproachQuat[approachCounterLimit-1] = glm::quat(finalDir);

	// TODO: Specify pre-grasp to grasp points here as well.
	finalApproachPos[0] = finalPosition;
	finalApproachPos[finalApproachCounterLimit-1] = glm::vec3(0.03, -0.11, 0.20);
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

	p0 = preApproachPos[0];
	p3 = preApproachPos[approachCounterLimit-1];

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
	    preApproachPos[itr] = p;
	}

	// Secondly, we will interpolate the rotations.
	for (int i = 0; i<approachCounterLimit; i++){
		initialApproachQuat[i] = glm::slerp(initialApproachQuat[0],
				initialApproachQuat[approachCounterLimit-1], (float)i/((float)approachCounterLimit-1));
		preApproachQuat[i] = glm::slerp(preApproachQuat[0],
				preApproachQuat[approachCounterLimit-1], (float)i/((float)approachCounterLimit-1));
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
	else if (type == preApproach)
	{
		controller.SetPose(m, d, preApproachPos[counter], preApproachQuat[counter]);
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
	if (type == initialApproach || type == preApproach)
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
void GraspPlanner::PerformGrasp(const mjModel* m, mjData* d){

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
		success = controller.Grasp(m, d, initialGrasp);
		if (success){
			ComputeTrajectory();
			startFlag = false;
			graspState = atDataApproach;
		}
		break;
	case atDataApproach:

		success = FollowTrajectory(m, d, initialApproach);
		if (success)
		{
			graspState = collectingData;
			counter = 0;
		}
		break;
	case collectingData:
		if (counter < 300)
		{
			counter++;
			break;
		}
		else
			counter = 0;

		// Data collection from the kinect camera. We start a new thread with the camera acquisition function, and wait for it to finish.
		if (!startFlag)
		{
			startFlag = true;
			pointCloud = CollectData(Simulator, m, d, depthBuffer, cameraPos, gazeDir, camSize, &finishFlag);
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
		success = FollowTrajectory(m, d, preApproach);
			if (success)
			{
				graspState = atPreGraspLocation;
				counter = 0;
			}
		break;
	case atPreGraspLocation: // A good point to put a pause and wait for user input.
		graspState = atFinalApproach;
		counter = 0;
		break;
	case atFinalApproach:
		success = FollowTrajectory(m, d, finalApproach);
		if (success){
			graspState = readyToGrasp;
			counter = 0;
		}
		break;
	case readyToGrasp:
		graspState = grasping;
		counter = 0;
		break;
	case grasping:
		success = controller.Grasp(m, d, finalGrasp);
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

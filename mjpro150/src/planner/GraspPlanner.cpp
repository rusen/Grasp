/*
 * GraspPlanner.cpp
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#include "mujoco.h"
#include <util/Connector.h>
#include <planner/GraspPlanner.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cstdlib>
#include <cerrno>

namespace Grasp {

void copyArray(glm::vec3 d, double*s, int c)
{
	for (int i = 0; i<c; i++)
		d[i] = s[i];
}

// set one frame, from global frame counter
void GraspPlanner::SetFrame(const mjModel* m, mjData * d)
{
    d->time = (mjtNum)data[0];
    mju_f2n(d->qpos, data+1, m->nq);
    mju_f2n(d->qvel, data+1+m->nq, m->nv);
    mju_f2n(d->ctrl, data+1+m->nq+m->nv, m->nu);
    mju_f2n(d->mocap_pos, data+1+m->nq+m->nv+m->nu, 3*m->nmocap);
    mju_f2n(d->mocap_quat, data+1+m->nq+m->nv+m->nu+3*m->nmocap, 4*m->nmocap);
    mju_f2n(d->sensordata, data+1+m->nq+m->nv+m->nu+7*m->nmocap, m->nsensordata);
}

GraspPlanner::GraspPlanner(const char * dropboxBase) {

	// Create file paths.
	fileId[0] = 0;
    strcat(fileId, "XXXXXX");
    mktemp(fileId);
	char prefix[1000];
	strcpy(dropboxFolder, dropboxBase);
	strcpy(prefix, dropboxBase);
	strcat(prefix, "/data/");
	strcat(prefix, fileId);
	strcat(prefix, "/");
	strcpy(baseFolder, "./tmp/");
	strcat(baseFolder, "data/");
	strcat(baseFolder, fileId);
	strcat(baseFolder, "/");
	char localPrefix[1000];
	strcpy(localPrefix, baseFolder);
	boost::filesystem::create_directories(baseFolder);
	boost::filesystem::create_directories(prefix);
	strcat(prefix, fileId);
	strcat(localPrefix, fileId);
	logFile [0] = debugLogFile[0] = pointFile [0] = rgbFile [0] = depthFile[0] = resultFile[0] = trajectoryFile[0] = 0; // Set to ""
	strcat(logFile, localPrefix); strcat(debugLogFile, localPrefix); strcat(pointFile, localPrefix); strcat(rgbFile, localPrefix); strcat(depthFile, localPrefix); strcat(resultFile, localPrefix); strcat(trajectoryFile, prefix);
	strcat(logFile, ".log"); strcat(debugLogFile, "_debug.log"); strcat(pointFile, ".pcd"); strcat(rgbFile, "_rgb.png"); strcat(depthFile, "_depth.png"); strcat(resultFile, ".gd"); strcat(trajectoryFile, ".trj");
	logStream = new std::ofstream(debugLogFile);

    // Create and start kinect simulator.
	CameraInfo camInfo;
    Simulator = new Simulate();
    strcpy(Simulator->cloudFile,pointFile);
    strcpy(Simulator->rgbFile,rgbFile);
    strcpy(Simulator->depthFile,depthFile);

    // Calculate a location and orientation for placing the depth cam.
//    srand(randSeed);
    float radialR = (((float) (rand()%360))/360.0f) * (2*3.1416);
    glm::vec3 gazePosition = {((float) (rand()%100) - 50)/1000.0f, ((float) (rand()%100) - 50)/1000.0f, -0.35};
    glm::vec3 handPosition(cos(radialR) * (0.5 + ((float) (rand()%100) - 50)/1000.0f), sin(radialR) * (0.5 + ((float) (rand()%100) - 50)/1000.0f), ((float) (rand()%100) - 50)/1000.0f);
    (*logStream)<<"Position of the hand:"<<handPosition[0]<<" "<<handPosition[1]<<" "<<handPosition[2]<<std::endl;

    // Calculate gaze dir.
    gazeDir = normalize(gazePosition - handPosition);

    // Next, we find the view-right vector (right hand vector)
    glm::vec3 tempUp(0, 0, 1), rightDir, upDir;
    rightDir = normalize(glm::cross(gazeDir, tempUp));
    upDir = normalize(glm::cross(rightDir, gazeDir));

    // Find camera pos.
    cameraPos = handPosition + 0.1f * upDir - 0.0135f * rightDir;
}

GraspPlanner::~GraspPlanner() {
	delete Simulator;
	char prefix[1000];
	strcpy(prefix, dropboxFolder);
	strcat(prefix, "/data/");
	strcat(prefix, fileId);

    if(boost::filesystem::exists(prefix))
    {
       boost::filesystem::remove_all(prefix);
    }
    logStream->close();
    delete logStream;

	if (finalApproach != NULL)
		delete finalApproach;
	fclose(trjFP);
}

glm::vec3 getPt( glm::vec3 n1 , glm::vec3 n2 , float perc )
{
    glm::vec3 diff = n2 - n1;
    return n1 + ( diff * perc );
}

void GraspPlanner::ReadTrajectory(){
	// Reads the next trajectory from the file, and sets path variable.
	float extraGrip = 0.5236; // 0.5236 for 30 degrees
	int wpCount = 0;
	float likelihood = 0;
	int graspType = 0;
	float wristPos[3];
	float wristQuat[4];
	float fingerPos[20];
	fread(&likelihood, 4, 1, trjFP);
	fread(&graspType, 4, 1, trjFP);
	fread(&wpCount, 4, 1, trjFP);
	finalApproach = new Path(wpCount+2);

	// Read trajectory waypoint by waypoint.
	for (int i = 1; i< wpCount+2; i++)
	{
		if (i<wpCount+1)
		{
			fread(wristPos, 4, 3, trjFP);
			fread(wristQuat, 4, 4, trjFP);
			fread(fingerPos, 4, 20, trjFP);
		}

		float multiplyFactor = 0, maxVal = 0;
		float differences[20];
		if (i == wpCount+1)
		{
			for (int k = 0; k<20; k++){
				float newVal = finalApproach->waypoints[i-1].jointAngles[k] -
						finalApproach->waypoints[i-2].jointAngles[k];
				differences[k] = newVal;
				if (newVal > maxVal)
					maxVal = newVal;
			}
			multiplyFactor = extraGrip / maxVal;
		}
		else
		{
			for (int k = 0; k<20; k++)
				differences[k] = 0;
		}

		// Read waypoints.
		finalApproach->waypoints[i].pos[0] = wristPos[0] - 0.5, finalApproach->waypoints[i].pos[1] = wristPos[1], finalApproach->waypoints[i].pos[2] = wristPos[2];
		finalApproach->waypoints[i].quat.x = wristQuat[0];
		finalApproach->waypoints[i].quat.y = wristQuat[1];
		finalApproach->waypoints[i].quat.z = wristQuat[2];
		finalApproach->waypoints[i].quat.w = wristQuat[3];

		for (int k = 0; k<20; k++)
		{
			if (!(k % 4))
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k];
			else if (k%4 == 2 || k%4 == 3)
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + multiplyFactor * differences[k];
			else if (k < 4)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + multiplyFactor * differences[k];
			}
			else if (k < 8)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + multiplyFactor * differences[k];
			}
			else if (k < 12)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + multiplyFactor * differences[k];
			}
			else if (k < 16)
			{
				finalApproach->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.087266) + multiplyFactor * differences[k];
			}
			else{
				finalApproach->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.1745) + multiplyFactor * differences[k];
			}
		}
	}

	// Set the first waypoint as an initial approach from outside.
	finalApproach->waypoints[0].pos[0] = 2 * finalApproach->waypoints[1].pos[0];
	finalApproach->waypoints[0].pos[1] = 2 * finalApproach->waypoints[1].pos[1];
	finalApproach->waypoints[0].pos[2] = 2 * (finalApproach->waypoints[1].pos[2] + 0.35) - 0.35;
	finalApproach->waypoints[0].quat.x = finalApproach->waypoints[1].quat.x;
	finalApproach->waypoints[0].quat.y = finalApproach->waypoints[1].quat.y;
	finalApproach->waypoints[0].quat.z = finalApproach->waypoints[1].quat.z;
	finalApproach->waypoints[0].quat.w = finalApproach->waypoints[1].quat.w;
	for (int k = 0; k<20; k++)
		finalApproach->waypoints[0].jointAngles[k] = 0;

}

bool GraspPlanner::FollowTrajectory(const mjModel* m, mjData* d, float yOffset){

	// Set pose of the hand.
	Waypoint wp = finalApproach->Interpolate(counter);

	// Add y offset, if requested
	wp.pos[1] += yOffset;

	// Set hand position.
	controller.SetPose(m, d, wp.pos, wp.quat, wp.jointAngles);

	// Increase counter for moving.
	counter++;

	// If total number of steps have been performed, we move on.
	if (counter>=finalApproach->steps)
		return true;
	else return false;

}

state GraspPlanner::getGraspState() const {
	return graspState;
}

void GraspPlanner::setGraspState(state graspState = collectingData) {
	this->graspState = graspState;
}

// Main function that plans a grasp from initial position to the final trajectory.
// It's a state driven function since it is called in every simulation step.
void GraspPlanner::PerformGrasp(const mjModel* m, mjData* d, mjtNum * stableQpos, mjtNum * stableQvel, mjtNum * stableCtrl, mjvScene *scn, mjrContext *con){

	bool success = false;
	switch (graspState)
	{
	case collectingData:

		// Data collection from the kinect camera. We start a new thread with the camera acquisition function, and wait for it to finish.
		if (!startFlag)
		{
			startFlag = true;
			CollectData(Simulator, m, d, scn, con, rgbBuffer, depthBuffer, cameraPos, gazeDir, camSize, minPointZ, &finishFlag, logStream);
			if (!boost::filesystem::exists(Simulator->cloudFile))
			{
				graspState = done;
				break;
			}
		}
		// Processing done, move on.
		if (finishFlag)
		{
			finishFlag = false;
			startFlag = false;
			graspState = planning;
		}
		break;
	case planning:
		if (!startFlag){

			// Not enough points?
			if (Simulator->cloud->size() < minPointCount)
			{
				// Point cloud empty or number of points too small. Next!
				graspState = done;
				break;
			}

			// First, upload the pcd file to the web server (where we expect it to be deleted).
			success = Connector::UploadFileToDropbox(fileId, pointFile, dropboxFolder);

			if (!success)
			{
				(*logStream)<< "Could not upload file " << fileId << std::endl;
				graspState = done;
				break;
			}

			uploadTime = time(NULL);
			startFlag = true;
			success = false;
			(*logStream)<< "Waiting for trajectories." << std::endl;
		}
		else
		{
			// Get trajectory data from server.
			success = Connector::DownloadFileFromDropbox(trajectoryFile);
			if (success)
			{
				trjFP = fopen(trajectoryFile, "rb");
				fread(&numberOfGrasps, 4, 1, trjFP);
				(*logStream)<< "Number of grasps:" << numberOfGrasps << std::endl; d
			}
			else
			{
				time_t timePassed = time(NULL) - uploadTime;
				if (timePassed > trajectoryTimeout)
				{
					graspState = done;
				}

			}
		}

		// All good! Move on.
		if (success)
		{
			startFlag = false;
			(*logStream)<<"Moving on to grasping."<<std::endl;

			// Switch to grasping state.
	    	graspState = pregrasp;
		}
		break;
	case pregrasp:

		// If enough grasps have been performed, exit.
		if (graspCounter >= numberOfGrasps)
		{
			graspState = done;
			break;
		}

		// Read trajectory
		ReadTrajectory();

		// Increase counter and move on.
		graspCounter++;
		graspState = checkingCollision;
		counter = 0;
		finishFlag = false;
		break;
	case checkingCollision:
		// Check a number of viewpoints.
		if (collisionCounter < collisionPoints){
			if (!collisionSet)
			{
				counter = round(((float) collisionCounter / float(collisionPoints-1)) * (float)(finalApproach->steps));
				FollowTrajectory(m, d, yOffset);
				collisionSet = true;
			}
			else
			{
				collisionSet = false; // toogle between follow trajectory and collision check in current state
				for (int i = 0; i<m->nconmax; i++)
				{
					// Check for hand-table contact.
					if ((d->contact[i].geom1 == 42 || d->contact[i].geom2 == 42) &&
						((d->contact[i].geom1 < 42 && d->contact[i].geom1 > 0) ||
						(d->contact[i].geom2 < 42 && d->contact[i].geom2 > 0)))
					{
						// Collision detected, abort!
						collisionCounter = 0;
						counter = 0;
						graspState = reset;
						hasCollided = true;
						break;
					}
				}

				// If no collision, move on.
				if (!hasCollided)
				{
					collisionCounter++;
				}
			}
		}
		else
		{
			collisionCounter = 0;
			counter = 0;
			collisionSet = false;
			graspState = reset;
		}
		break;
	case grasping:
		success = FollowTrajectory(m, d, 0);

		// If grasp complete, move on to lift the object.
		if (success){
			counter = 0;
			graspState = lifting; //lifting
		}
		break;
	case lifting:
		d->mocap_pos[2] += 0.001;
		counter++;
		if (counter > 1000){
			counter = 0;
			graspState = stand;
		}
		break;
	case stand:
		counter++;
		if (counter > 800){
			counter = 0;
			// If the total number of grasps has been reached, move on to the next phase. Otherwise, perform next grasp.
			graspState = reset;
		}
		break;
	case reset:
		if (counter < 1)
		{
			// Reset everything
			for (int i = 0; i<m->nq; i++)
				d->qpos[i] = stableQpos[i];
			for (int i = 0; i<m->nv; i++)
				d->qvel[i] = stableQvel[i];
			for (int i = 0; i<m->nu; i++)
				d->ctrl[i] = stableCtrl[i];
			counter++;
			mj_forward(m, d);
		}
		else
		{
			if (prevState  == checkingCollision && hasCollided)
			{
	//			(*logStream)<<"Collision! Moving on to the next grasp here."<<std::endl;
				graspState = pregrasp;
			}
			else if (prevState == stand)
			{
	//			(*logStream)<<"Successful grasp! Moving on to the next grasp."<<std::endl;
				graspState = pregrasp;
			}
			else
			{
		//		(*logStream)<<"Coming from "<<prevState<<". No collision in the checks. Perform grasp."<<std::endl;
				graspState = grasping;
			}
			counter = 0;
			hasCollided = false;
		}
		break;
	case done:
		break;
	}
	if (graspState != reset)
		prevState = graspState;
}

} /* namespace Grasp */

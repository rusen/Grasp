/*
 * GraspPlanner.cpp
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#include "mujoco.h"
#include <util/Connector.h>
#include <opencv2/opencv.hpp>
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

GraspPlanner::GraspPlanner(const char * dropboxBase, bool testFlag, int baseType) {

	// Save test flag and base type.
	this->testFlag = testFlag;
	this->baseType = baseType;

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

    // If testing, we don't need many angles.
    if (testFlag)
    	numberOfAngles = 1;

    // Fill collision state/grasp params array.
    for (int i = 0; i < 500; i++)
    {
    	collisionState[i] = -1;
    }

    // Find camera and gaze locations
    for (int i = 0; i<numberOfAngles; i++)
    {
		// Calculate a location and orientation for placing the depth cam.
		float radialR = RF * (2*PI);
		glm::vec3 gazePosition = {(RF-0.5) * 0.08, (RF-0.5) * 0.08, (-(0.24 + RF * 0.04))};
		glm::vec3 camPosition(cos(radialR) * (0.315 + RF * 0.03), sin(radialR) * (0.315 + RF * 0.03), RF * 0.03 + 0.07);

		// Calculate gaze dir.
		glm::vec3 tempGaze = normalize(gazePosition - camPosition);

		// Next, we find the view-right vector (right hand vector)
		glm::vec3 tempUp(0, 0, 1), rightDir, upDir;
		rightDir = normalize(glm::cross(tempGaze, tempUp));
		upDir = normalize(glm::cross(rightDir, tempGaze));

		// Find camera pos.
		glm::vec3 tempCameraPos = camPosition;

		// Add camera information to the arrays.
		cameraPosArr.push_back(tempCameraPos);
		gazeDirArr.push_back(tempGaze);

		// First point is the one shown to the camera.
		if (!i)
		{
			gazeDir = tempGaze;
			cameraPos = tempCameraPos;
		}
    }
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

    // Delete trajectories.
    for (int i = 0; i<numberOfGrasps; i++)
    {
    	if (finalApproachArr[i] != NULL)
    		delete finalApproachArr[i];
    }

	if (finalApproachArr != NULL)
		delete finalApproachArr;

	delete []resultArr;
}

glm::vec3 getPt( glm::vec3 n1 , glm::vec3 n2 , float perc )
{
    glm::vec3 diff = n2 - n1;
    return n1 + ( diff * perc );
}

void GraspPlanner::ReadTrajectories(int numberOfGrasps){
	finalApproachArr = new Path* [numberOfGrasps];
	for (int readCtr = 0; readCtr<numberOfGrasps; readCtr++)
	{
		// Reads the next trajectory from the file, and sets path variable.
		float extraGrip = 0.523; // 0.523 for 30 degrees
		int wpCount = 0;
		float likelihood = 0;
		int graspType = 0;
		float wristPos[3];
		float wristQuat[4];
		float fingerPos[20];
		fread(&likelihood, 4, 1, trjFP);
		fread(&graspType, 4, 1, trjFP);
		fread(&wpCount, 4, 1, trjFP);
		if (graspType < 0 || graspType > 9)
			graspType = 0;
		finalApproachArr[readCtr] = new Path(wpCount+2);
		finalApproachArr[readCtr]->graspType = graspType;

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
					float newVal = finalApproachArr[readCtr]->waypoints[i-1].jointAngles[k] -
							finalApproachArr[readCtr]->waypoints[i-2].jointAngles[k];
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
			finalApproachArr[readCtr]->waypoints[i].pos[0] = wristPos[0] - 0.5, finalApproachArr[readCtr]->waypoints[i].pos[1] = wristPos[1], finalApproachArr[readCtr]->waypoints[i].pos[2] = wristPos[2];
			finalApproachArr[readCtr]->waypoints[i].quat.x = wristQuat[0];
			finalApproachArr[readCtr]->waypoints[i].quat.y = wristQuat[1];
			finalApproachArr[readCtr]->waypoints[i].quat.z = wristQuat[2];
			finalApproachArr[readCtr]->waypoints[i].quat.w = wristQuat[3];

			for (int k = 0; k<20; k++)
			{
				if (!(k % 4))
				{
					// Sideways joints
					if (!k)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.05; // 0.087266
					else if (k == 4)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.043633;
					else if (k == 8)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k];
					else if (k == 12)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] - 0.043633;
					else if (k == 16)
						finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] - (0.0537);
				}
				else if (k%4 == 2 || k%4 == 3)
					// Middle joints
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + multiplyFactor * differences[k];
				else if (k < 4)
				{	// thumb
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.02908866667) + multiplyFactor * differences[k];
				}
				else if (k < 8)
				{	// index
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = fingerPos[k] + (0.07) + multiplyFactor * differences[k];
				}
				else if (k < 12)
				{	// middle
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.01) + multiplyFactor * differences[k];
				}
				else if (k < 16)
				{	// ring
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.06) + multiplyFactor * differences[k];
				}
				else{	// little
					finalApproachArr[readCtr]->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.160899) + multiplyFactor * differences[k];
				}
			}
		}

		// Set the first waypoint as an initial approach from outside.
		finalApproachArr[readCtr]->waypoints[0].pos[0] = 2 * finalApproachArr[readCtr]->waypoints[1].pos[0];
		finalApproachArr[readCtr]->waypoints[0].pos[1] = 2 * finalApproachArr[readCtr]->waypoints[1].pos[1];
		finalApproachArr[readCtr]->waypoints[0].pos[2] = 2 * (finalApproachArr[readCtr]->waypoints[1].pos[2] + 0.35) - 0.35;
		finalApproachArr[readCtr]->waypoints[0].quat.x = finalApproachArr[readCtr]->waypoints[1].quat.x;
		finalApproachArr[readCtr]->waypoints[0].quat.y = finalApproachArr[readCtr]->waypoints[1].quat.y;
		finalApproachArr[readCtr]->waypoints[0].quat.z = finalApproachArr[readCtr]->waypoints[1].quat.z;
		finalApproachArr[readCtr]->waypoints[0].quat.w = finalApproachArr[readCtr]->waypoints[1].quat.w;
		for (int k = 0; k<20; k++)
			finalApproachArr[readCtr]->waypoints[0].jointAngles[k] = 0;

		// Finally, save grasp parameter data
		std::vector<float> curParams = finalApproachArr[readCtr]->getGraspParams(resultArr[readCtr].gazeDir, resultArr[readCtr].camPos);
		graspParams.push_back(curParams);
	}
	fclose(trjFP);
}

bool GraspPlanner::FollowTrajectory(const mjModel* m, mjData* d, float yOffset){

	// Set pose of the hand.
	Waypoint wp = finalApproachArr[graspCounter-1]->Interpolate(counter);

	// Add y offset, if requested
	wp.pos[1] += yOffset;

	// Set hand position.
	controller.SetPose(m, d, wp.pos, wp.quat, wp.jointAngles);

	// Increase counter for moving.
	counter++;

	// If total number of steps have been performed, we move on.
	if (counter>=finalApproachArr[graspCounter-1]->steps)
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

			// Copy saved files into different names.
			char localPrefix[1000], imagePrefix[1000], tmpStr[1000];
			strcpy(localPrefix, baseFolder);
			strcat(localPrefix, fileId);
			strcat(localPrefix, "_");
			strcpy(imagePrefix, baseFolder);
			strcat(imagePrefix, "/views");

			if (!boost::filesystem::exists(imagePrefix))
				boost::filesystem::create_directories(imagePrefix);

			// Collect data from all angles.
			for (int i = numberOfAngles-1; i>-1; i--)
			{
				// Collect data
				CollectData(Simulator, m, d, scn, con, rgbBuffer, depthBuffer, cameraPosArr[i], gazeDirArr[i], camSize, minPointZ, &finishFlag, logStream, i);

				// Print aux info
				std::cout<<"Getting data from "<<i<<"th location."<<std::endl;
				std::cout<<cameraPosArr[i][0]<<" "<<cameraPosArr[i][1]<<" "<<cameraPosArr[i][2]<<std::endl;
				std::cout<<gazeDirArr[i][0]<<" "<<gazeDirArr[i][1]<<" "<<gazeDirArr[i][2]<<std::endl;

				// Crop and save file.
			    cv::Rect roi;
			    roi.x = 90;
			    roi.y = 10;
			    roi.width = 460;
			    roi.height = 460;

				// RGB
				strcpy(tmpStr, imagePrefix);
				strcat(tmpStr, "/");
				strcat(tmpStr, std::to_string(i).c_str());
				strcat(tmpStr, ".jpg");
				cv::Mat img = cv::imread(Simulator->rgbFile);
			    cv::Mat crop = img(roi);
			    cv::Mat dst;
			    cv::resize(crop, dst, cv::Size(224, 224), 0, 0, cv::INTER_CUBIC);
			    cv::imwrite(tmpStr, dst);

			    // DEPTH
				strcpy(tmpStr, imagePrefix);
				strcat(tmpStr, "/");
				strcat(tmpStr, std::to_string(i).c_str());
				strcat(tmpStr, ".png");
			    cv::Mat cropDepth = Simulator->scaled_im_(roi);
			    cv::Mat outDepth;
			    cv::flip(cropDepth, outDepth, -1);
			    cv::Mat dstDepth;
			    cv::resize(outDepth, dstDepth, cv::Size(224, 224), 0, 0, cv::INTER_NEAREST);
			    cv::imwrite(tmpStr, dstDepth);
			}
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

				// Allocate output array
				resultArr = new Grasp::GraspResult[numberOfGrasps];

				// Allocate a view for each grasp
				for (int i = 0; i<numberOfGrasps; i++)
				{
					int id = i; // unique view
					if (numberOfAngles < numberOfGrasps) // if not enough views, then randomly pick one
						id = rand()%numberOfAngles;
					resultArr[i].viewId = id;
					resultArr[i].camPos = cameraPosArr[id];
					resultArr[i].gazeDir = gazeDirArr[id];
				}

				// Read trajectories
				ReadTrajectories(numberOfGrasps);

				// Log/output
				(*logStream)<< "Number of grasps:" << numberOfGrasps << std::endl;
				std::cout<< "Number of grasps:" << numberOfGrasps << std::endl;
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
			std::cout<<"Moving on to grasping."<<std::endl;
			(*logStream)<<"Moving on to grasping."<<std::endl;

			// Switch to grasping state.
			prevState = stand;
	    	graspState = reset;
		}
		break;
	case pregrasp:

		// If enough grasps have been performed, exit.
		if (graspCounter >= numberOfGrasps)
		{
			graspState = done;
			break;
		}

		// Increase counter and move on.
		graspCounter++;
		graspState = checkingCollision;
		counter = 0;
		stableCounter = 0;
		finishFlag = false;
		break;
	case checkingCollision:

		// If collision has already been checked for this one, move on.
		if (collisionState[graspCounter-1] > -1)
		{
			// Reset
			stableCounter = 0;
			collisionCounter = 0;
			counter = 0;
			collisionSet = false;
			graspState = reset;

			// Mark hasCollided.
			if (!collisionState[graspCounter-1]) // No collision
			{
				hasCollided = false;
			}
			else // Collision
			{
				hasCollided = true;
			}
			break;
		}

		// Allow the hand to stabilize
		if (stableCounter < stableLimit)
		{
			FollowTrajectory(m, d, yOffset);
			counter = 0;
			stableCounter++;
			break;
		}

		// Check a number of points on the trajectory.
		if (collisionCounter < collisionPoints){
			if (!collisionSet)
			{
				counter = round(((float) collisionCounter / float(collisionPoints-1)) * (float)(finalApproachArr[graspCounter-1]->steps));
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
						stableCounter = 0;
						collisionCounter = 0;
						counter = 0;
						hasCollided = true;
						collisionState[graspCounter-1] = 1;
						graspState = reset;
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
			// Non-colliding grasp! Move on.
			stableCounter = 0;
			collisionCounter = 0;
			counter = 0;
			collisionSet = false;
			hasCollided = false;
			collisionState[graspCounter-1] = 0;
			numberOfNoncollidingGrasps++;
			graspState = reset;
		}
		break;
	case grasping:
		if (stableCounter < stableLimit)
		{
			FollowTrajectory(m, d, 0);
			counter = 0;
			stableCounter++;
			break;
		}
		else{
			success = FollowTrajectory(m, d, 0);
		}
		// If grasp complete, move on to lift the object.
		if (success){
			counter = 0;
			stableCounter = 0;
			graspState = lifting; //lifting
		}
		break;
	case lifting:
		if (testFlag)
			break;
		d->mocap_pos[2] += 0.000625;
		counter++;
		if (counter > 1600){
			counter = 0;
			graspState = stand;
		}
		break;
	case stand:
		counter++;
		if (counter > 1000){
			counter = 0;
			// If the total number of grasps has been reached, move on to the next phase. Otherwise, perform next grasp.
			graspState = reset;
		}
		break;
	case reset:
		if (counter < 50)
		{

			// Reset everything
			for (int i = 0; i<m->nq; i++)
				d->qpos[i] = stableQpos[i];
			if (testFlag)
				d->qpos[27] = d->qpos[27] + 1;
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
				// Collision! Moving on to the next grasp here.
				graspState = pregrasp;
			}
			else if (prevState == stand)
			{
				// Successful grasp! Moving on to the next grasp.
				graspState = pregrasp;
			}
			else
			{
				std::cout<<"Grasp #"<<graspCounter-1<<" starting. Grasp type:"<<finalApproachArr[graspCounter-1]->graspType<<std::endl;
				// No collision in the checks. Perform grasp.
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

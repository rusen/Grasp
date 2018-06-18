/*
 * GraspPlanner.cpp
 *
 *  Created on: 18 Jul 2017
 *      Author: rusi
 */

#include "mujoco.h"
#include <util/Connector.h>
#include <util/util.h>
#include <opencv2/opencv.hpp>
#include <planner/GraspPlanner.h>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <cstdlib>
#include <cerrno>
#include <random>

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

GraspPlanner::GraspPlanner(const char * dropboxBase, bool testFlag,
		bool reSimulateFlag, const char * existingId, char * dataFileName) {

	// Save test flag and base type.
	this->testFlag = testFlag;

	// Create file paths.
	fileId[0] = 0;
	char dataStr[20];
	strcpy(dataStr, "data.bin");

	if (!reSimulateFlag){
		strcat(fileId, "XXXXXX");
		mktemp(fileId);
	}
	else{
		strcpy(fileId, existingId);
		strcpy(dataStr, dataFileName);
	}
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
	if (!reSimulateFlag){
		boost::filesystem::create_directories(prefix);
	}
	strcat(prefix, fileId);
	strcat(localPrefix, fileId);
	logFile [0] = dataFile[0] = debugLogFile[0] = pointFile [0] = rgbFile [0] = depthFile[0] = resultFile[0] = trajectoryFile[0] = 0; // Set to ""
	strcat(logFile, localPrefix); strcat(dataFile, baseFolder); strcat(debugLogFile, localPrefix); strcat(pointFile, localPrefix); strcat(rgbFile, localPrefix); strcat(depthFile, localPrefix); strcat(resultFile, localPrefix); strcat(trajectoryFile, prefix);
	strcat(logFile, ".log"); strcat(dataFile, dataStr); strcat(debugLogFile, "_debug.log"); strcat(pointFile, ".pcd"); strcat(rgbFile, "_rgb.png"); strcat(depthFile, "_depth.png"); strcat(resultFile, ".gd"); strcat(trajectoryFile, ".trj");
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
    for (int i = 0; i < 1000; i++)
    {
    	collisionState[i] = -1;
    }

    if (!reSimulateFlag){
		// Find camera and gaze locations
		for (int i = 0; i<numberOfAngles; i++)
		{
			// Calculate a location and orientation for placing the depth cam.
			glm::vec3 camPosition, gazePosition = {(RF-0.5) * 0.06, (RF-0.5) * 0.06, (-(0.25 + RF * 0.06))};
			float lookAngle = 0.64 + 0.4 * RF; //(37-60 degrees)
			float distanceFromCenter = 0.4 + 0.35 * RF; // 0.45-0.75 meter distance!
			float horzAngle = RF * 2 * M_PI;
			float horzDistance = cos(lookAngle) * distanceFromCenter;
			camPosition[0] = cos(horzAngle) * horzDistance;
			camPosition[1] = sin(horzAngle) * horzDistance;
			camPosition[2] = sin(lookAngle) * distanceFromCenter + gazePosition[2];

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

void GraspPlanner::ReadPreMadeTrajectories(int numberOfGrasps){
	finalApproachArr = new Path* [numberOfGrasps];
	float wristPos[3];
	float wristQuat[4];
	float fingerPos[20];

	for (int readCtr = 0; readCtr<numberOfGrasps; readCtr++)
	{
		float tmp[15];
		int wpCount = 10;
		int graspType = 0;
		Path* tmpPath = new Path(wpCount);
		fread(tmp, 4, 15, trjFP);
		for (int itr = 5; itr<15; itr++)
		{
			if (tmp[itr] == 1)
				graspType = itr-5;
		}
		std::cout<<"Grasp type:"<<graspType<<std::endl;
		resultArr[readCtr].graspType = graspType;
		if (graspType < 4)
			resultArr[readCtr].wpCount = 3;
		else if (graspType < 9)
			resultArr[readCtr].wpCount = 4;
		else if (graspType < 10)
			resultArr[readCtr].wpCount = 5;

		// We know the forward transformation used to obtain the wrist position and orientation.
		// At this stage, we will revert the transformation.
		Eigen::Vector3f gazeDir(resultArr[readCtr].gazeDir[0], resultArr[readCtr].gazeDir[1], resultArr[readCtr].gazeDir[2]);
		Eigen::Vector3f camPos(resultArr[readCtr].camPos[0], resultArr[readCtr].camPos[1], resultArr[readCtr].camPos[2]);

		// Read each waypoint.
		for (int wpItr = 0; wpItr < wpCount; wpItr++){

			// Read the waypoint parameters.
			fread(wristPos, 4, 3, trjFP);
			fread(wristQuat, 4, 4, trjFP);
			fread(fingerPos, 4, 20, trjFP);

			// Obtain transformation matrices
			Transformation trans(gazeDir, camPos, false);
			Eigen::Vector4f pos(wristPos[0], wristPos[1], wristPos[2], 1);

			// Transform wrist point
			pos = trans.tM * pos;
			Eigen::Quaternionf wQuat = trans.tQuat * Eigen::Quaternionf(wristQuat[0], wristQuat[1], wristQuat[2], wristQuat[3]);
			wQuat.normalize();

			// Assign wrist position and orientation
			tmpPath->waypoints[wpItr].pos[0] = pos[0];
			tmpPath->waypoints[wpItr].pos[1] = pos[1];
			tmpPath->waypoints[wpItr].pos[2] = pos[2];
			tmpPath->waypoints[wpItr].quat.x = wQuat.x();
			tmpPath->waypoints[wpItr].quat.y = wQuat.y();
			tmpPath->waypoints[wpItr].quat.z = wQuat.z();
			tmpPath->waypoints[wpItr].quat.w = wQuat.w();

			// Assign joints.
			for (int i = 0; i < 20; i++)
			{
				tmpPath->waypoints[wpItr].jointAngles[i] = fingerPos[i];
			}
		}

		// The path has been read. Time to convert.
		finalApproachArr[readCtr] = new Path(resultArr[readCtr].wpCount + 2);
		finalApproachArr[readCtr]->graspType = graspType;

		// Set the first waypoint as an initial approach from outside.
		finalApproachArr[readCtr]->waypoints[0].pos[0] = 2 * tmpPath->waypoints[0].pos[0];
		finalApproachArr[readCtr]->waypoints[0].pos[1] = 2 * tmpPath->waypoints[0].pos[1];
		finalApproachArr[readCtr]->waypoints[0].pos[2] = 2 * tmpPath->waypoints[0].pos[2] + 0.35;
		finalApproachArr[readCtr]->waypoints[0].quat.x = tmpPath->waypoints[0].quat.x;
		finalApproachArr[readCtr]->waypoints[0].quat.y = tmpPath->waypoints[0].quat.y;
		finalApproachArr[readCtr]->waypoints[0].quat.z = tmpPath->waypoints[0].quat.z;
		finalApproachArr[readCtr]->waypoints[0].quat.w = tmpPath->waypoints[0].quat.w;
		for (int k = 0; k<20; k++)
			finalApproachArr[readCtr]->waypoints[0].jointAngles[k] = 0;

		// Set the rest of the wpCount + 1 waypoints.
		for (int wpItr = 0; wpItr<resultArr[readCtr].wpCount + 1; wpItr++)
		{
			int point = ceil((double) (wpItr * tmpPath->steps) / resultArr[readCtr].wpCount);
//			std::cout<<"Point "<<wpItr<<"/"<<resultArr[readCtr].wpCount<<" : "<<point<<std::endl;
			Grasp::Waypoint wp = tmpPath->Interpolate(point);
			finalApproachArr[readCtr]->waypoints[wpItr+1].pos[0] = wp.pos[0];
			finalApproachArr[readCtr]->waypoints[wpItr+1].pos[1] = wp.pos[1];
			finalApproachArr[readCtr]->waypoints[wpItr+1].pos[2] = wp.pos[2];
			finalApproachArr[readCtr]->waypoints[wpItr+1].quat.x = wp.quat.x;
			finalApproachArr[readCtr]->waypoints[wpItr+1].quat.y = wp.quat.y;
			finalApproachArr[readCtr]->waypoints[wpItr+1].quat.z = wp.quat.z;
			finalApproachArr[readCtr]->waypoints[wpItr+1].quat.w = wp.quat.w;
			for (int k = 0; k<20; k++)
				finalApproachArr[readCtr]->waypoints[wpItr+1].jointAngles[k] = wp.jointAngles[k];
		}

		// Sanity check. Seek set to beginning, and load some data, and compare.

//		fseek ( trjFP , -280 * 4 , SEEK_CUR );
//		float buf[270];
//		fread(buf, 4, 280, trjFP);

		// Finally, save grasp parameter data
		std::vector<float> curParams = finalApproachArr[readCtr]->getGraspParams(gazeDir, camPos, resultArr[readCtr].wpCount);

		/*
		float diff = 0;
		float maxDiff = 0;
		int maxDiffIdx = 0;
		for (int j = 0; j<280; j++)
		{
			diff += fabs(buf[j] - curParams[j]);
			if (fabs(buf[j] - curParams[j]) > maxDiff)
			{
				maxDiff = fabs(buf[j] - curParams[j]);
				maxDiffIdx = j;
			}
		}
		for (int j = 0; j<280; j++)
		{
			std::cout<<buf[j]<<" ";
		}
		std::cout<<std::endl;
		for (int j = 0; j<280; j++)
		{
			std::cout<<curParams[j]<<" ";
		}
		std::cout<<std::endl;
		std::cout<<"Max diff: "<<maxDiff<<" at idx "<<maxDiffIdx<<" "<<buf[maxDiffIdx]<<" "<<curParams[maxDiffIdx]<<std::endl;

		std::cout<<"SANITY CHECK: SHOULD BE VERY SMALL"<<diff<<std::endl;
*/
		graspParams.push_back(curParams);
	}
	fclose(trjFP);
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
		finalApproachArr[readCtr]->waypoints[0].pos[2] = 2 * finalApproachArr[readCtr]->waypoints[1].pos[2] + 0.35;
		finalApproachArr[readCtr]->waypoints[0].quat.x = finalApproachArr[readCtr]->waypoints[1].quat.x;
		finalApproachArr[readCtr]->waypoints[0].quat.y = finalApproachArr[readCtr]->waypoints[1].quat.y;
		finalApproachArr[readCtr]->waypoints[0].quat.z = finalApproachArr[readCtr]->waypoints[1].quat.z;
		finalApproachArr[readCtr]->waypoints[0].quat.w = finalApproachArr[readCtr]->waypoints[1].quat.w;
		for (int k = 0; k<20; k++)
			finalApproachArr[readCtr]->waypoints[0].jointAngles[k] = 0;

		// Finally, save grasp parameter data
		Eigen::Vector3f gazeDir(resultArr[readCtr].gazeDir[0], resultArr[readCtr].gazeDir[1], resultArr[readCtr].gazeDir[2]);
		Eigen::Vector3f camPos(resultArr[readCtr].camPos[0], resultArr[readCtr].camPos[1], resultArr[readCtr].camPos[2]);
		std::vector<float> curParams = finalApproachArr[readCtr]->getGraspParams(gazeDir, camPos, wpCount);
		graspParams.push_back(curParams);
	}
	fclose(trjFP);
}

bool GraspPlanner::FollowTrajectory(const mjModel* m, mjData* d, float yOffset){

	/*
	for (int i = 0; i<10; i++)
	{
		std::cout<<"Waypoint "<<i<<" for grasp counter "<<graspCounter-1<<std::endl;
		finalApproachArr[graspCounter-1]->waypoints->print();
	}
	return false;
	*/

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
void GraspPlanner::PerformGrasp(const mjModel* &m, mjData* &d, mjtNum * stableQpos, mjtNum * stableQvel, mjtNum * stableCtrl){

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
				std::cout<<"Gaze:"<<gazeDirArr[i][0]<<" "<<gazeDirArr[i][1]<<" "<<gazeDirArr[i][2]<<std::endl;
				CollectData(Simulator, m, d, depthBuffer, cameraPosArr[i], gazeDirArr[i], camSize, minPointZ, &finishFlag, logStream, i);
				std::cout<<"New Gaze:"<<gazeDirArr[i][0]<<" "<<gazeDirArr[i][1]<<" "<<gazeDirArr[i][2]<<std::endl;

				// Print aux info
		//		std::cout<<"Getting data from "<<i<<"th location."<<std::endl;
		//		std::cout<<cameraPosArr[i][0]<<" "<<cameraPosArr[i][1]<<" "<<cameraPosArr[i][2]<<std::endl;
		//		std::cout<<gazeDirArr[i][0]<<" "<<gazeDirArr[i][1]<<" "<<gazeDirArr[i][2]<<std::endl;

				// Crop and save file.
			    cv::Rect roi;
			    roi.x = 90;
			    roi.y = 10;
			    roi.width = 460;
			    roi.height = 460;

				// RGB
			    /*
				strcpy(tmpStr, imagePrefix);
				strcat(tmpStr, "/");
				strcat(tmpStr, std::to_string(i).c_str());
				strcat(tmpStr, ".jpg");
				cv::Mat img = cv::imread(Simulator->rgbFile);
			    cv::Mat crop = img(roi);
			    cv::Mat dst;
			    cv::resize(crop, dst, cv::Size(224, 224), 0, 0, cv::INTER_CUBIC);
			    cv::imwrite(tmpStr, dst);
			    */

			    // DEPTH
				strcpy(tmpStr, imagePrefix);
				strcat(tmpStr, "/");
				strcat(tmpStr, std::to_string(i).c_str());
				strcat(tmpStr, ".png");
			    cv::Mat cropDepth = Simulator->scaled_im_(roi);
			    cv::Mat outDepth, filteredDepth;
			    cv::flip(cropDepth, outDepth, -1);
			    medianBlur ( outDepth, filteredDepth, 3 );
			    cv::Mat dstDepth;
			    cv::resize(filteredDepth, dstDepth, cv::Size(224, 224), 0, 0, cv::INTER_NEAREST);

			    // Count non-nan pixels
			    int counter = 0;
			    for (int r=0; r<224; ++r) {
			    	unsigned int* depthMapRow = dstDepth.ptr<unsigned int>(r);
			    	for (int c = 0; c<224; c++)
			    	{
			    		unsigned int px = depthMapRow[224 - c];
			    		if (px > 0)
			    			counter++;
			    	}
			    }
			    if (counter >= validViewPixels)
			    {
			    	validViews.push_back(i);
			    	cv::imwrite(tmpStr, dstDepth);
			    }
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
		if (!startFlag && !reSimulateFlag){

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
			if (!reSimulateFlag)
			{
				// Get trajectory data from server.
				success = Connector::DownloadFileFromDropbox(trajectoryFile);
			}
			else
			{
				success = true;
				strcpy(trajectoryFile, dataFile);
			}
			if (success)
			{

				std::cout<<"Reading trajectories from the file "<<trajectoryFile<<" "<<std::endl;
				// Read trajectory count

				if (!reSimulateFlag){
					trjFP = fopen(trajectoryFile, "rb");
					fread(&numberOfGrasps, 4, 1, trjFP);
				}
				else
				{
					numberOfGrasps = boost::filesystem::file_size(trajectoryFile) / (285 * sizeof(float));
					trjFP = fopen(trajectoryFile, "rb");
				}

				if (numberOfGrasps > numberOfMaximumGrasps)
					numberOfGrasps = numberOfMaximumGrasps;

				// Allocate output array
				if (!reSimulateFlag)
				{
					resultArr = new Grasp::GraspResult[numberOfGrasps];

					// Allocate a view for each grasp
					for (int i = 0; i<numberOfGrasps; i++)
					{
						int id = validViews[rand()%validViews.size()];
						resultArr[i].viewId = id;
						resultArr[i].camPos = cameraPosArr[id];
						resultArr[i].gazeDir = gazeDirArr[id];
					}

					// Read trajectories
					ReadTrajectories(numberOfGrasps);
				}
				else
				{
					char tmpStr[1000];
					strcpy(tmpStr, baseFolder);
					strcat(tmpStr, "graspData.data");
					resultArr = Grasp::readGraspData(tmpStr);

					// Read trajectories
					ReadPreMadeTrajectories(numberOfGrasps);
				}
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
			{
				d->qpos[i] = stableQpos[i];
			}
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

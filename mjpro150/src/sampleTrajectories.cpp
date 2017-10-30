//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <stdio.h>
#include "stdlib.h"
#include "string.h"
#include <stdio.h>
#include <sensor/simulate.h>
#include <sensor/camera.h>
#include <iostream>
#include <fstream>
#include <math.h>       /* cos */
#include <chrono>
#include <planner/GraspPlanner.h>
#include <sensor/camera.h>
#include <record/savelog.h>
#include <util/util.h>
#include <util/Connector.h>
#include <util/Path.h>
#include <util/Waypoint.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#define PI 3.14159265

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam, wristCam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;

// Dataset related variables.
int objectCount = 255;
int baseIds[1000];
bool utensilFlag = false;
bool pauseFlag = true;
bool findStableFlag = true;
bool stableFlag = false;
int counter = 0;
mjtNum *lastQpos, *stableQpos = NULL, *stableQvel = NULL, *stableCtrl = NULL;
double stableError = 0.000005;
mjtNum lastPose[4], lastPos[3]; // For keeping track of pose after object is gripped.
int stableCounter = 0, stableIterations = 5000;
bool visualFlag = true;

// Get the path to the dot pattern
std::string dotPath = "./kinect-pattern_3x3.png";

// Grasp planner.
Grasp::GraspPlanner *planner;

// Camera stuff
glm::vec3 camDirection, camPosition;
glm::mat4 TM;
unsigned char glBuffer[2400*4000*3];
unsigned char addonBuffer[2400*4000];

// Data output stuff
FILE * outFile = NULL, *outGraspDataFile = NULL;
int skipSteps = 50, ctr = 0;

// Joint info debugging data.
double jointArr[20][6];

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
    }

    // backspace: reset simulation
	if( act==GLFW_PRESS && key==GLFW_KEY_ENTER )
	{
		if (pauseFlag)
			pauseFlag = false;
		else pauseFlag = true;
	}
}

// mouse button callback
void mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    // update button state
    button_left =   (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT)==GLFW_PRESS);
    button_middle = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_MIDDLE)==GLFW_PRESS);
    button_right =  (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT)==GLFW_PRESS);

    // update mouse position
    glfwGetCursorPos(window, &lastx, &lasty);
}

// mouse move callback
void mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    // no buttons down: nothing to do
    if( !button_left && !button_middle && !button_right )
        return;

    // compute mouse displacement, save
    double dx = xpos - lastx;
    double dy = ypos - lasty;
    lastx = xpos;
    lasty = ypos;

    // get current window size
    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // get shift key state
    bool mod_shift = (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT)==GLFW_PRESS ||
                      glfwGetKey(window, GLFW_KEY_RIGHT_SHIFT)==GLFW_PRESS);

    // determine action based on mouse button
    mjtMouse action;
    if( button_right )
        action = mod_shift ? mjMOUSE_MOVE_H : mjMOUSE_MOVE_V;
    else if( button_left )
        action = mod_shift ? mjMOUSE_ROTATE_H : mjMOUSE_ROTATE_V;
    else
        action = mjMOUSE_ZOOM;

    // move camera
    mjv_moveCamera(m, action, dx/height, dy/height, &scn, &cam);
}

// scroll callback
void scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    // emulate vertical mouse motion = 5% of window height
    mjv_moveCamera(m, mjMOUSE_ZOOM, 0, -0.05*yoffset, &scn, &cam);
}

void copyArray(mjtNum *src, mjtNum *dest, int no){
	for (int i = 0; i< no; i++)
		dest[i] = src[i];
}

void graspObject(const mjModel* m, mjData* d){

	if (!pauseFlag)
	{
		// Find stable position and preserve it.
		if (findStableFlag)
		{
			// Get total displacement
			double err = 0;
			for (int i = 0; i<m->nq; i++)
				err += fabs((d->qpos[i]) - lastQpos[i]);

			// If stable, move on.
			if (err < stableError || stableCounter >= stableIterations)
			{
				// Get stable data.
				copyArray(d->qpos, stableQpos, m->nq);
				copyArray(d->qvel, stableQvel, m->nv);
				copyArray(d->ctrl, stableCtrl, m->nu);
				stableFlag = true;
				findStableFlag = false;

				// Print data.
				(*(planner->logStream))<<"QPOS STABLE"<<std::endl;
				for (int i = 0; i<m->nq; i++)
				{
					(*(planner->logStream))<<d->qpos[i]<<" ";
				}
				(*(planner->logStream))<<std::endl;
			}

			// Get last data
			copyArray(d->qpos, lastQpos, m->nq);
			stableCounter++;
		}

		// When beginning a new grasp, save data.
     	if (planner->getGraspState() == Grasp::grasping && planner->counter == 0)
     	{
     		fprintf(outGraspDataFile, "#Time:%lf# Starting grasp %d.\n", d->time, planner->graspCounter);
     		(*(planner->logStream))<<"#Time:"<<d->time<<"# Starting grasp "<<planner->graspCounter<<"."<<std::endl;
     	}

     	if (planner->getGraspState() == Grasp::lifting && planner->counter == 0)
     	{
     		// Find stability info with respect to the global frame.
     		mjtNum stablePose[4], stablePos[3], tmpQuat[4], tmpQuat2[4], vec[3] = {0, 0, 1}, res[3], res2[3];
     		stablePos[0] = stableQpos[27];
     		stablePos[1] = stableQpos[28];
     		stablePos[2] = stableQpos[29];
     		stablePose[0] = stableQpos[30];
     		stablePose[1] = stableQpos[31];
     		stablePose[2] = stableQpos[32];
     		stablePose[3] = stableQpos[33];
     		lastPos[0] = d->qpos[27];
     		lastPos[1] = d->qpos[28];
     		lastPos[2] = d->qpos[29];
     		lastPose[0] = d->qpos[30];
     		lastPose[1] = d->qpos[31];
     		lastPose[2] = d->qpos[32];
     		lastPose[3] = d->qpos[33];

     		// Find difference quaternion
     		mju_negQuat(tmpQuat, stablePose);
     		mju_mulQuat(tmpQuat2, tmpQuat, lastPose);
     		mju_rotVecQuat(res, vec, tmpQuat2);

     		// Get angle between two vectors.
     		float angle = acos(mju_dot3(res, vec)) / PI; // scaled to 0-1.
     		float distance = sqrt(pow(lastPos[0] - stablePos[0],2) + pow(lastPos[1] - stablePos[1],2) + pow(lastPos[2] - stablePos[2],2));
     		fprintf(outGraspDataFile, "#Time:%lf# Grasp %d has %f shift and %f rotation.\n", d->time, planner->graspCounter, distance, angle);
     		(*(planner->logStream))<<"#Time:"<<d->time<<"# Grasp "<<planner->graspCounter<<" has "<<distance<<" shift and "<<angle<<" rotation."<<std::endl;
     	}

		// If we're at the end of a stand state, save log data.
     	if (planner->getGraspState() == Grasp::stand && planner->counter == 800)
     	{
     		// Calculate grasp success
     		bool graspSuccess = d->qpos[29] > 0;

     		mjtNum finalPose[4], finalPos[3], tmpQuat[4], tmpQuat2[4], vec[3] = {0, 0, 1}, res[3], res2[3];
     		finalPos[0] = d->qpos[27];
     		finalPos[1] = d->qpos[28];
     		finalPos[2] = d->qpos[29] - 1;
     		finalPose[0] = d->qpos[30];
     		finalPose[1] = d->qpos[31];
     		finalPose[2] = d->qpos[32];
     		finalPose[3] = d->qpos[33];

     		// Find difference quaternion
     		mju_negQuat(tmpQuat, lastPose);
     		mju_mulQuat(tmpQuat2, tmpQuat, finalPose);
     		mju_rotVecQuat(res, vec, tmpQuat2);

     		// Get angle between two vectors.
     		float angle = acos(mju_dot3(res, vec)) / PI; // scaled to 0-1.
     		float distance = sqrt(pow(finalPos[0] - lastPos[0],2) + pow(finalPos[1] - lastPos[1],2) + pow(finalPos[2] - lastPos[2],2));
     		fprintf(outGraspDataFile, "#Time:%lf# Finished grasp %d. Grasp success: %d. Grasp shift: %f. In-hand rotation: %f.\n", d->time, planner->graspCounter, (int)graspSuccess, distance, angle);
     		(*(planner->logStream))<<"#Time:"<<d->time<<"# Finished grasp "<<planner->graspCounter<<". Grasp success: "<<graspSuccess<<". Grasp shift: "<<distance<<". In-hand rotation: "<<angle<<"."<<std::endl;
     	}

		// Perform grasping loop.
		if (stableFlag)
		{
			// Perform grasp loop
			planner->PerformGrasp(m, d, stableQpos, stableQvel, stableCtrl, &scn, &con);
		}
	}
}


Grasp::Path * ReadNext(FILE * trjFP){
	// Reads the next trajectory from the file, and sets path variable.
	float extraGrip = 0.03;
	int wpCount = 0;
	float likelihood = 0;
	int graspType = 0;
	float wristPos[3];
	float wristQuat[4];
	float fingerPos[20];
	fread(&likelihood, 4, 1, trjFP);
	fread(&graspType, 4, 1, trjFP);
	fread(&wpCount, 4, 1, trjFP);
	Grasp::Path * finalApproach = new Grasp::Path(wpCount+1);

	// Read trajectory waypoint by waypoint.
	for (int i = 1; i< wpCount+1; i++)
	{
		fread(wristPos, 4, 3, trjFP);
		fread(wristQuat, 4, 4, trjFP);
		fread(fingerPos, 4, 20, trjFP);

		float addedVal = 0;
		if (i == wpCount)
			addedVal = extraGrip;

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
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + addedVal;
			else if (k < 4)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + addedVal;
			}
			else if (k < 8)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + 0.087266 + addedVal;
			}
			else if (k < 12)
			{
				finalApproach->waypoints[i].jointAngles[k] = fingerPos[k] + addedVal;
			}
			else if (k < 16)
			{
				finalApproach->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.087266) + addedVal;
			}
			else{
				finalApproach->waypoints[i].jointAngles[k] = (fingerPos[k] - 0.1745) + addedVal;
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

	return finalApproach;
}

// main function
int main(int argc, const char** argv)
{
	boost::filesystem::path p("./trjdata");
	boost::filesystem::directory_iterator end_itr;
	bool flag = false;
	char oldestName[100];
	time_t oldestTime = std::numeric_limits<time_t>::max();

	// Go through the files
	int count = 0;
	for (auto i = boost::filesystem::directory_iterator(p); i != boost::filesystem::directory_iterator(); i++)
	{

		// Only get trajectory files.
		if (i->path().extension().string() != ".trj")
			continue;

		// Read trajectory info
		int numberOfGrasps = 0;

		// File
		FILE * trjFP = fopen(i->path().string().c_str(), "rb");
		char newFile[1000];
		strcpy(newFile, "./processedTrj/");
		strcat(newFile, i->path().stem().string().c_str());
		strcat(newFile, ".trj");
		FILE * newTrjFP = fopen(newFile, "wb");
		fread(&numberOfGrasps, 4, 1, trjFP);

		// File 1
		char debugFile[1000];
		strcpy(debugFile, "./processedTrj/");
		strcat(debugFile, i->path().stem().string().c_str());
		strcat(debugFile, "_debug.log");
		std::ifstream t(debugFile);
		std::string str((std::istreambuf_iterator<char>(t)),
		                 std::istreambuf_iterator<char>());

		// Read camera parameters
		float posx, posy, posz, gazex, gazey, gazez, upx, upy, upz, rightx, righty, rightz;
		std::size_t found = str.find(std::string("RGB CAM POS:")) + 12;
		sscanf(str.c_str() + found, "%f %f %f", &posx, &posy, &posz);
		found = str.find(std::string("CAM GAZE:")) + 9;
		sscanf(str.c_str() + found, "%f %f %f", &gazex, &gazey, &gazez);
		found = str.find(std::string("CAM UP:")) + 7;
		sscanf(str.c_str() + found, "%f %f %f", &upx, &upy, &upz);
		found = str.find(std::string("CAM RIGHT:")) + 10;
		sscanf(str.c_str() + found, "%f %f %f", &rightx, &righty, &rightz);

		// Print data.
//		std::cout<<"POSITION: "<<posx<<" "<<posy<<" "<<posz<<std::endl;
//		std::cout<<"GAZE: "<<gazex<<" "<<gazey<<" "<<gazez<<std::endl;
//		std::cout<<"UP: "<<upx<<" "<<upy<<" "<<upz<<std::endl;
//		std::cout<<"RIGHT: "<<rightx<<" "<<righty<<" "<<rightz<<std::endl;

		glm::mat4 viewM = glm::lookAt(glm::vec3(posx, posy, posz), glm::vec3(posx+gazex, posy+gazey, posz+gazez), glm::vec3(upx, upy, upz));
		/*
		std::cout<<" **************************** "<<std::endl;
		const float *pSource = (const float*)glm::value_ptr(viewM);
		std::cout<<pSource[0]<<" "<<pSource[1]<<" "<<pSource[2]<<" "<<pSource[3]<<std::endl;
		std::cout<<pSource[4]<<" "<<pSource[5]<<" "<<pSource[6]<<" "<<pSource[7]<<std::endl;
		std::cout<<pSource[8]<<" "<<pSource[9]<<" "<<pSource[10]<<" "<<pSource[11]<<std::endl;
		std::cout<<pSource[12]<<" "<<pSource[13]<<" "<<pSource[14]<<" "<<pSource[15]<<std::endl;
		std::cout<<" **************************** "<<std::endl;
		*/

		// Read paths one by one
		for (int j = 0; j < numberOfGrasps; j++)
		{
			Grasp::Path * path = ReadNext(trjFP);

			// Interpolate waypoints to 10 separate points.
			int limit = 10;
			for (int k = 0; k<limit; k++){
				int cnt = round(k * (1500.0 / (limit-1)));
				Grasp::Waypoint wp = path->Interpolate(cnt);

				// Convert to view coordinates.
				glm::vec4 pos;
				pos[0] = wp.pos[0];
				pos[1] = wp.pos[1];
				pos[2] = wp.pos[2];
				pos[3] = 1;
				pos = viewM * pos;

				// Get rotation matrix
				glm::mat4 rotM = glm::toMat4(wp.quat);

				// Convert to the camera coordinate system.
				rotM = viewM * rotM;

				// Print data
				const float *pSource = (const float*)glm::value_ptr(rotM);
				/*
				std::cout<<"QUAT:"<<wp.quat.w<<" "<<wp.quat.x<<" "<<wp.quat.y<<" "<<wp.quat.z<<std::endl;
				std::cout<<" *************** "<<std::endl;
				std::cout<<pSource[0]<<" "<<pSource[1]<<" "<<pSource[2]<<" "<<pSource[3]<<std::endl;
				std::cout<<pSource[4]<<" "<<pSource[5]<<" "<<pSource[6]<<" "<<pSource[7]<<std::endl;
				std::cout<<pSource[8]<<" "<<pSource[9]<<" "<<pSource[10]<<" "<<pSource[11]<<std::endl;
				std::cout<<pSource[12]<<" "<<pSource[13]<<" "<<pSource[14]<<" "<<pSource[15]<<std::endl;
				std::cout<<" *************** "<<std::endl;
				*/
				glm::quat newQuat = glm::toQuat(rotM);
		//		std::cout<<"CONVERTED QUAT:"<<newQuat.w<<" "<<newQuat.x<<" "<<newQuat.y<<" "<<newQuat.z<<std::endl;
				float printedVals[7];
				printedVals[0] = pos[0];
				printedVals[1] = pos[1];
				printedVals[2] = pos[2];
				printedVals[3] = newQuat.w;
				printedVals[4] = newQuat.x;
				printedVals[5] = newQuat.y;
				printedVals[6] = newQuat.z;

				fwrite(printedVals, 4, 7, newTrjFP);
				// Convert position and quat.
				fwrite(wp.jointAngles, 4, 20, newTrjFP);
			}
		}
		fclose(trjFP);
		fclose(newTrjFP);
	}
}




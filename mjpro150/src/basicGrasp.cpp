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
#include <sensor/simulate.h>
#include <sensor/camera.h>
#include <iostream>
#include <fstream>
#include <math.h>       /* cos */
#include <chrono>
#include <vector>
#include <planner/GraspPlanner.h>
#include <sensor/camera.h>
#include <record/savelog.h>
#include <util/util.h>
#include <util/Connector.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
int objectCount = 0;
int nonCollidingGraspLimit = 10;
bool firstTimeFlag = true; // True in the first run, false otherwise.
int baseIds[1000];
bool pauseFlag = true;
double stableError = 0.000005;
mjtNum lastPose[4], lastPos[3]; // For keeping track of pose after object is gripped.
int stableCounter = 0, stableIterations = 4000, minStableIterations = 100, stableRuns = 3, runCounter = 0;
bool visualFlag = true;
bool testFlag = false;
bool terminateFlag = false;
int classSelection = 0;

// True, if we need to re-create the simulation
bool reSimulateFlag = false;
char existingId[] = "3vWRed";
bool keepCollidingGrasps = false;

// The variables that depend on the initialisation of the simulation
bool findStableFlag = true;
bool stableFlag = false;
mjtNum *lastQpos, *stableQpos = NULL, *stableQvel = NULL, *stableCtrl = NULL;

// Save latest grasp data
Grasp::GraspResult latestGrasp;

// Get the path to the dot pattern
std::string dotPath = "./kinect-pattern_3x3.png";

// Grasp planner.
Grasp::GraspPlanner *planner;

// Camera stuff
glm::vec3 camDirection, camPosition;
glm::mat4 TM;
unsigned char glBuffer[2400*1800*3];
unsigned char glBufferAfter[2400*1800*3];
unsigned char addonBuffer[2400*4000];
bool redPointCloud = false;
bool getObjectImages = false;

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

    // backspace: reset simulation
	if( act==GLFW_PRESS && key==GLFW_KEY_T )
	{
		if (testFlag)
		{
			testFlag = false;
			planner->testFlag = false;
		}
		else
		{
			testFlag = true;
			planner->testFlag = true;
		}
	}

    // b - Next grasp
	if( act==GLFW_PRESS && key==GLFW_KEY_N )
	{
		if ((planner->numberOfGrasps-1) > planner->graspCounter)
		{
			planner->counter = 0;
			if (testFlag)
				planner->prevState = Grasp::checkingCollision;
			else
				planner->prevState = Grasp::stand;
			planner->hasCollided = false;
			planner->graspState = Grasp::reset;
		}
	}

    // b - Previous grasp
	if( act==GLFW_PRESS && key==GLFW_KEY_B )
	{
		std::cout<<"Grasp counter: "<<planner->graspCounter<<std::endl;
		if (planner->graspCounter > 2)
		{
			planner->graspCounter -= 2;
			planner->counter = 0;
			if (testFlag)
				planner->prevState = Grasp::checkingCollision;
			else
				planner->prevState = Grasp::stand;
			planner->hasCollided = false;
			planner->graspState = Grasp::reset;
		}
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

			if (runCounter >= stableRuns)
			{
				terminateFlag = true;
				return;
			}

			// If stable, move on.
			if ((err < stableError || stableCounter >= stableIterations) && (stableCounter >= minStableIterations))
			{
				// If the object is outside the 20x20 box in the middle,
				// we put it in a random location in this square.
				bool relocated = false;
				std::cout<<"Object position: "<<d->qpos[m->nq-7]<<" "<<d->qpos[m->nq-6]<<std::endl;
				if ((fabs(d->qpos[m->nq-7]) > 0.05 || fabs(d->qpos[m->nq-6]) > 0.05) && planner->baseType != 1)
				{
					std::cout<<"Object has moved outside!"<<std::endl;
					// The object moved outside! Put it in a 10cmx10cm area in the middle.
					float newX = (RF - 0.5) * 0.09;
					float newY = (RF - 0.5) * 0.09;
					d->qpos[m->nq-7] = newX;
					d->qpos[m->nq-6] = newY;
					relocated = true;
					runCounter ++;
					stableCounter = 0;
					if (runCounter >= stableRuns)
						terminateFlag = true;
				}

				// If the object has not been relocated,
				if (!relocated)
				{
					// Get stable data.
					copyArray(d->qpos, stableQpos, m->nq);
					copyArray(d->qvel, stableQvel, m->nv);
					copyArray(d->ctrl, stableCtrl, m->nu);
					stableFlag = true;
					findStableFlag = false;

					// We save the stability data at this point.
					char tmp[1000];
					strcpy(tmp, planner->baseFolder);
					strcat(tmp, "stable.data");
					std::ofstream f1(tmp, std::ios::out | std::ios::binary);
					f1.write((char * ) stableQpos, m->nq * sizeof(mjtNum));
					f1.write((char * ) stableQvel, m->nv * sizeof(mjtNum));
					f1.write((char * ) stableCtrl, m->nu * sizeof(mjtNum));
					f1.close();

					// Print data.
					(*(planner->logStream))<<"QPOS STABLE"<<std::endl;
					for (int i = 0; i<m->nq; i++)
					{
						(*(planner->logStream))<<d->qpos[i]<<" ";
					}
					(*(planner->logStream))<<std::endl;
				}
			}

			// Get last data
			copyArray(d->qpos, lastQpos, m->nq);
			stableCounter++;
		}

		// Calculate error after lifting
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
     		latestGrasp.x1 = distance;
     		latestGrasp.r1 = angle;
     		(*(planner->logStream))<<"#Time:"<<d->time<<"# Grasp "<<planner->graspCounter<<" has "<<distance<<" shift and "<<angle<<" rotation."<<std::endl;
     	}

		// If we're at the end of a stand state, save log data.
     	if (planner->getGraspState() == Grasp::stand && planner->counter == 1000)
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

     		// Get grasp result
     		latestGrasp.x2 = distance;
     		latestGrasp.r2 = angle;
     		latestGrasp.successProbability = (double) graspSuccess;

     		// Save grasp data to the relevant array.
     		planner->resultArr[planner->graspCounter - 1].counter++;
			planner->resultArr[planner->graspCounter - 1].successProbability += latestGrasp.successProbability;
     		if (graspSuccess)
     		{
     			planner->resultArr[planner->graspCounter - 1].successCounter++;
				planner->resultArr[planner->graspCounter - 1].x1 += latestGrasp.x1;
				planner->resultArr[planner->graspCounter - 1].r1 += latestGrasp.r2;
				planner->resultArr[planner->graspCounter - 1].x2 += latestGrasp.x2;
				planner->resultArr[planner->graspCounter - 1].r2 += latestGrasp.r2;
     		}

     		// Output to file.
     		std::cout<<"Latest grasp:"<<planner->graspCounter<<" "<<latestGrasp.successProbability<<std::endl;
     		(*(planner->logStream))<<"#Time:"<<d->time<<"# Finished grasp "<<planner->graspCounter<<". Grasp success: "<<graspSuccess<<". Grasp shift: "<<distance<<". In-hand rotation: "<<angle<<"."<<std::endl;
     	}

		// Perform grasping loop.
		if (stableFlag)
		{

			std::cout.flush();
			// Perform grasp loop
//			std::cout<<"Grasp state:"<<planner->graspState<<std::endl;
			planner->PerformGrasp(m, d, stableQpos, stableQvel, stableCtrl);
		}
	}
}

void render(GLFWwindow* window, const mjModel* m, mjData* d)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
	mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);

	mjrRect bottomright = {
		rect.left+rect.width-planner->camSize[1],
		rect.bottom,
		planner->camSize[1],
		planner->camSize[0]
	};

	mjrRect bottomleft = {
		rect.left,
		rect.bottom,
		planner->camSize[1],
		planner->camSize[0]
	};

	// Render the depth buffer.
//	mjr_drawPixels(planner->depthBuffer, NULL, bottomright, &con);


	// Read pixels from the buffer
	mjrRect maxRect = mjr_maxViewport(&con);
	if (getObjectImages)
	{
		mjr_readPixels(glBuffer, NULL, maxRect, &con);
	}

	// Render the rgb buffer.
//	mjr_drawPixels(planner->rgbBuffer, NULL, bottomleft, &con);

	/*
    glLineWidth(10);
	glBegin(GL_LINES);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0, 0.5);
	glVertex3f(0, 0, 0);
	glVertex3f(0, 0.25, 0);
	glVertex3f(0, 0, 0);
	glVertex3f(0.1, 0, 0);
	glEnd();


    glLineWidth(5);
	glBegin(GL_LINES);
	glVertex3f(planner->cameraPos[0], planner->cameraPos[1], planner->cameraPos[2]);
	glVertex3f(planner->cameraPos[0]+(planner->gazeDir[0]*0.3), planner->cameraPos[1]+(planner->gazeDir[1]*0.3), planner->cameraPos[2]+(planner->gazeDir[2]*0.3));
	glm::vec3 right = glm::normalize(glm::cross(planner->gazeDir, glm::vec3(0, 0, 1)));
	glm::vec3 up = glm::normalize(glm::cross(right, planner->gazeDir));
	glVertex3f(planner->cameraPos[0], planner->cameraPos[1], planner->cameraPos[2]);
	glVertex3f(planner->cameraPos[0]+(right[0]*0.075), planner->cameraPos[1]+(right[1]*0.075), planner->cameraPos[2]+(right[2]*0.075));
	glVertex3f(planner->cameraPos[0], planner->cameraPos[1], planner->cameraPos[2]);
	glVertex3f(planner->cameraPos[0]+(up[0]*0.15), planner->cameraPos[1]+(up[1]*0.15), planner->cameraPos[2]+(up[2]*0.15));
	glEnd();
	*/

	// Fill in relevant pixels with point cloud data.
	if (planner->Simulator->cloud != nullptr){
		glBegin(GL_POINTS);
		for (int i = 0; i < planner->Simulator->cloud->size(); i++) {
			  glVertex3f(planner->Simulator->cloud->points[i].x - 0.5, //.x
						  planner->Simulator->cloud->points[i].y, //.y
						  planner->Simulator->cloud->points[i].z);
		}
		glEnd();
	}

	if (getObjectImages)
	{

		mjr_readPixels(glBufferAfter, NULL, maxRect, &con);

		// Pixel-by-pixel check.
		for (int i = 0; i < maxRect.height * maxRect.width; i++)
		{
			if (glBuffer[i * 3] != glBufferAfter[i * 3])
				glBufferAfter[i * 3] = 255;
		}
		mjr_drawPixels(glBufferAfter, NULL, maxRect, &con);
	}
}

bool ismember(std::vector<int> vec, int query)
{
	bool flag = false;
	for (int i = 0; i<vec.size(); i++)
	{
		if (vec[i] == query)
		{
			flag = true;
			break;
		}
	}
	return flag;
}

// main function
int main(int argc, const char** argv)
{
	unsigned int tmpNo;
    // Allocate planner
	unsigned int randSeed = time(NULL);

    // check command-line arguments
    if( argc < 6 )
    {
        printf(" USAGE:  basicGrasp outputFolder ModelFolder dropboxFolder <visualOn/visualOff> classSelection (RandomInt)\n");
        return 0;
    }

    // If random seed provided, use it
	sscanf(argv[5], "%ud", &tmpNo);
	randSeed = randSeed + (time_t) tmpNo;
    srand(randSeed);
    std::cout<<" RANDOM SEED: "<<randSeed << std::endl;
    std::cout<<"First random no:"<<rand()<<std::endl;

    // Read class selection parameter.
	sscanf(argv[4], "%ud", &classSelection);

    // Create dropbox directories
    char tmpDropboxFolder[1000];
    strcpy(tmpDropboxFolder, argv[2]);
    strcat(tmpDropboxFolder, "/");
    strcat(tmpDropboxFolder, "/points");
	if (!boost::filesystem::is_directory(tmpDropboxFolder))
		boost::filesystem::create_directories(tmpDropboxFolder);
    strcpy(tmpDropboxFolder, argv[2]);
    strcat(tmpDropboxFolder, "/");
    strcat(tmpDropboxFolder, "/data");
	if (!boost::filesystem::is_directory(tmpDropboxFolder))
		boost::filesystem::create_directories(tmpDropboxFolder);
    strcpy(tmpDropboxFolder, argv[2]);
    strcat(tmpDropboxFolder, "/");
    strcat(tmpDropboxFolder, "/upload");
	if (!boost::filesystem::is_directory(tmpDropboxFolder))
		boost::filesystem::create_directories(tmpDropboxFolder);

	// Exclude some objects. Hard coded. Needs to change.
	std::vector <int> excludedObjects;
	excludedObjects.push_back(0);

	// Count number of objects
    // Get base id file name
	char setsFile[1000];
    strcpy(setsFile, argv[1]);
    strcat(setsFile, "/sets.txt");

    // Read class id
    FILE * setsFid = fopen(setsFile, "r");
    for (int i = 0; i<10000; i++)
    {
    	int setId;
    	int charsRead = fscanf(setsFid, "%d\n", &setId);
    	if (charsRead < 1)
    		break;
    	if (setId < 2) // Only work with validation and test data for now
    		excludedObjects.push_back(i+1);
    }
    fclose(setsFid);

	// Count number of objects
    // Get base id file name
	char classIdFile[1000];
    strcpy(classIdFile, argv[1]);
    strcat(classIdFile, "/classIds.txt");

    // Read class id
    int classId = 0;
    FILE * classFid = fopen(classIdFile, "r");
    for (int i = 0; i<10000; i++)
    {
    	int charsRead = fscanf(classFid, "%d\n", &classId);
    	if (charsRead < 1)
    		break;
    	objectCount++;
    }
    fclose(classFid);
    std::cout<<"Object count:"<<objectCount<<std::endl;

    // Allocate planner
    planner = new Grasp::GraspPlanner(argv[2], testFlag, reSimulateFlag, existingId);

    // Depending on requested class, get relevant objects.
    int objectId = 0;
    if (!reSimulateFlag)
    {
		if (!classSelection)
		{
			while(ismember(excludedObjects, objectId))
				objectId = rand()%objectCount + 1;
		}
		else
		{
			std::vector<int> objectIdx;
			char tmpStr[1000];
			strcpy(tmpStr, argv[1]);
			strcat(tmpStr, "/classIds.txt");

			// Read relevant numbers into an array.
			int tmpNo;
			FILE * fid = fopen(tmpStr, "r");
			for (int i = 0; i<objectCount; i++)
			{
				fscanf(fid, "%d\n", &tmpNo);
				if (tmpNo == classSelection && !ismember(excludedObjects, i + 1))
				{
					objectIdx.push_back(i + 1);
				}
			}
			fclose(fid);

			// Check if we have any examples of this class
			if (!objectIdx.size())
			{
				std::cout<<"This class has no objects! Pick a number between 0-15, with 0 allocated for all classes."<<std::endl;
				return -1;
			}
			else std::cout<<"This class has "<<objectIdx.size()<<" objects."<<std::endl;

			// Select a random element of this class.
			objectId = objectIdx[rand()%(objectIdx.size())];
		}
    }
    else
    {
    	char objectIdOutput[1000];
    	strcpy(objectIdOutput, planner->baseFolder);
    	strcat(objectIdOutput, "/objectId.txt");
        FILE * fid = fopen(objectIdOutput, "r");
        fscanf(fid, "%d", &objectId);
        fclose(fid);
    }
    std::cout<<"Selected object:"<<objectId<<std::endl;

    char baseIdFile[1000];
    strcpy(baseIdFile, argv[1]);
    strcat(baseIdFile, "/baseIds.txt");

    // Read base id
    int baseType = 0;
    FILE * fid = fopen(baseIdFile, "r");
    for (int i = 0; i<objectId; i++)
    	fscanf(fid, "%d\n", &baseType);
    fclose(fid);

    // Update planner
    planner->randSeed = randSeed;
    planner->baseType = baseType;

	// Get baseId
    int baseId;
	char baseIdOutput[1000];
	strcpy(baseIdOutput, planner->baseFolder);
	strcat(baseIdOutput, "/baseId.txt");
	FILE * fBase = NULL;
    if (!reSimulateFlag)
    {
    	fBase = fopen(baseIdOutput, "w");
    	baseId = rand()%13 + 1;
    	fprintf(fBase, "%d\n", baseId);
    }
    else
    {
    	fBase = fopen(baseIdOutput, "r");
    	fscanf(fBase, "%d\n", &baseId);
    }
	fclose(fBase);

    // Activate software
    mj_activate("mjkey.txt");

    // Set visual flag based on input.
    if (!strcmp(argv[3], "visualOn"))
    	visualFlag = true;
    else
    	visualFlag = false;

    // Remove old files
    Grasp::RemoveOldTmpFolders(argv[1], reSimulateFlag);

    // install control callback
    mjcb_control = graspObject;

    std::cout<<"Object and base ID:"<<objectId<<" "<<baseId<<std::endl;

    // Read stable parameters
    if (reSimulateFlag)
    {
		findStableFlag = false;
		stableFlag = true;
    }

    // Create random xml files.
    std::string modelPath = Grasp::CreateXMLs(argv[1], planner, objectId, baseId, planner->numberOfTrials, reSimulateFlag);

    // Visual operations
	GLFWwindow* window = NULL;

	// create window, make OpenGL context current, request v-sync
	if (!visualFlag)
	{
		glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
		glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	}
	else
	{
		// init GLFW
		if( !glfwInit() )
			mju_error("Could not initialize GLFW");
		window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
		glfwMakeContextCurrent(window);
		glfwSwapInterval(1);
	}
	std::cout<<"GLFW INIT SKIPPED"<<std::endl;

	// initialize visualization data structures
	mjv_defaultCamera(&cam);
	mjv_defaultCamera(&wristCam);
	mjv_defaultOption(&opt);
	opt.geomgroup[0] = 1;
	opt.geomgroup[1] = 1;
	opt.geomgroup[2] = 1;
	opt.geomgroup[3] = 0;
	opt.geomgroup[4] = 0;
	for (int i = 0; i<18; i++)
	{
		if (i != 17)
			opt.flags[i] = 0;
	}

	std::cout<<"MJV CAM STARTED"<<std::endl;

	if (visualFlag)
	{
		// install GLFW mouse and keyboard callbacks
		glfwSetKeyCallback(window, keyboard);
		glfwSetCursorPosCallback(window, mouse_move);
		glfwSetMouseButtonCallback(window, mouse_button);
		glfwSetScrollCallback(window, scroll);

		// Enlarge the points
		glPointSize(5);
	}

	// Filename operations
	if (!boost::filesystem::is_directory("./tmp"))
		boost::filesystem::create_directories("./tmp"); // Create temp dir

    // We will grasp the object using each variation once.
    for (planner->varItr = 0; planner->varItr < planner->numberOfTrials; planner->varItr++)
    {
    	// Get the right object into the scene.
    	char currentFile[1000], objectFile[1000];
    	strcpy(currentFile, argv[1]), strcpy(objectFile, argv[1]);
        strcat(objectFile, "/"), strcat(objectFile, planner->fileId), strcat(objectFile, "_include_object.xml");
        strcat(currentFile, "/tmp/"), strcat(currentFile, planner->fileId), strcat(currentFile, "_include_object");
		strcat(currentFile, std::to_string(planner->varItr).c_str());
		strcat(currentFile, ".xml");
        boost::filesystem::copy_file(currentFile, objectFile, boost::filesystem::copy_option::overwrite_if_exists);

        std::cout<<objectFile<<" COPIED"<<std::endl;

		// load and compile model
		char error[1000] = "Could not load binary model";
		if( strlen(modelPath.c_str())>4 && !strcmp(modelPath.c_str()+strlen(modelPath.c_str())-4, ".mjb") )
			m = mj_loadModel(modelPath.c_str(), 0);
		else
			m = mj_loadXML(modelPath.c_str(), 0, error, 1000);
		if( !m )
			mju_error_s("Load model error: %s", error);


        std::cout<<modelPath.c_str()<<" MODEL READ"<<std::endl;

		// If this simulation is a re-run, we advance it to a later stage.

		// Make data, save space for stable position info, start visual structures.
		d = mj_makeData(m);
		if (!planner->varItr)
		{
			lastQpos = new mjtNum[m->nq], stableQpos = new mjtNum[m->nq], stableQvel = new mjtNum[m->nv], stableCtrl = new mjtNum[m->nu];

			if (visualFlag)
			{
				mjr_defaultContext(&con);
				mjv_makeScene(&scn, 1000);          // space for 1000 objects
				mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context
			}
		}
		else
		{
			// Reset planner
		    planner->graspCounter = 0;
			planner->counter = 0;
			if (testFlag)
				planner->prevState = Grasp::checkingCollision;
			else
				planner->prevState = Grasp::stand;
			planner->hasCollided = false;
			planner->graspState = Grasp::reset;
		}

		std::cout<<"DATA MADE"<<std::endl;

		// Read stable object position from the file, if we're re-running
	    if (reSimulateFlag)
	    {
			char tmp[1000];
			strcpy(tmp, planner->baseFolder);
			strcat(tmp, "stable.data");
			FILE * fp = fopen(tmp, "rb");
			fread(stableQpos, sizeof(mjtNum), m->nq, fp);
			fread(stableQvel, sizeof(mjtNum), m->nv, fp);
			fread(stableCtrl, sizeof(mjtNum), m->nu, fp);
			fclose(fp);
			std::cout<<"STABLE DATA READ"<<std::endl;
			planner->graspState = Grasp::planning;
			planner->reSimulateFlag = true;
	    }

		// Unset pause flag.
		pauseFlag = false;

		// run main loop, target real-time simulation and 60 fps rendering
		while( true )
		{
			// If stop requested, we stop simulation.
			if (visualFlag)
				if (glfwWindowShouldClose(window))
					break;

			// All grasps done? Break.
			if (planner->getGraspState() == Grasp::done || terminateFlag)
				break;

			// advance interactive simulation for 1/60 sec
			//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
			//  this loop will finish on time for the next frame to be rendered at 60 fps.
			//  Otherwise add a cpu timer and exit this loop when it is time to render.
			mjtNum simstart = d->time;
			while( d->time - simstart < 1.0/60.0 )
			{
				mj_step(m, d);
			}

			// render the scene.
			if (visualFlag)
			{
				// get framebuffer viewport
				mjrRect viewport = {0, 0, 0, 0};
				glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

				// Track object
				if (getObjectImages)
				{
					cam.lookat[0] = d->qpos[m->nq-7];
					cam.lookat[1] = d->qpos[m->nq-6];
					cam.lookat[2] = d->qpos[m->nq-5];
	//				cam.distance = 0.5;
				}

				// update scene and render
				mjv_updateScene(m, d, &opt, NULL, &cam, mjCAT_ALL, &scn);
				mjr_render(viewport, &scn, &con);
				render(window, m, d);

				// swap OpenGL buffers (blocking call due to v-sync)
				glfwSwapBuffers(window);

				// process pending GUI events, call GLFW callbacks
				glfwPollEvents();
			}
		}

		// Delete existing structures and start again.
	    mj_deleteData(d);
	    mj_deleteModel(m);
    }

    for (int i = 0; i<planner->numberOfGrasps; i++)
    {
    	if (planner->resultArr[i].counter)
    	{
    		std::cout<<"Grasp "<<i<<" has been performed "<<planner->resultArr[i].counter<<" times. Grasp success:"<<planner->resultArr[i].successProbability/(double)(planner->resultArr[i].counter)<<std::endl;
    	}
    }

    bool successFlag = false;
    for (int i = 0; i<planner->numberOfGrasps; i++)
    	if (planner->resultArr[i].counter > 0)
    		successFlag = successFlag || (planner->resultArr[i].successProbability/(double)planner->resultArr[i].counter) > 0.499999;

    // Upload the data.
    std::cout<<"Out of "<<planner->numberOfGrasps<<", "<< planner->numberOfNoncollidingGrasps<< " are non-colliding!"<<std::endl;
    if (!testFlag && planner->numberOfGrasps > 0 && successFlag && planner->numberOfNoncollidingGrasps >= nonCollidingGraspLimit)
    {
    	UploadFiles(argv[1], planner, objectId, baseId, keepCollidingGrasps || reSimulateFlag);
    }

    // delete tmp folder
    if(boost::filesystem::exists(planner->baseFolder) && !reSimulateFlag)
    {
//       boost::filesystem::remove_all(planner->baseFolder);
    }

    // close GLFW, free visualization storage
    if (visualFlag)
    {
    	glfwTerminate();
        mjv_freeScene(&scn);
        mjr_freeContext(&con);
    }

    // Delete final data structures, deactivate.
	delete[] planner->data;
	delete planner;
	delete[] lastQpos;
	delete [] stableQpos;
	delete [] stableQvel;
	delete [] stableCtrl;
    mj_deactivate();

    return 0;
}


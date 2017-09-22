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
#include <planner/GraspPlanner.h>
#include <sensor/camera.h>
#include <record/savelog.h>
#include <util/util.h>
#include <util/Connector.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
int skipSteps = 5, ctr = 0;

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

void print(glm::vec3 a){
	std::cout<<a[0]<<" "<<a[1]<<" "<<a[2]<<std::endl;
	return;
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
				std::cout<<"QPOS STABLE"<<std::endl;
				for (int i = 0; i<m->nq; i++)
				{
					std::cout<<d->qpos[i]<<" ";
				}
				std::cout<<std::endl;
			}

			// Get last data
			copyArray(d->qpos, lastQpos, m->nq);
			stableCounter++;
		}

		// When beginning a new grasp, save data.
     	if (planner->getGraspState() == Grasp::grasping && planner->counter == 0)
     	{
     		fprintf(outGraspDataFile, "#Time:%lf# Starting grasp %d.\n", d->time, planner->graspCounter);
     		std::cout<<"#Time:"<<d->time<<"# Starting grasp "<<planner->graspCounter<<"."<<std::endl;
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
     		std::cout<<"#Time:"<<d->time<<"# Grasp "<<planner->graspCounter<<" has "<<distance<<" shift and "<<angle<<" rotation."<<std::endl;
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
     		std::cout<<"#Time:"<<d->time<<"# Finished grasp "<<planner->graspCounter<<". Grasp success: "<<graspSuccess<<". Grasp shift: "<<distance<<". In-hand rotation: "<<angle<<"."<<std::endl;
     	}

		// Perform grasping loop.
		if (stableFlag)
		{
			// Perform grasp loop
			planner->PerformGrasp(m, d, stableQpos, stableQvel, stableCtrl, &scn, &con);
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
	mjr_drawPixels(planner->depthBuffer, NULL, bottomright, &con);

	// Render the rgb buffer.
	mjr_drawPixels(planner->rgbBuffer, NULL, bottomleft, &con);

	// Fill in relevant pixels with point cloud data.
	if (planner->Simulator->cloud != nullptr){

		glBegin(GL_POINTS);
		for (int i = 0; i < planner->Simulator->cloud->size(); i++) {
		  if (planner->Simulator->cloud->points[i].r > 0)
		  {
			  glVertex3f(planner->Simulator->cloud->points[i].x - 0.5, //.x
			  					  planner->Simulator->cloud->points[i].y, //.y
			  					  planner->Simulator->cloud->points[i].z);
		  }
		}
		glEnd();

	}

}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=4 )
    {
        printf(" USAGE:  basic ModelFolder dropboxFolder <visualOn/visualOff>\n");
        return 0;
    }

	#ifdef __unix__
    // activate software
    mj_activate("mjkey_unix.txt");
	#elif __APPLE__
    mj_activate("mjkey_macos.txt");
	#endif

    // Set visual flag based on input.
    if (!strcmp(argv[3], "visualOn"))
    	visualFlag = true;
    else
    	visualFlag = false;

    // Allocate planner
    planner = new Grasp::GraspPlanner(argv[2]);

    // Redirect cout to file.
    std::ofstream out(planner->debugLogFile);
    std::streambuf *coutbuf = std::cout.rdbuf(); //save old buf
    std::cout.rdbuf(out.rdbuf()); //redirect std::cout to out.txt!
    std::cerr.rdbuf(out.rdbuf()); //redirect std::err to out.txt!
    outGraspDataFile = fopen(planner->resultFile, "w");

    // Get random object, and relevant asset/object files.
    srand(time(NULL));
    int objectId = rand()%objectCount + 1;
    int baseId = rand()%13 + 1;

    // Manage file names
    char convexHullFile[1000], baseIdFile[1000], oldAssetFile[1000], oldBaseAssetFile[1000], newAssetFile[1000],
	newBaseAssetFile[1000], oldObjectFile[1000], oldBaseFile[1000], newObjectFile[1000], newBaseFile[1000],
	tmp[1000], lightFile[1000], tableFile[1000];

    // Copy light and table files.
    strcpy(tmp, argv[1]);
    strcat(tmp, "/include_lightOrg.xml");
    strcpy(lightFile, argv[1]);
    strcat(lightFile, "/include_light.xml");
    boost::filesystem::copy_file(tmp, lightFile, boost::filesystem::copy_option::overwrite_if_exists);
    strcpy(tmp, argv[1]);
    strcat(tmp, "/include_tableOrg.xml");
    strcpy(tableFile, argv[1]);
    strcat(tableFile, "/include_table.xml");
    boost::filesystem::copy_file(tmp, tableFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Create rest of the files.
    strcpy(convexHullFile, argv[1]);
    strcat(convexHullFile, "/convhull.txt");
    strcpy(baseIdFile, argv[1]);
    strcat(baseIdFile, "/baseIds.txt");

    strcpy(oldAssetFile, argv[1]);
    strcat(oldAssetFile, "/include_object_assets.xml");
    strcpy(oldBaseAssetFile, argv[1]);
    strcat(oldBaseAssetFile, "/include_base_assets.xml");

    strcpy(newAssetFile, argv[1]);
    strcat(newAssetFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_assets.xml", objectId, objectId);
    strcat(newAssetFile, tmp);

    strcpy(newBaseAssetFile, argv[1]);
    strcat(newBaseAssetFile, "/mesh/bases/");
    sprintf(tmp, "D_%d/D_%d_assets.xml", baseId, baseId);
    strcat(newBaseAssetFile, tmp);

    strcpy(oldObjectFile, argv[1]);
    strcat(oldObjectFile, "/include_object.xml");
    strcpy(oldBaseFile, argv[1]);
    strcat(oldBaseFile, "/include_base.xml");

    strcpy(newObjectFile, argv[1]);
    strcat(newObjectFile, "/mesh/objects/");
    sprintf(tmp, "D_%d/D_%d_object.xml", objectId, objectId);
    strcat(newObjectFile, tmp);

    strcpy(newBaseFile, argv[1]);
    strcat(newBaseFile, "/mesh/bases/");
    sprintf(tmp, "D_%d/D_%d_base.xml", baseId, baseId);
    strcat(newBaseFile, tmp);

    // Read base ids.
    FILE * fid = fopen(baseIdFile, "r");
    for (int i = 0; i<objectCount; i++)
    	fscanf(fid, "%d\n", &(baseIds[i]));

    // Create model name.
    char modelStr[1000];
    strcpy(modelStr, argv[1]);
    // If utensil, place in a base consisting of a cup.
    if (baseIds[objectId - 1] == 1)
    {
    	utensilFlag = true;
    	planner->minPointZ = -0.255; // Table is at -0.35, each base is about 9 cms.
        boost::filesystem::copy_file(newBaseFile, oldBaseFile, boost::filesystem::copy_option::overwrite_if_exists);
        boost::filesystem::copy_file(newBaseAssetFile, oldBaseAssetFile, boost::filesystem::copy_option::overwrite_if_exists);
    	strcat(modelStr, "/BHAM_Test_Base.xml");
    }

    // If a plate, bowl or a pan, place on an invisible base. There's no collision between the base and the hand.
    else if (baseIds[objectId - 1] == 2) {
    	strcat(modelStr, "/BHAM_Test_BaseInv.xml");
    }
    else strcat(modelStr, "/BHAM_Test.xml");

    // Copy files.
    boost::filesystem::copy_file(newAssetFile, oldAssetFile, boost::filesystem::copy_option::overwrite_if_exists);
    boost::filesystem::copy_file(newObjectFile, oldObjectFile, boost::filesystem::copy_option::overwrite_if_exists);

    // Modify the xml files with random parameters.
    Grasp::ModifyXMLs(argv[1], objectId, baseId);

    // Before we move on to grasping loop, we save all the model files to the cloud.
    char tmpStr[1000];
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_light.xml");
    boost::filesystem::copy_file(lightFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_table.xml");
    boost::filesystem::copy_file(tableFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_object.xml");
    boost::filesystem::copy_file(oldObjectFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_base.xml");
    boost::filesystem::copy_file(oldBaseFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_object_assets.xml");
    boost::filesystem::copy_file(oldAssetFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);
    strcpy(tmpStr, "./tmp/");
    strcat(tmpStr, planner->fileId);
    strcat(tmpStr, "_include_base_assets.xml");
    boost::filesystem::copy_file(oldBaseAssetFile, tmpStr, boost::filesystem::copy_option::overwrite_if_exists);
    Grasp::Connector::UploadFile(tmpStr);

    // install control callback
    mjcb_control = graspObject;

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(modelStr)>4 && !strcmp(modelStr+strlen(modelStr)-4, ".mjb") )
        m = mj_loadModel(modelStr, 0);
    else
        m = mj_loadXML(modelStr, 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);
    lastQpos = new mjtNum[m->nq], stableQpos = new mjtNum[m->nq], stableQvel = new mjtNum[m->nv], stableCtrl = new mjtNum[m->nu];

    GLFWwindow* window = NULL;
	// init GLFW
	if( !glfwInit() )
		mju_error("Could not initialize GLFW");

	// create window, make OpenGL context current, request v-sync
	if (!visualFlag)
	{
		glfwWindowHint(GLFW_AUTO_ICONIFY, GLFW_FALSE);
		glfwWindowHint(GLFW_FOCUSED, GLFW_FALSE);
		glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
	}
	window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
	glfwMakeContextCurrent(window);
	glfwSwapInterval(1);

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

    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

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

    // Filename operations, srand
	boost::filesystem::create_directories("./tmp/"); // Create temp dir
    std::srand(std::time(NULL));

    // Open out file.
    outFile = fopen(planner->logFile, "wb");
    std::cout<<planner->logFile<<" opened!"<<std::endl;

	// Print header.
    writeHeader(m, d, outFile);

    // Write object names.
    char c;
    for( int n=0; n<strlen(m->names); n++ )
    {
        std::fwrite(&c, 1, 1, outFile);
    }

    // Save the first log, and read it from the file into an array.
    // We'll use the array to reset the simulation.
    int recsz = 1 + m->nq + m->nv + m->nu + 7*m->nmocap + m->nsensordata;
    planner->data = new float[recsz];
	savelog(m, d, planner->data, outFile);

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
		if (planner->getGraspState() == Grasp::done)
			break;

		// advance interactive simulation for 1/60 sec
		//  Assuming MuJoCo can simulate faster than real-time, which it usually can,
		//  this loop will finish on time for the next frame to be rendered at 60 fps.
		//  Otherwise add a cpu timer and exit this loop when it is time to render.
		mjtNum simstart = d->time;
		while( d->time - simstart < 1.0/60.0 )
		{
			if (planner->getGraspState() == Grasp::grasping ||
					planner->getGraspState() == Grasp::lifting ||
					planner->getGraspState() == Grasp::stand)
			{
				// Here, we save data. Skip steps if needed.
				if (skipSteps > 0 && ctr < skipSteps)
				{
					ctr++;
				}
				else{
					int recsz = 1 + m->nq + m->nv + m->nu + 7*m->nmocap + m->nsensordata;
					float buffer[recsz];
					savelog(m, d, buffer, outFile);
					ctr = 0;
				}
			}

			mj_step(m, d);
		}

		// render the scene.
		if (visualFlag)
		{
			// get framebuffer viewport
			mjrRect viewport = {0, 0, 0, 0};
			glfwGetFramebufferSize(window, &viewport.width, &viewport.height);

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

    // Close record file
    fclose(outFile);
    fclose(outGraspDataFile);
    std::cout.rdbuf(coutbuf); //reset to standard output again

    // Save log file, grasp data and debug_log
//    Grasp::Connector::UploadFile(planner->logFile);
    Grasp::Connector::UploadFile(planner->debugLogFile);
    Grasp::Connector::UploadFile(planner->resultFile);

    // delete tmp folder
    if(boost::filesystem::exists("./tmp"))
    {
       boost::filesystem::remove_all("./tmp");
    }

    // close GLFW, free visualization storage
    if (visualFlag)
    	glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
	delete[] planner->data;
	delete planner;
	delete[] lastQpos;
	delete [] stableQpos;
	delete [] stableQvel;
	delete [] stableCtrl;
    mj_deleteModel(m);
    mj_deactivate();

    return 1;
}


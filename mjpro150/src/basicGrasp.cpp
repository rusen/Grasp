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
#include <thread>         // std::thread
#include <planner/GraspPlanner.h>
#include <sensor/camera.h>
#include <record/savelog.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

#define PI 3.14159265

// MuJoCo data structures
mjModel* m = NULL;                  // MuJoCo model
mjData* d = NULL;                   // MuJoCo data
mjvCamera cam;                      // abstract camera
mjvOption opt;                      // visualization options
mjvScene scn;                       // abstract scene
mjrContext con;                     // custom GPU context

// mouse interaction
bool button_left = false;
bool button_middle = false;
bool button_right =  false;
double lastx = 0;
double lasty = 0;
bool pauseFlag = true;

// Get the path to the dot pattern
std::string dotPath = "./kinect-pattern_3x3.png";

// Grasp planner.
Grasp::GraspPlanner planner;

// Camera stuff
glm::vec3 camDirection, camPosition;
glm::mat4 TM;
unsigned char glBuffer[2400*4000*3];
unsigned char addonBuffer[2400*4000];

// Data output stuff
FILE * outFile = NULL;
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


glm::mat4 getTM(glm::vec3 gaze, glm::vec3 pos, bool usePerspective){
	// Obtain handedness and up vectors.
	glm::vec3 r = glm::normalize(glm::cross(gaze, glm::vec3(0,0,1)));
	glm::vec3 u = glm::normalize(glm::cross(r, gaze));

    // Create transformation vector for second camera.
	glm::mat4 s(-1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1);
	glm::mat4 rot(r[0], r[1], r[2], 0,
			    u[0], 	u[1], 	  u[2], 0,
			    gaze[0],  gaze[1],  gaze[2], 0,
				0, 0, 0, 1);
	glm::mat4 t(1, 0, 0, -pos[0],
				0, 1, 0, -pos[1],
				0, 0, 1, -pos[2],
				0, 0, 0, 1);
	glm::mat4 p( 1, 0, 0, 0,
				 0, 1, 0, 0,
				 0, 0, 1, 0,
				 0, 0, 1, 0);

	glm::mat4 TM;
	// Create camera transformation matrix.
	if (usePerspective)
		TM = p * (s * (rot * t));
	else
		TM = s * (rot * t);

	return TM;
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

void graspObject(const mjModel* m, mjData* d){
	if (!pauseFlag)
		planner.PerformGrasp(m, d);
}

void render(GLFWwindow* window, const mjModel* m, mjData* d)
{

    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
	mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);

	mjrRect bottomright = {
		rect.left+rect.width-planner.camSize[1],
		rect.bottom,
		planner.camSize[1],
		planner.camSize[0]
	};

	// Render the scene.
	mjr_drawPixels(planner.depthBuffer, NULL, bottomright, &con);

	// Fill in relevant pixels with point cloud data.
	if (planner.Simulator->cloud != nullptr){
		glBegin(GL_POINTS);
		for (int i = 0; i < planner.Simulator->cloud->height * planner.Simulator->cloud->width; i++) {
		  if (planner.Simulator->cloud->points[i].r > 0)
		  {
			  /*
			  glVertex3f(planner.Simulator->cloud->points[i].y, //.x
					  planner.Simulator->cloud->points[i].x - 0.5, //.y
					  planner.Simulator->cloud->points[i].z);
					  */
			  glVertex3f(planner.Simulator->cloud->points[i].x - 0.5, //.x
			  					  planner.Simulator->cloud->points[i].y, //.y
			  					  planner.Simulator->cloud->points[i].z);
		  }
		}
		glEnd();

		/*

		glBegin(GL_LINES);
		for (int i = 0; i < planner.Simulator->cloud->height * planner.Simulator->cloud->width; i++) {
			  glVertex3f(planner.Simulator->cloud->points[i].x,
					  planner.Simulator->cloud->points[i].y,
					  planner.Simulator->cloud->points[i].z);

			  glVertex3f(planner.Simulator->cloud->points[i].x + planner.Simulator->cloud->points[i].normal_x*0.01,
					  planner.Simulator->cloud->points[i].y + planner.Simulator->cloud->points[i].normal_y*0.01,
					  planner.Simulator->cloud->points[i].z + planner.Simulator->cloud->points[i].normal_z*0.01);
		}
		glEnd();
	*/
	}

	// Here, we save data. Skip steps if needed.
	if (skipSteps > 0 && ctr < skipSteps)
	{
		ctr++;
		return;
	}

	savelog(m, d, outFile);
	ctr = 0;

	/*
	// Highlight the joint information here.
	glColor3f(1.0f, 0.0f, 0.0f);
	glBegin(GL_POINTS);
	for (int i = 0; i < 20; i++) {
	  glVertex3f(jointArr[i][0], jointArr[i][1], jointArr[i][2]);
	}
	glEnd();

	glLineWidth(2.5);
	glColor3f(1.0, 0.0, 0.0);
	glBegin(GL_LINES);
	for (int i = 0; i < 20; i++) {
		glVertex3f(jointArr[i][0], jointArr[i][1], jointArr[i][2]);
		glVertex3f(jointArr[i][0] + (jointArr[i][3]/10), jointArr[i][1] + (jointArr[i][4]/10), jointArr[i][2] + (jointArr[i][5]/10));
	}
	glEnd();
	*/


}

// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=2 )
    {
        printf(" USAGE:  basic mujocoModel.xml\n");
        return 0;
    }

	#ifdef __unix__
    // activate software
    mj_activate("mjkey_unix.txt");
	#elif __APPLE__
    mj_activate("mjkey_macos.txt");
	#endif

    // install control callback
    mjcb_control = graspObject;

    // load and compile model
    char error[1000] = "Could not load binary model";
    if( strlen(argv[1])>4 && !strcmp(argv[1]+strlen(argv[1])-4, ".mjb") )
        m = mj_loadModel(argv[1], 0);
    else
        m = mj_loadXML(argv[1], 0, error, 1000);
    if( !m )
        mju_error_s("Load model error: %s", error);

    // make data
    d = mj_makeData(m);

    // init GLFW
    if( !glfwInit() )
        mju_error("Could not initialize GLFW");

    // create window, make OpenGL context current, request v-sync
    GLFWwindow* window = glfwCreateWindow(1200, 900, "Demo", NULL, NULL);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(1);

    // initialize visualization data structures
    mjv_defaultCamera(&cam);
    mjv_defaultOption(&opt);
    mjr_defaultContext(&con);
    mjv_makeScene(&scn, 1000);                   // space for 1000 objects
    mjr_makeContext(m, &con, mjFONTSCALE_100);   // model-specific context

    // install GLFW mouse and keyboard callbacks
    glfwSetKeyCallback(window, keyboard);
    glfwSetCursorPosCallback(window, mouse_move);
    glfwSetMouseButtonCallback(window, mouse_button);
    glfwSetScrollCallback(window, scroll);

    // Create random file name.
	boost::filesystem::create_directory("./tmp/"); // Create temp dir
    std::srand(std::time(NULL));

    // Open out file.
    outFile = fopen(planner.logFile, "wb");
    std::cout<<planner.logFile<<" opened!"<<std::endl;

	// Print header.
    writeHeader(m, d, outFile);

    // Write object names.
    char c;
    for( int n=0; n<strlen(m->names); n++ )
    {
        std::fwrite(&c, 1, 1, outFile);
    }

    // Enlarge the points
    glPointSize(5);

    // run main loop, target real-time simulation and 60 fps rendering
    while( !glfwWindowShouldClose(window) )
    {
        // advance interactive simulation for 1/60 sec
        //  Assuming MuJoCo can simulate faster than real-time, which it usually can,
        //  this loop will finish on time for the next frame to be rendered at 60 fps.
        //  Otherwise add a cpu timer and exit this loop when it is time to render.
        mjtNum simstart = d->time;
        while( d->time - simstart < 1.0/60.0 ){
            mj_step(m, d);
    }

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

    // close GLFW, free visualization storage
    glfwTerminate();
    mjv_freeScene(&scn);
    mjr_freeContext(&con);

    // free MuJoCo model and data, deactivate
    mj_deleteData(d);
    mj_deleteModel(m);
    mj_deactivate();

    // Close record file
    fclose(outFile);

    return 1;
}


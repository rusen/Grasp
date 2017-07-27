//---------------------------------//
//  This file is part of MuJoCo    //
//  Written by Emo Todorov         //
//  Copyright (C) 2017 Roboti LLC  //
//---------------------------------//


#include "mujoco.h"
#include "glfw3.h"
#include "stdio.h"
#include "stdlib.h"
#include "string.h"
#include <sensor/simulate.h>
#include <sensor/camera.h>
#include <iostream>
#include <controller/MPLHandController.h>
#include <planner/GraspPlanner.h>
#include <glm/glm.hpp>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>
#include <Eigen/Geometry>
#include <Eigen/Core>

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

// Camera Parameters for Carmine 1.09
render_kinect::CameraInfo cam_info;

// Get the path to the dot pattern
std::string dotPath = "./resources/kinect-pattern_3x3.png";

// allocate simulator
render_kinect::Simulate* Simulator = NULL;

// Grasp planner.
Grasp::GraspPlanner planner;

// Hand controller
Grasp::HandControllerInterface * handController = new Grasp::MPLHandController();

// Rendering params
float depth_buffer[5120*2880];        // big enough for 5K screen
unsigned char depth_rgb[1280*720*3];  // 1/4th of screen

// keyboard callback
void keyboard(GLFWwindow* window, int key, int scancode, int act, int mods)
{
    // backspace: reset simulation
    if( act==GLFW_PRESS && key==GLFW_KEY_BACKSPACE )
    {
        mj_resetData(m, d);
        mj_forward(m, d);
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

void graspObject(const mjModel* m, mjData* d){
	planner.PerformGrasp(m, d, handController);
}

// render

void render(GLFWwindow* window)
{
    // past data for FPS calculation
    static double lastrendertm = 0;

    // get current framebuffer rectangle
    mjrRect rect = {0, 0, 0, 0};
    glfwGetFramebufferSize(window, &rect.width, &rect.height);
    mjrRect smallrect = rect;

	// get the depth buffer
	mjr_readPixels(NULL, depth_buffer, rect, &con );

	// convert to RGB, subsample by 4
	for( int r=0; r<rect.height; r+=4 )
		for( int c=0; c<rect.width; c+=4 )
		{
			// get subsampled address
			int adr = (r/4)*(rect.width/4) + c/4;

			// assign rgb
			depth_rgb[3*adr] = depth_rgb[3*adr+1] = depth_rgb[3*adr+2] =
				(unsigned char)((1.0f-depth_buffer[r*rect.width+c])*255.0f);
		}

	// show in bottom-right corner, offset for profiler and sensor
	mjrRect bottomright = {
		smallrect.left+(3*smallrect.width)/4,
		smallrect.bottom,
		smallrect.width/4,
		smallrect.height/4
	};
	mjr_drawPixels(depth_rgb, NULL, bottomright, &con);

}


// main function
int main(int argc, const char** argv)
{
    // check command-line arguments
    if( argc!=3 )
    {
        printf(" USAGE:  basic mujocoModel.xml model.obj \n");
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

    // Get the path to the object mesh model.
    std::string objectModelsDir = "./model/objects/";
    std::stringstream fullPath;
    fullPath << objectModelsDir << argv[2];

    // Create the simulator.
    Simulator = new render_kinect::Simulate(cam_info, fullPath.str(), dotPath);

    // TODO: Change with actual transformation.
    Eigen::Affine3d transform(Eigen::Affine3d::Identity());

    // Get image from kinect camera.
    Simulator->simulateMeasurement(transform, true, true, true);

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
    	render(window);

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
    delete Simulator;

    return 1;
}


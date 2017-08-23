/*
 * CollectData.cpp
 *
 *  Created on: 4 Aug 2017
 *      Author: rusi
 */

#include <sensor/simulate.h>
#include "mujoco.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <thread>
#include <iostream>
#include <unistd.h>

namespace Grasp{

void CollectData(Simulate* Simulator, const mjModel* m, mjData* d, unsigned char* depthBuffer,
		glm::vec3 cameraPos, glm::vec3 gazeDir, int * camSize, bool*finishFlag)
{
	std::cout<<"Entered data collection"<<std::endl;
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// up vector
	glm::vec3 newCamUp;
	int startIdx = (m->nbody-1);

	// Get image from kinect camera.
	Simulator->simulateMeasurement(m, d, cameraPos, gazeDir);
    pcl::PCDWriter().write(Simulator->name, *(Simulator->cloud), true);

	int rows = Simulator->scaled_im_.rows;
	int cols = Simulator->scaled_im_.cols;

	// convert to RGB, subsample by 4
	for( int rowId=0; rowId<rows; rowId++ ){
		for( int colId=0; colId<cols; colId++ )
		{
			// Find where to get the data and where to write it.
			int offset = (rowId)*(cols) + colId;

			// assign rgb
			depthBuffer[offset * 3] = depthBuffer[offset * 3 + 1] = depthBuffer[offset * 3 + 2] =
					Simulator->scaled_im_.at<unsigned char>(rowId, colId); //input[offset];
		}
	}
	*finishFlag = true;

	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
	std::cout<<"Depth image captured!"<<std::endl;
	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
}

}



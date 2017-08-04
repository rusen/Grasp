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

namespace Grasp{

unsigned char* CollectData(Simulate* Simulator, const mjModel* m, mjData* d, int * camSize)
{
	// Allocate space for output.
	unsigned char * depthBuffer = new unsigned char(camSize[0] * camSize[1]);

	// Modify the object's position and orientation.
	int startIdx = (m->nbody-1);
	Eigen::Affine3d transformObject(Eigen::Affine3d::Identity());
	transformObject.translate(Eigen::Vector3d(d->xpos[startIdx * 3], d->xpos[startIdx * 3 + 1], d->xpos[startIdx * 3 + 2]));
	transformObject.rotate(Eigen::Quaterniond(d->xquat[startIdx * 4],d->xquat[startIdx * 4 + 1],d->xquat[startIdx * 4 + 2],d->xquat[startIdx * 4 + 3]));

	// Get image from kinect camera.
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();
	Simulator->simulateMeasurement(m, d);
	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();

	std::cout << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;

	// Tell the world you've captured an image.
	std::cout<<"Depth image captured!"<<std::endl;

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
	return depthBuffer;
}
}



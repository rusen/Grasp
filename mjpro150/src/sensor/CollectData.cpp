/*
 * CollectData.cpp
 *
 *  Created on: 4 Aug 2017
 *      Author: rusi
 */

#include <sensor/simulate.h>
#include "mujoco.h"
#include <thread>
#include <iostream>
#include <unistd.h>

namespace Grasp{

void CollectData(Simulate* Simulator, const mjModel* m, mjData* d,  mjvScene *scn, mjrContext *con, unsigned char* rgbBuffer, unsigned char* depthBuffer,
		glm::vec3 cameraPos, glm::vec3 gazeDir, int * camSize, float minPointZ, bool*finishFlag, std::ofstream * out)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// up vector
	glm::vec3 newCamUp;
	int startIdx = (m->nbody-1);

	// Get image from kinect camera.
	Simulator->simulateMeasurement(m, d, scn, con, cameraPos, gazeDir, minPointZ, out);
	if (Simulator->cloud->size() > 0)
	{
	//	pcl::PCDWriter().write(Simulator->cloudFile, *(Simulator->cloud), true);

	    pcl::PointCloud<pcl::PointXYZ> *cloud = new pcl::PointCloud<pcl::PointXYZ>(Simulator->cloud->size(), 1);
	    for (int i = 0; i<Simulator->cloud->size(); i++)
	    {
	    	if (cloud->points[i].z >= minPointZ)
	    	{
	    		  cloud->points[i].x = Simulator->cloud->points[i].x;
	    		  cloud->points[i].y = Simulator->cloud->points[i].y;
	    		  cloud->points[i].z = Simulator->cloud->points[i].z;
	    	}
	    }
	    pcl::PCDWriter().write(Simulator->cloudFile, *(cloud), true);
	}
	else
		return;

	int rows = Simulator->scaled_im_.rows;
	int cols = Simulator->scaled_im_.cols;

	// convert to RGB, subsample by 4
	for( int rowId=0; rowId<rows; rowId++ ){
		for( int colId=0; colId<cols; colId++ )
		{
			// Find where to get the data and where to write it.
			int offset = (rowId)*(cols) + ((cols - 1) - colId);

			// assign rgb
			depthBuffer[offset * 3] = depthBuffer[offset * 3 + 1] = depthBuffer[offset * 3 + 2] =
					Simulator->scaled_im_.at<unsigned char>(rowId, colId); //input[offset];

			rgbBuffer[offset * 3] = Simulator->rgbIm.data[offset * 3];
			rgbBuffer[offset * 3 + 1] = Simulator->rgbIm.data[offset * 3 + 1];
			rgbBuffer[offset * 3 + 2] = Simulator->rgbIm.data[offset * 3 + 2];
		}
	}
	*finishFlag = true;

	std::chrono::steady_clock::time_point end= std::chrono::steady_clock::now();
	(*out)<<"Depth image captured!"<<std::endl;
	(*out) << "Time difference = " << std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count() <<std::endl;
}

}



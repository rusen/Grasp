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
		glm::vec3 cameraPos, glm::vec3 gazeDir, int * camSize, float minPointZ, bool*finishFlag, std::ofstream * out, int imageId)
{
	std::chrono::steady_clock::time_point begin = std::chrono::steady_clock::now();

	// up vector
	glm::vec3 newCamUp;
	int startIdx = (m->nbody-1);
	glm::quat q;

	// Get image from kinect camera.
	Simulator->simulateMeasurement(m, d, scn, con, cameraPos, gazeDir, minPointZ, out, &q);
	if (Simulator->cloud->size() > 0)
	{
	    // Find camera frame and write the point cloud into a file.
	    Eigen::Vector4f cameraOrigin(cameraPos[0] + 0.5, cameraPos[1], cameraPos[2], 0);
	    Eigen::Quaternionf cameraRot(q.w, q.x, q.y, q.z);
	    pcl::PCLPointCloud2 pc2;
	    pcl::toPCLPointCloud2(*(Simulator->cloud), pc2);
	    pcl::PCDWriter().writeBinaryCompressed(Simulator->cloudFile, pc2, cameraOrigin, cameraRot);
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



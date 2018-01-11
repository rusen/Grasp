/*
 * VirtualCamera.cpp
 *
 *  Created on: 10 Jan 2018
 *      Author: rusi
 */

#include "sensor/VirtualCamera.h"
#include <cmath>

namespace Grasp {

VirtualCamera::VirtualCamera() {
	// TODO Auto-generated constructor stub
}

VirtualCamera::~VirtualCamera() {
	// TODO Auto-generated destructor stub
}

void VirtualCamera::ReprojectPointCloud(char * fileName, pcl::PointCloud<pcl::PointXYZ> * cloud, Eigen::Vector4f origin, Eigen::Quaternionf orientation)
{
	CameraInfo camInfo;
	Camera cam(camInfo);

	// Obtain a transformation matrix out of the orientation quaternion
	Eigen::Matrix3f mat(orientation);

	// Go over the point cloud and find each non-nan point, project it to a pixel.
	int pointCount = 0;
    for(int r=0; r<cam.getHeight(); ++r) {
      for(int c=0; c<cam.getWidth(); ++c) {
    	  if (!std::isnan(cloud->points[r*cam.getWidth()+c].x))
    		  pointCount++;
      }
    }
}

} /* namespace Grasp */

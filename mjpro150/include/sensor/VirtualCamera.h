/*
 * VirtualCamera.h
 *
 *  Created on: 10 Jan 2018
 *      Author: rusi
 */

#ifndef SRC_UTIL_VIRTUALCAMERA_H_
#define SRC_UTIL_VIRTUALCAMERA_H_

#include "camera.h"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

namespace Grasp {

class VirtualCamera {
public:
	VirtualCamera();
	virtual ~VirtualCamera();

	// Reprojection of point cloud
	static void ReprojectPointCloud(char * fileName, pcl::PointCloud<pcl::PointXYZ> * cloud, Eigen::Vector4f origin, Eigen::Quaternionf orientation);
};

} /* namespace Grasp */

#endif /* SRC_UTIL_VIRTUALCAMERA_H_ */

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
#ifndef RF
#define RF (static_cast <float> (rand()) / static_cast <float> (RAND_MAX))
#endif  /* RF */

namespace Grasp {

class VirtualCamera {
public:
	VirtualCamera();
	virtual ~VirtualCamera();

	// Reprojection of point cloud
	static void ReprojectPointCloud(pcl::PointCloud<pcl::PointXYZ> * cloud,
			cv::Mat &depth_map, Eigen::Vector4f origin, Eigen::Vector3f &newGaze,
		    Eigen::Quaternionf orientation, Eigen::Quaternionf &newOrientation, bool findNewCamera);
};

} /* namespace Grasp */

#endif /* SRC_UTIL_VIRTUALCAMERA_H_ */

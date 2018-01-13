/*
 * VirtualCamera.cpp
 *
 *  Created on: 10 Jan 2018
 *      Author: rusi
 */

#include "sensor/VirtualCamera.h"
#include <cmath>
#include <opencv2/imgproc/imgproc.hpp>

namespace Grasp {

VirtualCamera::VirtualCamera() {
	// TODO Auto-generated constructor stub
}

VirtualCamera::~VirtualCamera() {
	// TODO Auto-generated destructor stub
}

void VirtualCamera::ReprojectPointCloud(char * fileName, pcl::PointCloud<pcl::PointXYZ> * cloud,
		  cv::Mat &depth_map, Eigen::Vector4f origin, Eigen::Quaternionf orientation)
{
	CameraInfo camInfo;
	Camera cam(camInfo);

	// Obtain a transformation matrix out of the orientation quaternion
	Eigen::Matrix3f m2(orientation.conjugate());
   	Eigen::Matrix4f TM, PrM;
   	TM.setZero();
   	PrM.setZero();
   	TM(0,0) = m2(0,0), TM(0,1) = m2(0,1), TM(0, 2) = m2(0,2),
   			TM(1, 0) = m2(1, 0), TM(1, 1) = m2(1, 1), TM(1, 2) = m2(1, 2),
   			TM(2, 0) = m2(2, 0), TM(2, 1) = m2(2, 1), TM(2, 2) = m2(2, 2);
   	TM(3, 3) = 1.0f;
   	PrM(0,0) = 1, PrM(1,1) = 1, PrM(2,2) = 1, PrM(3,2) = 1;

   	Eigen::Affine3f transform(Eigen::Translation3f(-origin[0],-origin[1],-origin[2]));
   	Eigen::Matrix4f TrM = transform.matrix();
   	TM = PrM * TM * TrM;

   	// Allocate image
    depth_map = cv::Mat(cam.getHeight(), cam.getWidth(), CV_8UC1);
    depth_map.setTo(0);

	// Go over the point cloud and find each non-nan point, project it to a pixel.
    for(int r=0; r<cam.getHeight(); ++r) {
      unsigned char* depth_map_i = depth_map.ptr<unsigned char>(r);
      for(int c=0; c<cam.getWidth(); ++c) {
    	  if (!std::isnan(cloud->points[r*cam.getWidth()+c].x))
    	  {
    		// Convert to
    	   	Eigen::Vector4f p(cloud->points[r*cam.getWidth()+c].x,
    	   			cloud->points[r*cam.getWidth()+c].y,
					cloud->points[r*cam.getWidth()+c].z, 1.0f);

    	   	// Find point-camera distance
    	   	float distance = sqrt((origin[0] - p[0])*(origin[0] - p[0]) +
    	   			(origin[1] - p[1])*(origin[1] - p[1]) +
					(origin[2] - p[2])*(origin[2] - p[2]));

    	   	// Scale the distance with respect to Z far.
    	   	if (distance < cam.getZFar())
    	   		distance = distance / cam.getZFar();
    	   	else
    	   		distance = 0;
    	   	unsigned char distanceScaled = (unsigned char) round(distance * 255.0);

    		// Convert p to camera coordinates.
    		p = TM * p;

    		// Make p homogenous
    		p = p / p[2];

    		// Find pixels
    		cv::Point2d uv_rect;
    		double x = cam.info_.width - ((cam.info_.fx * p[0]) + cam.info_.cx_);
    		uv_rect.x = (double) x;
    		uv_rect.y = (cam.info_.fy * p[1]) + cam.info_.cy_;

    		// Obtain distance to the camera. We will use this as depth image.
    		depth_map_i[(int)c] = distanceScaled;
    	  }
      }
    }
}

} /* namespace Grasp */

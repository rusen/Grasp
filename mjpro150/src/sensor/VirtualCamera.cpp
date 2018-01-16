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

void VirtualCamera::ReprojectPointCloud(pcl::PointCloud<pcl::PointXYZ> * cloud,
		  cv::Mat &depth_map, Eigen::Vector4f origin, Eigen::Vector3f &newGaze,
		  Eigen::Quaternionf orientation, Eigen::Quaternionf &outOrientation,
		  bool findNewCamera)
{
	CameraInfo camInfo;
	Camera cam(camInfo);
	Eigen::Quaternionf newOrientation;

    // Find a new camera frame to attach to the file.
    if (findNewCamera)
    {
    	double centerx = 0, centery = 0, centerz = 0, numberOfPoints = 0;
        for(int r=0; r<cam.getHeight(); ++r) {
          for(int c=0; c<cam.getWidth(); ++c) {
        	  if (!std::isnan(cloud->points[r*cam.getWidth()+c].x))
        	  {
        		  centerx += cloud->points[r*cam.getWidth()+c].x;
        		  centery += cloud->points[r*cam.getWidth()+c].y;
        		  centerz += cloud->points[r*cam.getWidth()+c].z;
        		  numberOfPoints++;
        	  }
          }
        }

        // If number of points is zero, return
        if (!numberOfPoints)
        	return;

        // Find center of viewport
        centerx /= numberOfPoints;
        centery /= numberOfPoints;
        centerz /= numberOfPoints;
        std::cout<<"Center point: "<<centerx<<" "<<centery<<" "<<centerz<<std::endl;

        // Find a new camera frame with added noise
        Eigen::Vector3f newGazePoint(centerx + (RF-0.5)*0.1, centery + (RF-0.5)*0.1, centerz + (RF-0.5)*0.06);
        std::cout<<"newGazePoint:"<<newGazePoint[0]<<" "<<newGazePoint[1]<<" "<<newGazePoint[2]<<std::endl;
        newGaze = newGazePoint - Eigen::Vector3f(origin[0], origin[1], origin[2]);
        newGaze.normalize();
        Eigen::Vector3f tempUp(0, 0, 1), newRight, newUp;
        newRight = newGaze.cross(tempUp);
        newRight.normalize();
        newUp = newRight.cross(newGaze);
        newUp.normalize();

        // Find the quaternion of the frame
        Eigen::Vector3f tv = Eigen::Vector3f(newGaze[0], newGaze[1], 0);
        tv.normalize();
        float angle2;
        if (tv[1] > 0)
        	angle2 = -(acos(tv[0]) - M_PI/2);
        else
        	angle2 = -((2*M_PI - acos(tv[0])) - M_PI/2);
        float dist1 = sqrt(newGaze[0] * newGaze[0] + newGaze[1] * newGaze[1]);
        float angle3 = atan(newGaze[2]/dist1);
        Eigen::Matrix3f m2;
        Eigen::Quaternionf q1(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitX()));
        Eigen::Quaternionf q2(Eigen::AngleAxisf(angle2, Eigen::Vector3f::UnitY()));
        Eigen::Quaternionf q3(Eigen::AngleAxisf(angle3, Eigen::Vector3f::UnitX()));
        Eigen::Quaternionf q4(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
        m2 = q1 * q2 * q3 * q4;
        Eigen::Quaternionf tmpQuat(m2);
        newOrientation = tmpQuat;
        outOrientation = tmpQuat;
        float centerRatio = ((origin[2] + 0.3)/(origin[2] - centerz));
        std::cout<<"Camera distance to average point:"<<sqrt((centerx - origin[0])*(centerx - origin[0]) + (centery - origin[1])*(centery - origin[1]) + (centerz - origin[2])*(centerz - origin[2]))*centerRatio<<std::endl;
        // Find angle and print that
        float d1 = origin[2] - centerz;
        float d2 = sqrt((origin[1] - centery)*(origin[1] - centery) + (origin[0] - centerx)*(origin[0] - centerx));
        std::cout<<"Ratio: "<<d1/d2<<" and z of center:"<<centerz<<", center ratio:"<<centerRatio<<std::endl;
    }
    else
    {
    	newOrientation = orientation;
    }

    std::cout<<"New orientation: "<<newOrientation.w()<<" "<<newOrientation.x()<<" "<<newOrientation.y()<<" "<<newOrientation.z()<<std::endl;

	// Obtain a transformation matrix out of the orientation quaternion
	Eigen::Matrix3f m2(newOrientation.conjugate());
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
    depth_map = cv::Mat(cam.getHeight(), cam.getWidth(), CV_16UC1);
    depth_map.setTo(0);

	// Go over the point cloud and find each non-nan point, project it to a pixel.
    for(int r=0; r<cam.getHeight(); ++r) {
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
    	   	uint16_t distanceScaled = (uint16_t) round(distance * 65535);

    		// Convert p to camera coordinates.
    		p = TM * p;

    		// Make p homogenous
    		p = p / p[2];

    		// Find pixels
    		cv::Point2d uv_rect;

    		double x1, y1;
    		x1 = ((cam.info_.fx * p[0]) + cam.info_.cx_);
    		y1 = (cam.info_.fy * p[1]) + cam.info_.cy_;
    		int posx = round(x1);
    		int posy = round(y1);

    		// If the pixel falls outside, don't do anything.
    		if (posx < 0 || posy < 0 || posx >= cam.getWidth()|| posy >= cam.getHeight())
    			continue;

    		// Pixel data
    		int row, col;
			row = posy;
			col = posx;
    		uint16_t* depth_map_i = depth_map.ptr<uint16_t>(row);

    		// Obtain distance to the camera. We will use this as depth image.
    		if (depth_map_i[col] == 0 || (distanceScaled < depth_map_i[col]))
    			depth_map_i[col] = distanceScaled;
    	  }
      }
    }
}

} /* namespace Grasp */

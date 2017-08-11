/*********************************************************************
 *
 *  Copyright (c) 2014, Jeannette Bohg - MPI for Intelligent System
 *  (jbohg@tuebingen.mpg.de)
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Jeannette Bohg nor the names of MPI
 *     may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/
/* Header file for camera class that keeps the camera parameters and type of noise.
 * Most important functionality is the conversion of pixel coordinates to rays and 
 * the projection of rays onto the image plane. These functions are taken from the 
 * ros-package image_geometry.
 */

#ifndef KINECT_SIM_CAMERA_H_
#define KINECT_SIM_CAMERA_H_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <glm/gtc/quaternion.hpp>
#include <glm/gtx/quaternion.hpp>

namespace Grasp
{
  enum NoiseType
  {
    GAUSSIAN=1,
    PERLIN,
    SIMPLEX,
    NONE
  };

  class CameraInfo
  {
  public:
	// Initialized with Carmine 1.09 parameters.
    int width = 640, height = 480;
    double z_near = 0.1, z_far = 1.4;
    double fx = 575.0, fy = 450.0;
    double cx_ = 320, cy_ = 240;
    double tx_ = -0.05; // 0.075 normally.

    NoiseType noise_ = GAUSSIAN;

  };

  class Camera
  {
  public:
    
    double getFx()
    {
      return info_.fx;
    }

    double getFy()
    {
      return info_.fy;
    }

    double getCx()
    {
      return info_.cx_;
    }

    double getCy()
    {
      return info_.cy_;
    }

    int getWidth()
    {
      return info_.width;
    }

    int getHeight()
    {
      return info_.height;
    }

    double getZNear()
    {
      return info_.z_near;
    }

    double getZFar()
    {
      return info_.z_far;
    }

    double getTx() 
    {
      return info_.tx_;
    }
    
  Camera(const CameraInfo p_info)
    : info_(p_info) {}
    
    // two functions adopted from ros::image_geometry::PinholeCameraModel
    cv::Point2d project3dToPixel(const glm::vec3 tempP, const Eigen::Matrix4d TM) const
      {
    	Eigen::Matrix<double, 4, 1> p;
    	p << tempP[0], tempP[1], tempP[2], 1;

    	// Convert p to camera coordinates.
    	p = TM * p;

    	// Make p homogenous
    	p = p / p[3];

    	// Write back the output.
		cv::Point2d uv_rect;
		int x = info_.width - ((info_.fx * p[0]) + info_.cx_);
    	uv_rect.x = (double) x;
    	uv_rect.y = (info_.fy * p[1]) + info_.cy_;
		return uv_rect;
      }

    glm::vec3 projectPixelTo3dRay(const cv::Point2d& uv_rect, const glm::vec3 cameraPos, const glm::vec3 viewportCenter, const glm::vec3 normGaze, const glm::vec3 normRight, const glm::vec3 viewUp)
      {
    	glm::vec3 ray;

		// We need to find where the ray is supposed to go.
		glm::vec3 added1, added2, temp(0,0,0);
		added1 = (normRight * (float)(uv_rect.x- info_.cx_))/(float)info_.fx;
		added2 = (viewUp * (float) (uv_rect.y- info_.cy_))/(float)info_.fy;
		temp = viewportCenter + added1 + added2;

//		std::cout<<"Temp point:"<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<std::endl;
//		std::cout<<"Camera pos:"<<cameraPos[0]<<" "<<cameraPos[1]<<" "<<cameraPos[2]<<std::endl;
		// Calculate ray.
		ray = temp - cameraPos;
		ray = normalize(ray);
//		std::cout<<"Ray:"<<ray[0]<<" "<<ray[1]<<" "<<ray[2]<<" "<<glm::length(ray)<<std::endl;
//		glm::vec3 temp2 = (float)glm::length(ray) * ray + cameraPos;
//		std::cout<<"Temp point:"<<temp[0]<<" "<<temp[1]<<" "<<temp[2]<<" Second temp:"<<temp2[0]<<" "<<temp2[1]<<" "<<temp2[2]<<std::endl;

		return ray;
      }

  private:
    CameraInfo info_;
  };
 
} // namespace render_kinect




#endif // KINECT_SIM_CAMERA_H_

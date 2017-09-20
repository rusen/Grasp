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
/* Header file that sets up the simulator and triggers the simulation 
 * of the kinect measurements and stores the results under a given directory.
 */
#ifndef SIMULATE_H
#define SIMULATE_H

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/normal_3d.h>

#include <string.h>
#include <cstdio>
#include <iostream>
#include <string>

#include <sensor/kinectSimulator.h>

static unsigned countf = 0;

namespace Grasp {

  class Simulate {
  public:
  
  Simulate()
    : out_path_("/tmp/") 
      {

		// allocate memory for depth image
		int w = cam_info.width;
		int h = cam_info.height;

		depth_im_ = cv::Mat(h, w, CV_32FC1);
		scaled_im_ = cv::Mat(h, w, CV_32FC1);
		rgbIm = cv::Mat(h, w, CV_8UC3);

		object_model_ = new KinectSimulator(cam_info);

		name = NULL;
		cloud = nullptr;
      }

    ~Simulate() {
      delete object_model_;
      if (cloud != nullptr)
    	  delete cloud;
    }

    void simulateMeasurement(const mjModel* m, mjData* d,  mjvScene *scn, mjrContext *con, glm::vec3 newCamPos, glm::vec3 newCamGaze, float minPointZ) {
      countf++;

      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      cloud = object_model_->intersect(m, d, scn, con, rgbIm, depth_im_, newCamPos, newCamGaze, minPointZ);
      
      // store on disk
  	  std::stringstream lD;
  	  convertScaleAbs(depth_im_, scaled_im_, 255.0f);

      // Save depth map on a file.
      cv::Mat outDepth;
      cv::flip(scaled_im_, outDepth, -1);
      cv::imwrite( "./tmp/tempDepth.png", outDepth );

  	  // Calculate surface normals as well!
  	  int pointCount = cloud->size();
  	  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> ptr(new pcl::PointCloud<pcl::PointXYZ>(pointCount, 1));
  	  for (int i = 0; i < pointCount; i++) {
  		  if (cloud->points[i].r > 0)
  		  {
				ptr->points[i].x = cloud->points[i].x;
				ptr->points[i].y = cloud->points[i].y;
				ptr->points[i].z = cloud->points[i].z;
  		  }
  	  }

  	  // Calculate normals.
  	  ne.setInputCloud(ptr);
      pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
      ne.setSearchMethod (tree);
      pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
      ne.setRadiusSearch (0.05);
      ne.compute (*cloud_normals);
      normals = cloud_normals;

      // Assign back the normals.
  	  for (int i = 0; i < pointCount; i++) {
  		  if (cloud->points[i].r > 0)
  		  {
				cloud->points[i].normal_x = -normals->points[i].normal_x;
				cloud->points[i].normal_y = -normals->points[i].normal_y;
				cloud->points[i].normal_z = -normals->points[i].normal_z;
				cloud->points[i].curvature = -normals->points[i].curvature;
  		  }
  	  }
  	  ptr.reset();
  	  tree.reset();
  	  cloud_normals.reset();

    }

    KinectSimulator *object_model_;
    CameraInfo cam_info;
    cv::Mat depth_im_, scaled_im_, labels_;
    cv::Mat rgbIm;
    std::string out_path_;
    pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud;
	pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
	pcl::PointCloud<pcl::Normal>::Ptr normals;
	char * name;

  };

  // Function to collect data from simulated Kinect camera.
  void CollectData(Simulate* Simulator, const mjModel* m, mjData* d, mjvScene *scn, mjrContext *con, unsigned char* rgbBuffer, unsigned char* depthBuffer, glm::vec3 cameraPos, glm::vec3 gazeDir, int * camSize, float minPointZ, bool*finishFlag);

} //namespace Grasp
#endif // SIMULATE_H

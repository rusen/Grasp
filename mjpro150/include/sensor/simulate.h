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

#ifdef HAVE_PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#endif 

#include <string.h>

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

	object_model_ = new KinectSimulator(cam_info);

	transform_ = Eigen::Affine3d::Identity();

      }

    ~Simulate() {
      delete object_model_;
    }

    void simulateMeasurement(const mjModel* m, mjData* d, glm::vec3 newCamPos, glm::vec3 newCamGaze) {
      countf++;
      
      // simulate measurement of object and store in image, point cloud and labeled image
      cv::Mat p_result;
      object_model_->intersect(m, d, point_cloud_, depth_im_, newCamPos, newCamGaze);
      
      // store on disk
  	  std::stringstream lD;
  	  convertScaleAbs(depth_im_, scaled_im_, 255.0f);
  	  cv::imwrite("/tmp/deneme.png", scaled_im_);

#ifdef HAVE_PCL
	pcl::PointCloud<pcl::PointXYZ> cloud;
	// Fill in the cloud data
	cloud.width = point_cloud_.rows;
	cloud.height = 1;
	cloud.is_dense = false;
	cloud.points.resize(cloud.width * cloud.height);

	for (int i = 0; i < point_cloud_.rows; i++) {
	  const float* point = point_cloud_.ptr<float>(i);
	  cloud.points[i].x = point[0];
	  cloud.points[i].y = point[1];
	  cloud.points[i].z = point[2];

//	  std::cout<<" POINT "<<i<<" : "<<point[0]<<" "<<point[1]<<" "<<point[2]<<std::endl;
	}
#else
	std::cout << "Couldn't store point cloud since PCL is not installed." << std::endl;
#endif

    }

    KinectSimulator *object_model_;
    CameraInfo cam_info;
    cv::Mat depth_im_, scaled_im_, point_cloud_, labels_;
    std::string out_path_;
    Eigen::Affine3d transform_; 

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  };

  // Function to collect data from simulated Kinect camera.
  cv::Mat CollectData(Simulate* Simulator, const mjModel* m, mjData* d, unsigned char* depthBuffer, glm::vec3 cameraPos, glm::vec3 gazeDir, int * camSize, bool*finishFlag);

} //namespace Grasp
#endif // SIMULATE_H

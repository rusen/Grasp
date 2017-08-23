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

/* This is a C++ reimplementation of the kinect sensor model as proposed 
   in 
   
   @incollection{Gschwandtner11b,
   title = {{BlenSor: Blender Sensor Simulation Toolbox Advances in Visual Computing}},
   author = {Gschwandtner, Michael and Kwitt, Roland and Uhl, Andreas and Pree, Wolfgang},
   pages = {199--208},
   publisher = {Springer Berlin / Heidelberg},
   series = {Lecture Notes in Computer Science},
   year = {2011}
   }

   and implemented as a python plugin for blender.
}

*/

#include <sensor/kinectSimulator.h>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sensor/camera.h>
#include <sensor/gaussian.h>
#include <sensor/perlin.h>
#include <sensor/simplex.h>

#include <string>
#include <stdio.h>

#ifdef HAVE_OPENMP
#include <omp.h>
#endif

//static unsigned countf = 0;
//static const int prec = 5;

namespace Grasp {

  const float KinectSimulator::invalid_disp_ = 99999999.9;
  const float KinectSimulator::window_inlier_distance_ = 0.1;

  // Constructor
  KinectSimulator::KinectSimulator(const CameraInfo &p_camera_info)
    : camera_( p_camera_info)
    , noise_type_(p_camera_info.noise_)
    , noise_gen_(NULL)
  {
    
    // colour map assumes there is only one object
    color_map_.push_back(cv::Scalar( rand()&255, rand()&255, rand()&255 ));

    // reading dot pattern for filtering of disparity image
    dot_pattern_ = cv::imread("./kinect-pattern_3x3.png", 0);
    if(! dot_pattern_.data ) // Check for invalid input
      {
	std::cout <<  "Could not load dot pattern from ./kinect-pattern_3x3.png" << std::endl ;
	exit(-1);
      }
    
    // initialize filter matrices for simulated disparity
    weights_ = cv::Mat(size_filt_,size_filt_,CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
      float *weights_i = weights_.ptr<float>(x);
      for(int y=0; y<size_filt_; ++y){
	int tmp_x = x-size_filt_/2;
	int tmp_y = y-size_filt_/2;
	if(tmp_x!=0 && tmp_y!=0)
	  weights_i[y] = 1.0/(sq(1.2*(float)tmp_x)+sq(1.2*(float)tmp_y));
	else 
	  weights_i[y] = 1.0;
      }
    }

    fill_weights_ = cv::Mat(size_filt_, size_filt_, CV_32FC1);
    for(int x=0; x<size_filt_; ++x){
      float *weights_i = fill_weights_.ptr<float>(x);
      for(int y=0; y<size_filt_; ++y){
	int tmp_x = x-size_filt_/2;
	int tmp_y = y-size_filt_/2;
	if(std::sqrt(sq(tmp_x)+sq(tmp_y)) < 3.1)
	  weights_i[y] = 1.0/(1.0+sq(tmp_x)+sq(tmp_y));
	else 
	  weights_i[y] = -1.0;
      }
    }

    // Extracting noise type and setting up noise generator
    if(noise_type_==Grasp::GAUSSIAN)
      {
	//Gaussian Noise
	float mean = 0.0;
	float std  = 0.15;
	noise_gen_ = new GaussianNoise( camera_.getWidth(), camera_.getHeight(), mean, std);
      } else if (noise_type_==Grasp::PERLIN)
      {
	float scale = 0.4;
	noise_gen_ = new PerlinNoise( camera_.getWidth(), camera_.getHeight(), scale);
      } else if (noise_type_==Grasp::SIMPLEX)
      {
	float scale = 0.5;
	noise_gen_ = new SimplexNoise( camera_.getWidth(), camera_.getHeight(), scale);
      }
  }

  // Destructor
  KinectSimulator::~KinectSimulator() 
  {
	  delete noise_gen_;
  }
  
  
  // Function that intersects rays with the object model at current state.
pcl::PointCloud<pcl::PointXYZRGBNormal> * KinectSimulator::intersect(const mjModel* m, mjData* d,
				  cv::Mat &depth_map,
				  glm::vec3 newCamPos,
				  glm::vec3 newCamGaze)
  {

    // allocate memory for depth map
    depth_map = cv::Mat(camera_.getHeight(), camera_.getWidth(), CV_64FC1);
    depth_map.setTo(0.0);
    cv::Mat disp(camera_.getHeight(), camera_.getWidth(), CV_32FC1);
    disp.setTo(invalid_disp_);

    // Allocate space for vectors.
    glm::vec3 rayDir = {0,0,0}, normGaze, viewportCenter;
    int geomId, geomId2;

    // First, we find center of the viewport.
    normGaze = normalize(newCamGaze);
    viewportCenter = newCamPos + normGaze;

    // Next, we find the view-right vector (right hand vector)
    glm::vec3 tempUp(0, 0, 1), viewRight, normRight;
    normRight = normalize(glm::cross(normGaze, tempUp));

    // Get location of the second cam.
    glm::vec3 newCamPos2(0,0,0), temp, tempP, tempP2;
    temp = normRight * (float) camera_.getTx();
    newCamPos2 = newCamPos + temp;

    // Finally, get view-up vector.
    glm::vec3 viewUp;
    viewUp = normalize(glm::cross(normRight, normGaze));
    mjvOption vopt;
    mjv_defaultOption(&vopt);

    // Print the vectors.
    std::cout<<" CAM POS: "<< newCamPos[0]<<" "<< newCamPos[1]<<" "<< newCamPos[2]<<std::endl;
    std::cout<<" CAM GAZE: "<< normGaze[0]<<" "<< normGaze[1]<<" "<< normGaze[2]<<std::endl;
    std::cout<<" CAM VIEWPORT: "<< viewportCenter[0]<<" "<< viewportCenter[1]<<" "<< viewportCenter[2]<<std::endl;
    std::cout<<" CAM RIGHT: "<< normRight[0]<<" "<< normRight[1]<<" "<< normRight[2]<<std::endl;
    std::cout<<" CAM UP: "<< viewUp[0]<<" "<< viewUp[1]<<" "<< viewUp[2]<<std::endl;
    std::cout<<" CAM 2 POS: "<< newCamPos2[0]<<" "<< newCamPos2[1]<<" "<< newCamPos2[2]<<std::endl;

    // Create transformation vector for second camera.
	Eigen::Matrix4d r, s, t, tOrg, p;
	s << -1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1;
	r <<  normRight[0], normRight[1], normRight[2], 0,
		  viewUp[0], 	viewUp[1], 	  viewUp[2], 0,
		  normGaze[0],  normGaze[1],  normGaze[2], 0,
			0, 0, 0, 1;
	t << 1, 0, 0, -newCamPos2[0],
		 0, 1, 0, -newCamPos2[1],
		 0, 0, 1, -newCamPos2[2],
		 0, 0, 0, 1;
	tOrg << 1, 0, 0, -newCamPos[0],
		 0, 1, 0, -newCamPos[1],
		 0, 0, 1, -newCamPos[2],
		 0, 0, 0, 1;
	p << 1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 0, 0, 1, 0;

	// Create camera transformation matrix.
	Eigen::Matrix4d TM = p * (s * (r * t));

    vopt.geomgroup[0] = 0;
    vopt.geomgroup[1] = 1;
    vopt.geomgroup[2] = 0;

    int pointCount = 0;

    // Send rays for each and every pixel!
    for(int c=0; c<camera_.getWidth(); ++c) {
      for(int r=0; r<camera_.getHeight(); ++r) {

    	// Obtain ray, and rotate it.
    	glm::vec3 rayDir = camera_.projectPixelTo3dRay(cv::Point2f(c,r), newCamPos, viewportCenter, normGaze, normRight, viewUp);

    	double t1[3] = {newCamPos[0], newCamPos[1], newCamPos[2]};
    	double t2[3] = {rayDir[0], rayDir[1], rayDir[2]};

		// compute ray from pixel and camera configuration
		double distance  = mj_ray (m, d, t1, t2, vopt.geomgroup, mjVIS_CONVEXHULL, -1, &geomId);

		if (distance != -1){

			glm::vec3 newPoint = newCamPos + (float)distance * rayDir;

			// Find hit point of the ray and new ray direction.
			temp = normalize(rayDir);
			temp = temp * (float) distance;
			tempP = newCamPos + temp;
			temp = tempP - newCamPos2;
			temp = normalize(temp);

	    	double t1[3] = {newCamPos2[0], newCamPos2[1], newCamPos2[2]};
	    	double t2[3] = {temp[0], temp[1], temp[2]};

			// check if point is also visible in second camera by casting a ray to this point
			double distance2  = mj_ray(m, d, t1 , t2, vopt.geomgroup, mjVIS_CONVEXHULL, -1, &geomId2);

			if (distance2 != -1){

				// Find hit point wrt the second camera.
				temp = temp * (float)distance2;
				tempP2 = newCamPos2 + temp;

				// Compare two hit points!
				Point diff(tempP2[0] - tempP[0], tempP2[1] - tempP[1],tempP2[2] - tempP[2]);
				if(abs(diff)<0.0001) {

				  // get pixel position of ray in right image
				  cv::Point2f left_pixel = camera_.project3dToPixel(tempP, TM);

				  // quantize right_pixel
//				  left_pixel.x = round(left_pixel.x*8.0)/8.0;
//				  left_pixel.y = round(left_pixel.y*8.0)/8.0;

				  // compute disparity image
				  float quant_disp = (float)c - left_pixel.x;

				  float* disp_i = disp.ptr<float>(r);
				  disp_i[(int)c] = quant_disp;
				  pointCount ++;
				}
			}
		} // if mesh reached from right camera
      } // camera_.getHeight()
    } // camera_.getWidth()

    // Filter disparity image and add noise 
    cv::Mat out_disp;
    out_disp = disp;
    filterDisp(disp, out_disp);

    // Allocate space for cloud.
    pcl::PointCloud<pcl::PointXYZRGBNormal> *cloud = new pcl::PointCloud<pcl::PointXYZRGBNormal>(pointCount, 1);

    // Reset point counter.
    pointCount = 0;

    //Go over disparity image and recompute depth map and point cloud after filtering and adding noise etc
    for(int r=0; r<camera_.getHeight(); ++r) {
      float* disp_i = out_disp.ptr<float>(r);
      double* depth_map_i = depth_map.ptr<double>(r);
      for(int c=0; c<camera_.getWidth(); ++c) {
	float disp = disp_i[camera_.getWidth() - c];
	if(disp<invalid_disp_){
	  cv::Point3d new_p;
	  new_p.z = (camera_.getFx()*camera_.getTx())/disp;

	  if(new_p.z<camera_.getZNear() || new_p.z>camera_.getZFar()){
	    continue;
	  }
	  new_p.x = (new_p.z/ camera_.getFx()) * (c - camera_.getCx());
	  new_p.y = (new_p.z/ camera_.getFy()) * (r - camera_.getCy());

	  // Set x-y-z coordinates of points.
	  cloud->points[pointCount].x = new_p.x;
	  cloud->points[pointCount].y = new_p.y;
	  cloud->points[pointCount].z = new_p.z;

	  // Assign colours to these poins.
  	  cloud->points[pointCount].r = 127;
  	  cloud->points[pointCount].g = 127;
  	  cloud->points[pointCount].b = 127;
	  pointCount++;

	  depth_map_i[(int)c] = new_p.z;
	}
      }
    }

 // Convert into real world coordinates.
    Eigen::Matrix4d T = s * (r * tOrg);
    Eigen::Matrix4d invT = T.inverse();

	for (int i = 0; i < pointCount; i++) {
	  Eigen::Vector4d p(cloud->points[i].x, cloud->points[i].y, cloud->points[i].z, 1);
	  p = invT * p;
	  p = p / p(3);
	  cloud->points[i].x = (float) p[0] + 0.5;
	  cloud->points[i].y = (float) p[1];
	  cloud->points[i].z = (float) p[2];
	}

	return cloud;
  }

  // filter disparity with a 9x9 correlation window
  void KinectSimulator::filterDisp(const cv::Mat& disp, cv::Mat& out_disp)
  {
    cv::Mat interpolation_map = cv::Mat::zeros(disp.rows,disp.cols, CV_32FC1);
    
    cv::Mat noise_field;
    if(noise_type_==NONE)
      noise_field = cv::Mat::zeros(disp.rows,disp.cols, CV_32FC1);
    else 
      // generate noise field according to given noise type
      // can be Gaussian, Perlin or Simplex
      noise_gen_->generateNoiseField(noise_field);

    // mysterious parameter
    float noise_smooth = 1.5;

    // initialise output arrays
    out_disp = cv::Mat(disp.rows, disp.cols, disp.type());
    out_disp.setTo(invalid_disp_);

    // determine filter boundaries
    unsigned lim_rows = std::min(camera_.getHeight()-size_filt_, dot_pattern_.rows-size_filt_);
    unsigned lim_cols = std::min(camera_.getWidth()-size_filt_, dot_pattern_.cols-size_filt_);
    int center = size_filt_/2.0;
    for(unsigned r=0; r<lim_rows; ++r) {
      const float* disp_i = disp.ptr<float>(r+center);
      const float* dots_i = dot_pattern_.ptr<float>(r+center);
      float* out_disp_i = out_disp.ptr<float>(r+center);
      float* noise_i =  noise_field.ptr<float>((int)((r+center)/noise_smooth));
      
      // window shifting over disparity image
      for(unsigned c=0; c<lim_cols; ++c) {
	if( dots_i[c + center]>0 && disp_i[c + center] < invalid_disp_){
	  cv::Rect roi = cv::Rect(c, r, size_filt_, size_filt_);
	  cv::Mat window = disp(roi);
	  cv::Mat dot_win = dot_pattern_(roi);
	  // check if we are at a occlusion boundary without valid disparity values
	  // return value not binary but between 0 or 255
	  cv::Mat valid_vals = (window<invalid_disp_);
	  cv::Mat valid_dots;
	  cv::bitwise_and( valid_vals, dot_win, valid_dots);
	  cv::Scalar n_valids = cv::sum(valid_dots)/255.0;
	  cv::Scalar n_thresh = cv::sum(dot_win)/255.0;

	  // only add depth value at center of window if there are more
	  // valid disparity values than 2/3 of the number of dots
	  if( n_valids(0) > n_thresh(0)/1.5 ) {
	    // compute mean only over the valid values of disparities in that window
	    cv::Scalar mean = cv::mean(window, valid_vals);
	    // weighted deviation from mean
	    cv::Mat diffs = cv::abs(window-mean);
	    cv::multiply( diffs, weights_, diffs);
	    // get valid values that fall on dot pattern
	    cv::Mat valids = (diffs<window_inlier_distance_);
	    cv::bitwise_and( valids, valid_dots, valid_dots);
	    n_valids = cv::sum(valid_dots)/255.0;

	    // only add depth value at center of window if there are more
	    // valid disparity values than 2/3 of the number of dots
	    if(n_valids(0)>n_thresh(0)/1.5) {
	      float accu = window.at<float>(center,center);
	      assert(accu<invalid_disp_);
	      out_disp_i[c + center] = round((accu + noise_i[(int)((c+center)/noise_smooth)])*8.0)/8.0;

	      cv::Mat interpolation_window = interpolation_map(roi);
	      cv::Mat disp_data_window = out_disp(roi);
	      cv::Mat label_data_window;
	      cv::Mat substitutes = interpolation_window < fill_weights_;
	      fill_weights_.copyTo( interpolation_window, substitutes);
	      disp_data_window.setTo(out_disp_i[c + center], substitutes);
	    }
	  }
	}
      }
    }
  }


}    //namespace render_kinect

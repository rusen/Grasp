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
#include <opencv2/imgproc/imgproc.hpp>

#include <sensor/camera.h>
#include <sensor/gaussian.h>
#include <sensor/perlin.h>
#include <sensor/simplex.h>

#include <string>
#include <limits>
#include <stdio.h>
#include <glm/gtc/type_ptr.hpp>
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <random>

#include <Eigen/Geometry>
#define PI 3.14159265

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
	float std  = 0.075;
	noise_gen_ = new GaussianNoise( camera_.getWidth(), camera_.getHeight(), mean, std);
      } else if (noise_type_==Grasp::PERLIN)
      {
	float scale = 0.2;
	noise_gen_ = new PerlinNoise( camera_.getWidth(), camera_.getHeight(), scale);
      } else if (noise_type_==Grasp::SIMPLEX)
      {
	float scale = 0.25;
	noise_gen_ = new SimplexNoise( camera_.getWidth(), camera_.getHeight(), scale);
      }
  }

  // Destructor
  KinectSimulator::~KinectSimulator() 
  {
	  delete noise_gen_;
  }
  
/*
// Function that intersects rays with the object model at current state.
void KinectSimulator::captureRGB(const mjModel* m, mjData* d,
			      mjvScene *scn,
				  mjrContext *con,
				  cv::Mat &rgb_map,
				  glm::vec3 newCamPos,
				  glm::vec3 newCamGaze,
				  char * rgbFile)
{

	newCamGaze = glm::normalize(newCamGaze);
	// Next, we find the view-right vector (right hand vector)
    glm::vec3 tempUp(0, 0, 1), viewRight, normRight;
    normRight = glm::normalize(glm::cross(newCamGaze, tempUp));
    glm::vec3 rgbCamPos = newCamPos + normRight * (float) camera_.getColourTx();
    glm::vec3 rgbCamLookAt = rgbCamPos + newCamGaze;
    mjvOption vopt;
    mjv_defaultOption(&vopt);

    // Get RGB image.
    mjvCamera rgbCam;
    mjv_defaultCamera(&rgbCam);
    rgbCam.lookat[0] = rgbCamLookAt[0], rgbCam.lookat[1] = rgbCamLookAt[1], rgbCam.lookat[2] = rgbCamLookAt[2];
    rgbCam.distance = 1;
    float valx = newCamGaze[0];
    float valy = newCamGaze[1];
    float l = sqrt(valx*valx + valy*valy);
    valx /= l;
    valy /= l;
    if (valy > 0)
    {
    	rgbCam.azimuth = (acos(valx) * 180) / PI;
    }else
    {
    	rgbCam.azimuth = -(acos(valx) * 180) / PI;
    }

    rgbCam.elevation = (asin(newCamGaze[2]) * 180.0) / PI;
    mjrRect viewport = {0, 0, 640, 480};

    // Set visualization options.
    mjvOption opt;
    mjv_defaultOption(&opt);
	opt.geomgroup[0] = 1;
	opt.geomgroup[1] = 0;
	opt.geomgroup[2] = 1;
	opt.geomgroup[3] = 0;
	opt.geomgroup[4] = 0;
    for (int i = 0; i<18; i++)
    {
    	if (i != 17)
    		opt.flags[i] = 0;
    }
    // Render scene!
    mjv_updateScene(m, d, &opt, NULL, &rgbCam, mjCAT_ALL, scn);
    mjr_render(viewport, scn, con);
	unsigned char rgbBuffer[640*480*3];

    // Read image.
	mjr_readPixels(rgbBuffer, NULL, viewport, con);
    for (int p = 0; p<640*480*3; p++)
    {
    	rgb_map.data[p] = rgbBuffer[p];
    }

    // Flip for opencv
    cv::Mat out;
    cv::flip(rgb_map, out, 0);
    cv::imwrite( rgbFile, out );
}
*/

// Function that intersects rays with the object model at current state.
pcl::PointCloud<pcl::PointXYZ> * KinectSimulator::intersect(const mjModel* m, mjData* d,
				  cv::Mat &depth_map,
				  glm::vec3 newCamPos,
				  glm::vec3 newCamGaze,
				  float minPointZ,
				  std::ofstream * ostream,
				  glm::quat * q)
  {
	// Default option for the camera.
    mjvOption vopt;
    mjv_defaultOption(&vopt);

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
    glm::vec3 emitterPos(0,0,0), rgbCamPos, temp, temp2, tempP, tempP2;
    temp = normRight * (float) camera_.getTx();
    emitterPos = newCamPos + temp;

    // Finally, get view-up vector.
    glm::vec3 viewUp;
    viewUp = normalize(glm::cross(normRight, normGaze));

    // Print the vectors.
    (*ostream)<<"CAM POS: "<< newCamPos[0]<<" "<< newCamPos[1]<<" "<< newCamPos[2]<<std::endl;
    (*ostream)<<"RGB CAM POS: "<< rgbCamPos[0]<<" "<< rgbCamPos[1]<<" "<< rgbCamPos[2]<<std::endl;
    (*ostream)<<"CAM GAZE: "<< normGaze[0]<<" "<< normGaze[1]<<" "<< normGaze[2]<<std::endl;
    (*ostream)<<"CAM VIEWPORT: "<< viewportCenter[0]<<" "<< viewportCenter[1]<<" "<< viewportCenter[2]<<std::endl;
    (*ostream)<<"CAM RIGHT: "<< normRight[0]<<" "<< normRight[1]<<" "<< normRight[2]<<std::endl;
    (*ostream)<<"CAM UP: "<< viewUp[0]<<" "<< viewUp[1]<<" "<< viewUp[2]<<std::endl;
    (*ostream)<<"EMITTER POS: "<< emitterPos[0]<<" "<< emitterPos[1]<<" "<< emitterPos[2]<<std::endl;

    // Create transformation vector for second camera.
	glm::mat4 r, s, t, tOrg, p;
	s = glm::mat4(-1, 0, 0, 0,
		  0, 1, 0, 0,
		  0, 0, 1, 0,
		  0, 0, 0, 1);
	r = glm::mat4(normRight[0], viewUp[0], normGaze[0], 0,
			normRight[1],	viewUp[1], normGaze[1], 0,
			normRight[2], viewUp[2],  normGaze[2], 0,
			0, 0, 0, 1);
	t = glm::mat4(1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 -emitterPos[0], -emitterPos[1], -emitterPos[2], 1);
	tOrg = glm::mat4(1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 0,
		 -newCamPos[0], -newCamPos[1], -newCamPos[2], 1);
	p = glm::mat4(1, 0, 0, 0,
		 0, 1, 0, 0,
		 0, 0, 1, 1,
		 0, 0, 0, 0);

	// Create camera transformation matrix.
	glm::mat4 TM = p * (s * (r * t));
    vopt.geomgroup[0] = 0;
    vopt.geomgroup[1] = 1;
    vopt.geomgroup[2] = 0;

	// TODO: PRINT TM
    /*
    std::cout<<" PRINTING TM "<<std::endl;
	std::cout<<TM[0][0]<<" "<<TM[0][1]<<" "<<TM[0][2]<<" "<<TM[0][3]<<std::endl;
	std::cout<<TM[1][0]<<" "<<TM[1][1]<<" "<<TM[1][2]<<" "<<TM[1][3]<<std::endl;
	std::cout<<TM[2][0]<<" "<<TM[2][1]<<" "<<TM[2][2]<<" "<<TM[2][3]<<std::endl;
	std::cout<<TM[3][0]<<" "<<TM[3][1]<<" "<<TM[3][2]<<" "<<TM[3][3]<<std::endl;
    std::cout<<"  **************************************  "<<std::endl;
	*/

    // find camera transformation matrix (and quaternion)
    glm::vec3 tv = glm::normalize(glm::vec3(normGaze[0], normGaze[1], 0));
    float angle2;
    if (tv[1] > 0)
    	angle2 = -(acos(tv[0]) - M_PI/2);
    else
    	angle2 = -((2*M_PI - acos(tv[0])) - M_PI/2);
    float dist1 = sqrt(normGaze[0] * normGaze[0] + normGaze[1] * normGaze[1]);
    float angle3 = atan(normGaze[2]/dist1);
    Eigen::Matrix3f m2;
    Eigen::Quaternionf q1(Eigen::AngleAxisf(-0.5*M_PI, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf q2(Eigen::AngleAxisf(angle2, Eigen::Vector3f::UnitY()));
    Eigen::Quaternionf q3(Eigen::AngleAxisf(angle3, Eigen::Vector3f::UnitX()));
    Eigen::Quaternionf q4(Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitZ()));
    m2 = q1 * q2 * q3 * q4;

    /*
    std::cout<<"PRINTING MATRIX"<<std::endl;
    std::cout<<m2(0)<<" "<<m2(1)<<" "<<m2(2)<<std::endl;
    std::cout<<m2(3)<<" "<<m2(4)<<" "<<m2(5)<<std::endl;
    std::cout<<m2(6)<<" "<<m2(7)<<" "<<m2(8)<<std::endl;
    */

    Eigen::Quaternionf qTmp2(m2);
    q->w = qTmp2.w();
    q->x = qTmp2.x();
    q->y = qTmp2.y();
    q->z = qTmp2.z();

    // Send rays for each and every pixel!
    int pointCount = 0;
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
			temp = tempP - emitterPos;
			temp = normalize(temp);

	    	double t1[3] = {emitterPos[0], emitterPos[1], emitterPos[2]};
	    	double t2[3] = {temp[0], temp[1], temp[2]};

			// check if point is also visible in second camera by casting a ray to this point
			double distance2  = mj_ray(m, d, t1 , t2, vopt.geomgroup, mjVIS_CONVEXHULL, -1, &geomId2);

			if (distance2 != -1){

				// Find hit point wrt the second camera.
				temp = temp * (float)distance2;
				tempP2 = emitterPos + temp;

				// Compare two hit points!
				glm::vec3 diff(tempP2[0] - tempP[0], tempP2[1] - tempP[1],tempP2[2] - tempP[2]);
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
    float f = std::numeric_limits<float>::quiet_NaN();
    pcl::PointXYZ pNull;
    pNull.x = f;
    pNull.y = f;
    pNull.z = f;
    pcl::PointCloud<pcl::PointXYZ> *tempCloud = new pcl::PointCloud<pcl::PointXYZ>(640, 480, pNull);
    tempCloud->is_dense = true;
	tempCloud->points.resize(640*480);

    // Matrix transformations
    glm::mat4 T = s * (r * tOrg);
    glm::mat4 invT = glm::inverse(T);

    float addedNoiseX = 0;
    float addedNoiseY = 0;
    float addedNoiseZ = 0;

    srand(time(NULL));
    int idx1 = rand()%1000, idx2 = rand()%1000, idx3 = rand()%1000;
	std::default_random_engine generator;
	std::normal_distribution<double> distribution(0.0,0.006);
	for (int ktr = 0; ktr < 1000; ktr++)
	{
		float no = distribution(generator);
		if (idx1 == ktr)
			addedNoiseX = no;
		if (idx2 == ktr)
			addedNoiseY = no;
		if (idx3 == ktr)
			addedNoiseZ = no;
	}
	std::cout<<"Sampled calibration error in cms: "<<addedNoiseX*100 << " "<< addedNoiseY*100 << " "<<addedNoiseZ*100 << std::endl;

    //Go over disparity image and recompute depth map and point cloud after filtering and adding noise etcs
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

	  // Check if minimum point is below z threshold
	  glm::vec4 p(new_p.x, new_p.y, new_p.z, 1);
	  p = invT * p;
	  p = p / p[3];

	  // If the point is below the minimum plane, remove it from the point cloud.
	  if (p.z >= minPointZ)
	  {
		  // Save point
		  tempCloud->points[r*640+c].x = (float) p.x + 0.5 + addedNoiseX;
		  tempCloud->points[r*640+c].y = (float) p.y + addedNoiseY;
		  tempCloud->points[r*640+c].z = (float) p.z + addedNoiseZ;
	  }

	  // save z to a depth map.
	  depth_map_i[(int)c] = new_p.z;
	}
      }
    }

    // Return new point cloud.
	return tempCloud;
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

#ifndef __MARK_FOLLOWER_CLASS_H__
#define __MARK_FOLLOWER_CLASS_H__

// ROS Lib
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/Pose2D.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Float64.h>
#include <mark_follower/markPoseStamped.h>
#include <ids_viewer/IDSparams.h>
// Standard Lib
#include <wchar.h>
#include <stdlib.h>
#include <stdio.h>
#include <iostream>
#include <sys/time.h>
#include <sstream>
#include <thread>
// OpenCV Lib
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
// Custom  Lib
#include <SortList.h>
#include <enum_header.h>
#include <KDTreeDescObj.h>

#define CAMERA_VID_WIDTH 		1280 
#define CAMERA_VID_HEIGHT 		720
#define CAMERA_VID_BPP 		24

// WHITE HSV CHANNEL
#define CAM_WHITE_H_MIN		0
#define CAM_WHITE_H_MAX		80
#define CAM_WHITE_S_MIN		00
#define CAM_WHITE_S_MAX		80
#define CAM_WHITE_V_MIN		60
#define CAM_WHITE_V_MAX		255
// BLACK HSV CHANNEL
#define CAM_BLACK_H_MIN		0
#define CAM_BLACK_H_MAX		120
#define CAM_BLACK_S_MIN		0	
#define CAM_BLACK_S_MAX		120
#define CAM_BLACK_V_MIN		0
#define CAM_BLACK_V_MAX		120
// BLUE HSV CHANNEL
#define CAM_BLUE_LH				85 // 75
#define CAM_BLUE_HH			160
#define CAM_BLUE_LS				150 //170 
#define CAM_BLUE_HS 			255
#define CAM_BLUE_LV 			195 //205
#define CAM_BLUE_HV			255
// RED HSV CHANNEL
#define CAM_RED_LH			161 //164 //0 
#define CAM_RED_HH			255 //179 //35
#define CAM_RED_LS			120 //229 //0
#define CAM_RED_HS	 		255 //255
#define CAM_RED_LV 			180 //100 //55
#define CAM_RED_HV			255 //253
// BLACK HSV CHANNEL
#define CAM_BLACK_LH		0
#define CAM_BLACK_HH		0
#define CAM_BLACK_LS		0
#define CAM_BLACK_HS	 	0
#define CAM_BLACK_LV 		0
#define CAM_BLACK_HV		100
// GREEN field HSV CHANNEL
#define CAM_GREEN_LH		0 
#define CAM_GREEN_HH		255
#define CAM_GREEN_LS		0
#define CAM_GREEN_HS 		255
#define CAM_GREEN_LV 		0
#define CAM_GREEN_HV		190
// YELLOW field HSV CHANNEL
#define CAM_YELLOW_LH		0 
#define CAM_YELLOW_HH		255
#define CAM_YELLOW_LS		0
#define CAM_YELLOW_HS 		255
#define CAM_YELLOW_LV 		0
#define CAM_YELLOW_HV		190
// WHITE field HSV CHANNEL
#define CAM_WHITE_LH		0 
#define CAM_WHITE_HH		255
#define CAM_WHITE_LS		0
#define CAM_WHITE_HS 		255
#define CAM_WHITE_LV 		180
#define CAM_WHITE_HV		255

// CHALLENGES
#define  FIRST_CHALLENGE 	0
#define  THIRD_CHALLENGE 	1


#define MAX_BLOB_SIZE		1000000
#define MIN_BLOB_SIZE		200//((altitude_ > 22.5)?1000:(altitude_ > 17.5)?2500:(altitude_ > 12.5)?3500:(altitude_ > 7.5)?6500:(altitude_ > 5)?20000:(altitude_ > 3.5)?42000:(altitude_ > 2.5)?90000:150000)
   
				
#define RAD2DEG				(180.0 / M_PI)

#define VIDEO_NAME			"/home/solaris/recognition"

using namespace std;
using namespace cv;

namespace cam_vid {

	class mark_follower_class{

	public:
		// Ros constructor
		mark_follower_class(ros::NodeHandle*, const bool verbose = false);
		
		~mark_follower_class();

	private:

		void get_frame(const char*);
		void get_static_frame();

		cv::Mat morph_operation(cv::Mat, const bool inv = false);
		cv::Mat otsuTH(cv::Mat, bool inv = false);
		void t_matching(descriptor, cv::Mat);
		vector<descriptor> get_blob(const cv::Mat);
		vector<descriptor> get_blobFirstCh(const cv::Mat);
		vector<descriptor> get_blobThirdCh(const cv::Mat);
		// vector<descriptor> get_blobThirdCh(const cv::Mat, const cv::Mat);
		float tau_extraction(const cv::Mat);
		float estimateTau(vector<Point2f>, vector<Point2f>, Point2f);
		Point2f estimateFOE(vector<Point2f>, vector<Point2f>);
		cv::Mat adjust_rotation(descriptor);
		void mark_obj(string, const cv::Point, const cv::Point, const cv::Scalar);
		cv::Mat color_detection(const cv::Mat, const colors);
		cv::Mat remove_field(cv::Mat);
		void calculate_histogram(cv::Mat, const int , int* );
		cv::Point get_median(cv::Mat);
		colors get_color(cv::Mat, cv::Point);

		// Callback function
		void imageFirstChallengeCallback(const sensor_msgs::ImageConstPtr&);
		void imageThirdChallengeCallback(const sensor_msgs::ImageConstPtr&);
		void altitudeCallback(const std_msgs::Float64Ptr&);

		// Kalman Function

		void init_kalman(const cv::Point);
		cv::Point kalman(const cv::Point);

		// Further functions

		float approx_perpendicular(double, double);
		std::vector<cv::Point> getIntersection(std::vector<Vec4i>);
		cv::Point searchNPG(std::vector<Point>);

		double diff_ms(const timeval, const timeval);


		void trial();

		// Variables

		bool verbose_;

		// Camera params
		int width_, height_;

		// Pyramid frames

		cv::Mat ocvMat_; // Original image
		cv::Mat ocvMat_lv1_; // Half image resolution
		cv::Mat ocvMat_lv2_; // Quad image resolution
	
		cv::Mat tmpl_;

		cv::VideoWriter outVid_;

		cv::KalmanFilter* KF_;
		bool state_kf_;

		cv::Point target_;

		// Control tau
		vector<Point2f> points_[2]; // Good point features
		cv::Mat prevGray_;
		cv::Mat medianWinTau_;
		int tauCount_;
		cv::Mat ROI_;

		// SortList 

		SortList sl;

		// Ros node

		ros::NodeHandle* n_;

		// FPV publisher
		image_transport::Publisher image_aug_pub_;

		// ROS SITL operations
		image_transport::Subscriber image_tr_sub_;
		ros::Publisher target_pos_pub_;
		// no SITL
		
		ros::Subscriber state_sub_;
		ros::Subscriber altitude_sub_;

		ros::Publisher overrideRCIn_pub_;

		ros::Publisher markTarget_pub_;

		int budgetResidual_;
		float refVariance_;

		// Flight params
		float altitude_;

		bool challenge_;
		bool sitl_;

		/// ------------------------------------------------------->Color Range<-------------------------------------------------------

		int iLowH;
		int iLowHd;
		int iHighH;

		int iLowS; 
		int iHighS;

		int iLowV;
		int iHighV;

		/// -------------------------------------------------------------------------------------------------------------------------------------

		cv::VideoCapture* cap_;


		/// ----------------------------------------------------> Time manager <----------------------------------------------------

		struct timeval time_after, time_before;

		/// -------------------------------------------------------------------------------------------------------------------------------------

	};
} 
#endif 


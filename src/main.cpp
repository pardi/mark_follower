#include "mark_follower_class.h"

using namespace cam_vid;

int main(int argc, char** argv){

	// INIT ros
	ros::init(argc, argv, "camera_class");

	ros::NodeHandle n;

	// Call mark_follower_class constructor
	mark_follower_class mfc(&n);
	
	return 0;
}
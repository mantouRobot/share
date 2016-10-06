#include "NxCamera.hpp"

int main( int argc, char** argv )
{
	ros::init(argc,argv,"orbcamera_node");
	ros::NodeHandle nh;
	NxCamera orb_cam(nh);
	orb_cam.Init();
	ros::AsyncSpinner spinner(6); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
    return 0;
}

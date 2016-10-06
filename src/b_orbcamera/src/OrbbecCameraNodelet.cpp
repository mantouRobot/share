#include "NxCamera.hpp"
#include <nodelet/nodelet.h>
// this should really be in the implementation (.cpp file)




namespace b_orbcamera
{

    class OrbbecCameraNodelet : public nodelet::Nodelet
    {
    protected:
    	NxCamera* orb_cam;

		virtual void onInit()
		{
			orb_cam = new NxCamera(getNodeHandle());
			orb_cam->Init();
		}

    public:
		~OrbbecCameraNodelet()
		{
			if(orb_cam != NULL)
			{
				delete orb_cam;
				orb_cam = NULL;
			}
		}
    };

}

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(b_orbcamera::OrbbecCameraNodelet, nodelet::Nodelet)
PLUGINLIB_DECLARE_CLASS(b_orbcamera, OrbbecCameraNodelet, b_orbcamera::OrbbecCameraNodelet, nodelet::Nodelet);

/*
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
*/


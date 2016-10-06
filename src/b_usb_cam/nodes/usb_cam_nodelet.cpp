#include "usb_cam_node.cpp"
#include <nodelet/nodelet.h>
// this should really be in the implementation (.cpp file)




namespace usb_cam
{

    class usb_cam_nodelet : public nodelet::Nodelet
    {
    protected:
    	usb_cam::UsbCamNode* cusb_cam;
		virtual void onInit()
		{
      cusb_cam = new usb_cam::UsbCamNode(getNodeHandle(), getPrivateNodeHandle());
		}

    public:
		~usb_cam_nodelet()
		{
			if(cusb_cam != NULL)
			{
				delete cusb_cam;
				cusb_cam = NULL;
			}
		}
    };

}

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(b_orbcamera::OrbbecCameraNodelet, nodelet::Nodelet)
PLUGINLIB_DECLARE_CLASS(usb_cam, usb_cam_nodelet, usb_cam::usb_cam_nodelet, nodelet::Nodelet);

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


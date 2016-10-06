#include "NxCamera.hpp"
#include <nodelet/nodelet.h>
// this should really be in the implementation (.cpp file)
#include <image_transport/image_transport.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <image_geometry/pinhole_camera_model.h>


namespace b_orbcamera
{

    class TestOrbbecCameraNodelet : public nodelet::Nodelet
    {
    protected:
    	boost::shared_ptr<image_transport::ImageTransport> it_;
    	image_transport::CameraSubscriber sub_depth_;
		virtual void onInit()
		{
			ros::NodeHandle nh = getNodeHandle();
			it_.reset(new image_transport::ImageTransport(nh));
		    image_transport::TransportHints hints("raw", ros::TransportHints(), getPrivateNodeHandle());
		    sub_depth_ = it_->subscribeCamera("camera/depth/image", 1, &TestOrbbecCameraNodelet::DepthCb, this, hints);
			//image_transport::ImageTransport it(nh);

			//depth_sub = it.subscribe("camera/depth/image", 1, TestOrbbecCameraNodelet::DepthCb,this);

		}

		void DepthCb(const sensor_msgs::ImageConstPtr& depth_msg,
                const sensor_msgs::CameraInfoConstPtr& info_msg)
		{
			//double value = depth_msg->data[0];
			ROS_INFO("subscribe depth image in nodelet");
		}

    public:
		~TestOrbbecCameraNodelet()
		{
		}
    };

}

// watch the capitalization carefully
#include <pluginlib/class_list_macros.h>
//PLUGINLIB_EXPORT_CLASS(b_orbcamera::OrbbecCameraNodelet, nodelet::Nodelet)
PLUGINLIB_DECLARE_CLASS(b_orbcamera, TestOrbbecCameraNodelet, b_orbcamera::TestOrbbecCameraNodelet, nodelet::Nodelet);

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


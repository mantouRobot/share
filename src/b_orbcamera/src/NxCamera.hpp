#include <iostream>
#include "NiTE.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <boost/thread.hpp>
#include "NxSkeleton.hpp"
#include "OrbbecCamera.hpp"
// using namespace
using namespace std;
using namespace nite;

class NxCamera {
private:
	ros::NodeHandle nh;
	NxSkeleton nxskeleton;
	OrbbecCamera orbbecCamera;
public:
	openni::Device m_device;
public:
	NxCamera(ros::NodeHandle in_nh){
		nh = in_nh;
	}

	~NxCamera() {
	}

	int Init() {
		InitDevice();

		InitOrbbecCamera();
		InitNxSkeleton();
	}

	int InitDevice() {
		openni::Status rc = openni::OpenNI::initialize();
		if (rc != openni::STATUS_OK) {
			ROS_INFO("Failed to initialize OpenNI\n%s\n",
					openni::OpenNI::getExtendedError());
			return rc;
		} else {
			ROS_INFO("Success to initialize OpenNI\n%s\n",
					openni::OpenNI::getExtendedError());
		}

		const char* deviceUri = openni::ANY_DEVICE;
		rc = m_device.open(deviceUri);

		if (rc != openni::STATUS_OK) {
			ROS_INFO("Failed to open device\n%s\n",
					openni::OpenNI::getExtendedError());
			return rc;
		} else {
			printf("Success to open device\n%s\n",
					openni::OpenNI::getExtendedError());
		}

	}

	int InitNxSkeleton() {
		nxskeleton.Init(&nh, &m_device);
	}

	int InitOrbbecCamera() {
		orbbecCamera.Init(&nh, &m_device);
	}

};

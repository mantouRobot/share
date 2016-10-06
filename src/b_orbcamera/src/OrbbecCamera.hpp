#include <iostream>
#include "NiTE.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/LaserScan.h>
#include <boost/thread.hpp>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include "../include/depth_conversions1.h"

#include <tf/transform_listener.h>
#include <message_filters/subscriber.h>
#include <tf/message_filter.h>

#include "CameraParameter.hpp"


//线程休眠，频率
#define  BOOST_SLEEP(n)  boost::thread::sleep(boost::get_system_time()+boost::posix_time::millisec(1000.0/n))
//图像数据发布频率
#define PUB_RATE_FREE 1
#define PUB_RATE_RUN 35 //深度图只能发布到25hz左右

//#define CAMERA_PARAMS_INI   "./camera_params.ini"

using namespace std;
using namespace nite;

class OrbbecCamera
{
private:
	ros::NodeHandle* pnh;

	// published topics
	ros::Publisher pub_depth_cam_info,pub_depth_regist_cam_info, pub_pointcloud, pub_laser;
	image_transport::Publisher pub_depth,pub_depth_regist;
	boost::thread* 	depthpub_thread_,*depth_regist_pub_thread_,*pointcloudpub_thread_,*laserpub_thread_;
	bool 			pub_depth_status,pub_depth_regist_status,pub_pointcloud_status,pub_laser_status;
	double 			depthpub_rate,depth_regist_pub_rate,pointcloudpub_rate,laserpub_rate;


	double prev_time_stamp_;

	openni::VideoFrameRef 	m_frame,m_depth_regist_frame,m_pointcloud_frame,m_laser_frame;
	openni::VideoStream 	m_stream_depth,m_stream_depth_regist,m_pointcloud_stream,m_laser_stream;

	openni::Device* 		m_pdevice;
	image_geometry::PinholeCameraModel pc_model_, laser_model_;

	//laser param
	double 	min_height_, max_height_, angle_min_, angle_max_, angle_increment_, scan_time_, range_min_, range_max_;
	bool 	use_inf_;
	double 	smooth_factor_;
	std::string	target_frame_;

	//laser to base_footprint
	tf::TransformListener tf_listener_;
	tf::StampedTransform tf_pixel_;

	//tf2

	boost::mutex connect_mutex_;

	double regist_mat[16];

public:
	OrbbecCamera() :
		tf_listener_(ros::Duration(10.0))
	{
		depthpub_thread_ = NULL;
		pointcloudpub_thread_ = NULL;
		laserpub_thread_ = NULL;
	}

	~OrbbecCamera()
	{
		DestroyThread(depthpub_thread_);
		DestroyThread(pointcloudpub_thread_);
		DestroyThread(laserpub_thread_);

	}

	int Init(ros::NodeHandle* in_nh, openni::Device* in_pdevice)
	{
		pnh = in_nh;
		m_pdevice = in_pdevice;//相机设备

		//初始化相机
		InitStream(m_pdevice, openni::SENSOR_DEPTH, m_stream_depth);
		InitStream(m_pdevice, openni::SENSOR_DEPTH, m_stream_depth_regist);
		InitStream(m_pdevice, openni::SENSOR_DEPTH, m_pointcloud_stream);
		InitStream(m_pdevice, openni::SENSOR_DEPTH, m_laser_stream);

		image_transport::ImageTransport it(*pnh);

		//深度图
		pub_depth_cam_info = pnh->advertise<sensor_msgs::CameraInfo>("camera/depth/camera_info",1);
		depthpub_rate 		= 	PUB_RATE_FREE;
		pub_depth_status = false;
    pub_depth = it.advertise("camera/depth/image", 1);
		depthpub_thread_ = new boost::thread(boost::bind(&OrbbecCamera::PublicDepthThread, this));

		//深度图 配准的
		pub_depth_regist_cam_info = pnh->advertise<sensor_msgs::CameraInfo>("camera/depth_regist/camera_info",1);
		depth_regist_pub_rate 		= 	PUB_RATE_FREE;
		pub_depth_regist_status = false;
		pub_depth_regist = it.advertise("camera/depth_regist/image", 1);
		depth_regist_pub_thread_ = new boost::thread(boost::bind(&OrbbecCamera::PublicDepthRegistThread, this));

		//点云
		pointcloudpub_rate    =	PUB_RATE_FREE;
		pub_pointcloud_status = false;
		pub_pointcloud = pnh->advertise<sensor_msgs::PointCloud2>("camera/depth/points", 1);
		pointcloudpub_thread_ = new boost::thread(boost::bind(&OrbbecCamera::PublicPointCloudThread, this));

		//激光
		ros::NodeHandle pnh_("~");
		pnh_.param<double>("min_height", min_height_, -0.0);
		pnh_.param<double>("max_height", max_height_, 1.5);//based by base_footprint
		pnh_.param<double>("angle_min", angle_min_, -0.5);
		pnh_.param<double>("angle_max", angle_max_, 0.5);
		pnh_.param<double>("angle_increment", angle_increment_, 0.0032);
		pnh_.param<double>("scan_time", scan_time_, 0.033);
		pnh_.param<double>("range_min", range_min_, 0.4);
		pnh_.param<double>("range_max", range_max_, 8.0);
		pnh_.param<bool>("use_inf", use_inf_, true);
		pnh_.param<double>("smooth_factor", smooth_factor_, 0.5);
		laserpub_rate    = 	PUB_RATE_FREE;
		pub_laser_status = false;
		pub_laser = pnh->advertise<sensor_msgs::LaserScan>("/scan", 5);
		laserpub_thread_ = new boost::thread(boost::bind(&OrbbecCamera::PublicLaserThread, this));

		CameraParameter camera_parameter;
		char CAMERA_PARAMS_INI[50] = "./camera_params.ini";
		bool isCameraParamsExist = camera_parameter.checkFile(CAMERA_PARAMS_INI);
		if (isCameraParamsExist)
		{
			camera_parameter.loadCameraParams(CAMERA_PARAMS_INI);
			camera_parameter.calcALLMat(regist_mat);
		}else
		{
			camera_parameter.calcALLMatDefault(regist_mat);
		}

		return 0;
	}

private:
	int PublicDepthThread()
	{
		ros::Rate rate(25);
		while(1)
		{
			BOOST_SLEEP(depthpub_rate);
			DepthStream(m_pdevice,openni::SENSOR_DEPTH);
		}
	}

	int PublicDepthRegistThread()
	{
		ros::Rate rate(25);
		while(1)
		{
			BOOST_SLEEP(depth_regist_pub_rate);
			DepthRegistStream(m_pdevice,openni::SENSOR_DEPTH);
		}
	}


	int PublicPointCloudThread()
	{
		while(1)
		{
			BOOST_SLEEP(pointcloudpub_rate);
			PointCloudStream(m_pdevice,openni::SENSOR_DEPTH);
		}
	}

	int PublicLaserThread()
	{
		while(1)
		{
			BOOST_SLEEP(laserpub_rate);
			LaserStream(m_pdevice,openni::SENSOR_DEPTH);
		}
	}

	/**
	 * 	消毁线程
	 */
	bool DestroyThread(boost::thread* thread_)
	{
		if(thread_ != NULL)
		{
			thread_->interrupt();
			thread_->join();
			thread_ = NULL;
			return true;
		}
		return true;
	}

	void ResetPubRate(ros::Rate* rate, double freq)
	{
		if(rate != NULL)
		{
			delete rate;
			rate = NULL;
		}
		rate = new ros::Rate(freq);
	}

	sensor_msgs::ImagePtr CreateDepImage(string frame_str,int set_value)
	{
		sensor_msgs::ImagePtr image(new sensor_msgs::Image);

		ros::Time ros_now = ros::Time::now();
		image->header.stamp = ros_now;
		image->header.frame_id = frame_str;//"camera_depth_optical_frame";
		ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

		prev_time_stamp_ = ros_now.toSec();
		image->width = m_frame.getWidth();
		image->height = m_frame.getHeight();
		ROS_DEBUG("frame width,height:%d,%d",image->width,image->height);

		std::size_t data_size = m_frame.getDataSize();

		image->data.resize(data_size);
		if(set_value==-1)
		{
			memcpy(&image->data[0], m_frame.getData(), data_size);
		}else
		{
			memset(&image->data[0], set_value, data_size);

		}


		image->is_bigendian = 0;

		const openni::VideoMode& video_mode = m_frame.getVideoMode();
		//ROS_INFO("video format:%d",video_mode.getPixelFormat());
		switch (video_mode.getPixelFormat())
		{
			case openni::PIXEL_FORMAT_DEPTH_1_MM:
				image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
				image->step = sizeof(unsigned char) * 2 * image->width;
				break;
			default:
				ROS_ERROR("Invalid image encoding");
				break;
		}
		return image;
	}

	int DepthStream(openni::Device* device, openni::SensorType stype)
	{
		if(pub_depth.getNumSubscribers()== 0)
		{
			if(pub_depth_status == true)
			{
				depthpub_rate = PUB_RATE_FREE;
				m_stream_depth.stop();
				pub_depth_status = false;
				ROS_INFO("Success stop the depth stream!");
			}
			return 0;
		}

		if((pub_depth.getNumSubscribers()>0) && pub_depth_status == false)
		{

			openni::Status rc = m_stream_depth.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start the depth stream\n%s\n", openni::OpenNI::getExtendedError());
				return 4;
			}
			depthpub_rate = PUB_RATE_RUN;
			pub_depth_status = true;
			ROS_INFO("Success start the depth stream!");

		}

		m_stream_depth.readFrame(&m_frame);

		if (m_frame.isValid())
		{
			sensor_msgs::ImagePtr image= CreateDepImage("camera_depth_optical_frame",-1);
			// Update camera model
			sensor_msgs::CameraInfoPtr camerainfoPtr =getDepthInfo(image->header.stamp);

			pc_model_.fromCameraInfo(camerainfoPtr);
			pub_depth.publish(image);
			pub_depth_cam_info.publish(getDepthInfo(image->header.stamp));
		}
		return 0;
	}


	int DepthRegistStream(openni::Device* device, openni::SensorType stype)
	{
		if(pub_depth_regist.getNumSubscribers()== 0)
		{
			if(pub_depth_regist_status == true)
			{
				depth_regist_pub_rate = PUB_RATE_FREE;
				m_stream_depth_regist.stop();
				pub_depth_regist_status = false;
				ROS_INFO("Success stop the depth regist stream!");
			}
			return 0;
		}

		if(pub_depth_regist.getNumSubscribers()>0 && pub_depth_regist_status == false)
		{

			openni::Status rc = m_stream_depth_regist.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start the depth regist stream\n%s\n", openni::OpenNI::getExtendedError());
				return 4;
			}
			depth_regist_pub_rate = PUB_RATE_RUN;
			pub_depth_regist_status = true;
			ROS_INFO("Success start the depth regist stream!");

		}

		m_stream_depth_regist.readFrame(&m_frame);

		if (m_frame.isValid())
		{
			sensor_msgs::ImagePtr image= CreateDepImage("camera_depth_optical_frame",-1);
			sensor_msgs::ImagePtr image_regist= CreateDepImage("camera_rgb_optical_frame",0);
			// Update camera model
			sensor_msgs::CameraInfoPtr camerainfoPtr =getDepthInfo(image->header.stamp);

			pc_model_.fromCameraInfo(camerainfoPtr);


			if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
			{
				depth_proc::convert2depth_regist<uint16_t>(image,image_regist, regist_mat, pc_model_/*, tf_pixel_*/);
			}
			else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
			{
				depth_proc::convert2depth_regist<float>(image,image_regist, regist_mat, pc_model_/*, tf_pixel_*/);
			}
			else
			{
				ROS_ERROR_THROTTLE(5, "Depth regist image has unsupported encoding [%s]", image->encoding.c_str());
				return -1;
			}
			pub_depth_regist.publish(image_regist);
			pub_depth_regist_cam_info.publish(getDepthInfo(image_regist->header.stamp));
		}
		return 0;
	}


	int PointCloudStream(openni::Device* device, openni::SensorType stype)
	{
		if(pub_pointcloud.getNumSubscribers()== 0)
		{
			if(pub_pointcloud_status == true)
			{
				m_pointcloud_stream.stop();
				pub_pointcloud_status = false;
				ROS_ERROR("Success stop the pointcloud stream!");
				pointcloudpub_rate = PUB_RATE_FREE;
			}
			//ROS_ERROR("no publish point cloud!");
			return 0;
		}

		if(pub_pointcloud.getNumSubscribers()>0 && pub_pointcloud_status == false)
		{
			openni::Status rc = m_pointcloud_stream.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start the pointcloud stream\n%s\n", openni::OpenNI::getExtendedError());
				return 4;
			}

			pub_pointcloud_status = true;
			ROS_INFO("Success start the pointcloud stream!");
			pointcloudpub_rate=PUB_RATE_RUN;
			return 0;
		}

		//ROS_ERROR("to publish point cloud!");
		m_pointcloud_stream.readFrame(&m_pointcloud_frame);

		if (m_pointcloud_frame.isValid())
		{
			sensor_msgs::ImagePtr image(new sensor_msgs::Image);

			ros::Time ros_now = ros::Time::now();
			image->header.stamp = ros_now;

			ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

			prev_time_stamp_ = ros_now.toSec();
			image->width = m_pointcloud_frame.getWidth();
			image->height = m_pointcloud_frame.getHeight();

			std::size_t data_size = m_pointcloud_frame.getDataSize();

			image->data.resize(data_size);
			memcpy(&image->data[0], m_pointcloud_frame.getData(), data_size);

			image->is_bigendian = 0;

			const openni::VideoMode& video_mode = m_pointcloud_frame.getVideoMode();
			//ROS_INFO("video format:%d",video_mode.getPixelFormat());
			switch (video_mode.getPixelFormat())
			{
				case openni::PIXEL_FORMAT_DEPTH_1_MM:
					image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
					image->step = sizeof(unsigned char) * 2 * image->width;
					break;
				default:
					ROS_ERROR("Invalid image encoding");
					break;
			}
			sensor_msgs::PointCloud2::Ptr cloud_msg(new sensor_msgs::PointCloud2);
			cloud_msg->header.stamp = ros_now;
			cloud_msg->header.frame_id = "camera_depth_optical_frame";

			cloud_msg->header = image->header;
			cloud_msg->height = image->height;
			cloud_msg->width  = image->width;
			cloud_msg->is_dense = false;
			cloud_msg->is_bigendian = false;

			sensor_msgs::PointCloud2Modifier pcd_modifier(*cloud_msg);
			pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

			// Update camera model
			sensor_msgs::CameraInfoPtr camerainfoPtr =getDepthInfo(ros_now);
			pc_model_.fromCameraInfo(camerainfoPtr);
			ROS_DEBUG("encoding:%s",image->encoding.c_str());

			if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
			{
				depth_proc::convert<uint16_t>(image, cloud_msg, pc_model_/*, tf_pixel_*/);
			}
			else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
			{
				depth_proc::convert<float>(image, cloud_msg, pc_model_/*, tf_pixel_*/);
			}
			else
			{
				ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", image->encoding.c_str());
				return -1;
			}
			cloud_msg->header.stamp = ros_now;
			cloud_msg->header.frame_id = "camera_depth_optical_frame";
			pub_pointcloud.publish(cloud_msg);
		}
		return 0;
	}

	int LaserStream(openni::Device* device, openni::SensorType stype)
	{
		if(pub_laser.getNumSubscribers()== 0)
		{
			if(pub_laser_status == true)
			{
				laserpub_rate=PUB_RATE_FREE;
				m_laser_stream.stop();
				pub_laser_status = false;
				ROS_INFO("Success stop the laser stream!");
			}
			return 0;
		}

		if(pub_laser.getNumSubscribers()>0 && pub_laser_status == false)
		{
			openni::Status rc = m_laser_stream.start();
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't start the laser stream\n%s\n", openni::OpenNI::getExtendedError());
				return 4;
			}
			laserpub_rate=PUB_RATE_RUN;
			pub_laser_status = true;
			ROS_INFO("Success start the laser stream!");

		}

		m_laser_stream.readFrame(&m_laser_frame);

		if (m_laser_frame.isValid())
		{
			sensor_msgs::ImagePtr image(new sensor_msgs::Image);

			ros::Time ros_now = ros::Time::now();
			image->header.stamp = ros_now;

			ROS_DEBUG("Time interval between frames: %.4f ms", (float)((ros_now.toSec()-prev_time_stamp_)*1000.0));

			prev_time_stamp_ = ros_now.toSec();
			image->width = m_laser_frame.getWidth();
			image->height = m_laser_frame.getHeight();

			std::size_t data_size = m_laser_frame.getDataSize();

			image->data.resize(data_size);
			memcpy(&image->data[0], m_laser_frame.getData(), data_size);

			image->is_bigendian = 0;

			const openni::VideoMode& video_mode = m_laser_frame.getVideoMode();
			//ROS_INFO("video format:%d",video_mode.getPixelFormat());
			switch (video_mode.getPixelFormat())
			{
			case openni::PIXEL_FORMAT_DEPTH_1_MM:
				image->encoding = sensor_msgs::image_encodings::TYPE_16UC1;
				image->step = sizeof(unsigned char) * 2 * image->width;
				break;
			default:
				ROS_ERROR("Invalid image encoding");
				break;
			}

			sensor_msgs::LaserScan laser;
			laser.header.stamp = ros_now;
			laser.header.frame_id = "base_footprint";//head_link
			if (!target_frame_.empty())//建议取消该参数
			{
				laser.header.frame_id = target_frame_;
			}
			laser.angle_min = angle_min_;
			laser.angle_max = angle_max_;
			laser.time_increment = 0.0;
			laser.scan_time = scan_time_;
			laser.range_min = range_min_;
			laser.range_max = range_max_;
			laser.angle_increment = angle_increment_;

			//determine amount of rays to create used by pub_
			uint32_t ranges_size = std::ceil((laser.angle_max - laser.angle_min) / laser.angle_increment);

			//determine if laserscan rays with no obstacle data will evaluate to infinity or max_range
			if (use_inf_)
			{
				laser.ranges.assign(ranges_size, std::numeric_limits<double>::infinity());
			}
			else
			{
				laser.ranges.assign(ranges_size, laser.range_max + 1.0);
			}

			// Update camera model
			sensor_msgs::CameraInfoPtr camerainfoPtr =getDepthInfo(ros_now);
			laser_model_.fromCameraInfo(camerainfoPtr);
			ROS_DEBUG("encoding:%s",image->encoding.c_str());

			//tf
			try
			{
				tf_listener_.waitForTransform("base_footprint", "camera_depth_optical_frame", ros::Time::now(), ros::Duration(5));
				tf_listener_.lookupTransform("base_footprint", "camera_depth_optical_frame", laser.header.stamp, tf_pixel_);
			}
			catch(tf::TransformException ex)
			{
				ROS_ERROR_STREAM("Couldn't transform from "<<"camera_depth_optical_frame"<<" to "<< "base_footprint");
				ROS_ERROR("%s",ex.what());
				return 1;
			}

			//ROS_INFO_STREAM(tf_pixel_.getOrigin().x());
			if (image->encoding == sensor_msgs::image_encodings::TYPE_16UC1)
			{	//ROS_INFO("conver2laser uint16");
				depth_proc::convert2laser<uint16_t>(image, laser, laser_model_, max_height_, min_height_, tf_pixel_);
			}
			else if (image->encoding == sensor_msgs::image_encodings::TYPE_32FC1)
			{
				//ROS_INFO("conver2laser float");
				depth_proc::convert2laser<float>(image, laser, laser_model_, max_height_, min_height_, tf_pixel_);
			}
			else
			{
				ROS_ERROR_THROTTLE(5, "Depth image has unsupported encoding [%s]", image->encoding.c_str());
				return -1;
			}
			//ROS_INFO("pub laser...");
			pub_laser.publish(laser);
		}
		return 0;
	}

	bool InitStream(openni::Device* device, openni::SensorType stype,openni::VideoStream& stream)
	{
		bool flag = true;
		if (device->getSensorInfo(stype) != NULL)
		{
			openni::Status rc = stream.create(*device, stype);
			if (rc != openni::STATUS_OK)
			{
				printf("Couldn't create stream stream\n%s\n", openni::OpenNI::getExtendedError());
				return 3;
			}

			// set video mode
			openni::VideoMode vmMode;
			vmMode.setFps( 30 );
			vmMode.setResolution( 640, 480 );
			vmMode.setPixelFormat( openni::PIXEL_FORMAT_DEPTH_1_MM );
			//openni::CameraSettings* camera_seeting = stream->getCameraSettings();
			if( stream.setVideoMode( vmMode ) == openni::STATUS_OK )
			{
				// OK
				ROS_INFO("Success set up the video model!");
				stream.setMirroringEnabled(false);
			}else
			{
				ROS_INFO("Failed set up the video model!");
			}
		}
		return flag;
	}

	sensor_msgs::CameraInfoPtr getDepthInfo(ros::Time tm)
	{
		sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
		info->header.stamp=tm;
		info->header.frame_id = "camera_depth_frame";
		info->width  = 640;
		info->height = 480;

		// No distortion
		info->D.resize(5, 0.0);
		info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;

		// Simple camera matrix: square pixels (fx = fy), principal point at center
		info->K.assign(0.0);
		info->K[0] = 570.3422241210938;
		info->K[2] = 314.5;
		// Aspect ratio for the camera center on Kinect (and other devices?) is 4/3
		// This formula keeps the principal point the same in VGA and SXGA modes
		info->K[4] = 570.3422241210938;
		info->K[5] = 235.5;
		info->K[8] = 1.0;

		// No separate rectified image plane, so R = I
		info->R.assign(0.0);
		info->R[0] = info->R[4] = info->R[8] = 1.0;

		// Then P=K(I|0) = (K|0)
		info->P.assign(0.0);
		info->P[0]  = 570.3422241210938;
		info->P[2]  = 314.5;
		info->P[5]  = 570.3422241210938;
		info->P[6]  = 235.5;
		info->P[10] = 1.0;

		info->binning_x = 0;
		info->binning_y = 0;
		info->roi.x_offset=0;
		info->roi.y_offset= 0;
		info->roi.height=0;
		info->roi.width=0;
		info->roi.do_rectify= false;

		return info;
	}

};

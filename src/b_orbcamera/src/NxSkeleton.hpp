#include <iostream>
#include <strstream>
#include "NiTE.h"
#include "ros/ros.h"
#include <tf/transform_broadcaster.h>
#include "b_orbcamera/skeltrack.h"
#include <boost/thread.hpp>
#include <std_msgs/String.h>
// using namespace
using namespace std;
using namespace nite;

class NxSkeleton
{
private:
	ros::NodeHandle* nh;
	ros::ServiceServer switch_srv;//start/stop skeleton service
	bool enabled;
	bool start_skeleton_flag;
	boost::thread* campub_thread_;

	openni::Device*		m_pdevice;

	int raisehand_count;
	ros::Publisher bodypose_msg_pub,bodyaction_msg_pub;

	bool open_skeleton;
public:
	NxSkeleton()
	{
		campub_thread_ = NULL;
	}

	~NxSkeleton()
	{
		DestroyThread(campub_thread_);
	}

	int Init(ros::NodeHandle* in_nh, openni::Device* in_pdevice)
	{
		nh = in_nh;

		m_pdevice = in_pdevice;
		switch_srv = nh->advertiseService("s_skeleton", &NxSkeleton::ChangeSkeletonStateSrvCb, this);

		nh->param<bool>("b_orbcamera_nodelet/open_skeleton", open_skeleton, false);
		if(open_skeleton)StartThread();

		raisehand_count = 0;
		bodyaction_msg_pub = nh->advertise<std_msgs::String>("skeleton/body_action",1);
		bodypose_msg_pub = nh->advertise<std_msgs::String>("skeleton/body_pose",1);
		SetSkeleton2ParamService(0,"");

	}

	void StartThread()
	{
		enabled = true;
		campub_thread_ = new boost::thread(boost::bind(&NxSkeleton::PublicSkeletonTFThread, this));
	}

	void StopThread()
	{
		enabled = false;
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

	int PublicSkeletonTFThread()
	{
		ros::Rate rate(1);
		ROS_INFO("running skeleton tracking!");
		TrackingbyCenterandClosest();
		ROS_INFO("stopped skeleton tracking!");
	}

	void PublishTransform(const nite::SkeletonJoint& node,  const string& frame_id,  const string& child_frame_id) {
	    static tf::TransformBroadcaster br;
	    double x = -node.getPosition().x/ 1000.0;
	    double y = node.getPosition().y/ 1000.0;
	    double z = node.getPosition().z/ 1000.0;
	    double qx = node.getOrientation().x;
	    double qy = node.getOrientation().y;
	    double qz = node.getOrientation().z;
	    double qw = node.getOrientation().w;
	    if(qw == 0)qw = 1;

	    tf::Transform transform;
		transform.setOrigin(tf::Vector3(x, y, z));
		transform.setRotation(tf::Quaternion(qx, -qy, -qz, qw));

		tf::Transform change_frame;
		change_frame.setOrigin(tf::Vector3(0, 0, 0));
		tf::Quaternion frame_rotation;
		frame_rotation.setEulerZYX(1.5708, 0, 1.5708);
		change_frame.setRotation(frame_rotation);

		transform = change_frame * transform;
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), frame_id, child_frame_id));
	}

	void PublishTransforms(const nite::UserData& user, const std::string& frame_id)
	{
		const char * const joint_name[] = {"head","neck","left_shoulder","right_shoulder",
					"left_elbow","right_elbow","left_hand","right_hand","torso",
					"left_hip","right_hip","left_knee","right_knee","left_foot","right_foot"};
		for(int i = 0; i < 15; i++)
			PublishTransform(user.getSkeleton().getJoint((JointType)i), frame_id, joint_name[i]);
	}

	void SetSkeleton2ParamService(double curr_time_sec, string body_action_str)
	{
		nh->setParam("/body_action_name", body_action_str.c_str());
		nh->setParam("/body_action_time", curr_time_sec);
	}

	void DetectRaiseHand(const UserData& user)
	{
		const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
		double x_right_shoulder = -right_shoulder.getPosition().x/ 1000.0;
		double y_right_shoulder = right_shoulder.getPosition().y/ 1000.0;

		const nite::SkeletonJoint& right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
		double x_right_hand = -right_hand.getPosition().x/ 1000.0;
		double y_right_hand = right_hand.getPosition().y/ 1000.0;
		ROS_DEBUG("y_right_shoulder::%f,y_right_hand::%f,y_thres:%f;x_thres:%f",y_right_shoulder, y_right_hand,y_right_shoulder - y_right_hand, fabs(x_right_shoulder - x_right_hand));

		const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
		double x_left_shoulder = -left_shoulder.getPosition().x/ 1000.0;
		double y_left_shoulder = left_shoulder.getPosition().y/ 1000.0;

		const nite::SkeletonJoint& left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
		double x_left_hand = -left_hand.getPosition().x/ 1000.0;
		double y_left_hand = left_hand.getPosition().y/ 1000.0;
		ROS_DEBUG("y_left_shoulder::%f,y_left_hand::%f,y_thres:%f;x_thres:%f",y_left_shoulder, y_left_hand,y_left_shoulder - y_left_hand, fabs(x_left_shoulder - x_left_hand));

		if((y_right_shoulder < y_right_hand && fabs(x_right_shoulder - x_right_hand)< 0.5)
				|| (y_left_shoulder < y_left_hand && fabs(x_left_shoulder - x_left_hand)< 0.5))
		{
			raisehand_count++;
			if(raisehand_count > 12)raisehand_count=12;
		}else
		{
			raisehand_count--;
			if(raisehand_count < 0)raisehand_count=0;
		}
		ROS_DEBUG("raisehand_count:%d",raisehand_count);
		if(raisehand_count>= 10)
		{
			std_msgs::String str;
			str.data = "hand_raise_up";
			SetSkeleton2ParamService(ros::Time::now().toSec(),str.data);
			bodyaction_msg_pub.publish(str);
		}
	}

public:
	bool ChangeSkeletonStateSrvCb(b_orbcamera::skeltrack::Request &request, b_orbcamera::skeltrack::Response& response)
	{
		if ((enabled == true) && (request.type == request.END))
		{
		  ROS_INFO("Change mode service request: the skeleton tracking stopped");
		  SetSkeleton2ParamService(0,"");
		  StopThread();
		  response.status = response.SUCCESS;
		}
		else if ((enabled == false) && (request.type == request.RUN))
		{
		  ROS_INFO("Change mode service request: the skeleton tracking (re)started");
		  StartThread();
		  SetSkeleton2ParamService(0,"");
		  response.status = response.RUNNING;
		}else if(enabled == true)
		{
			ROS_INFO("Change mode service request: nxskeleton is running, no need to restart");
			response.status = response.RUNNING;
		}else if(enabled == false)
		{
			response.status = response.SUCCESS;
		}
		return true;
	}

	int TrackingbyCenterandClosest()
	{
		nite::NiTE::initialize();
		nite::UserTracker userTracker;
		nite::Status rcn = userTracker.create(m_pdevice);
		if (rcn != nite::STATUS_OK)
		{
			//#REP101:it is usually caused by incorrect OpenNI, please specify the correct Version.
			//the correct version is :RedistforNite, not Redist.
			ROS_INFO("Couldn't create user tracker : %d\n",rcn);
			return openni::STATUS_ERROR;
		}

		ros::Rate rate(50);
		while(true)
		{
			rate.sleep();
			if(enabled == false)
				break;
			// 读取帧信息
			UserTrackerFrameRef mUserFrame;
			userTracker.readFrame( &mUserFrame);

			// 通过帧信息，获得用户数据UserData
			const Array<UserData>& aUsers = mUserFrame.getUsers();

			if(aUsers.getSize()==0)
				continue;
			for( int i = 0; i < aUsers.getSize(); ++ i )
			{
				const UserData& rUser = aUsers[i];
				const UserId& uID = rUser.getId();
				if( rUser.isNew()){
					userTracker.startSkeletonTracking(uID);
					ROS_INFO_THROTTLE(2, "New User %d.", uID);
				}
				else if( rUser.isLost()){
					ROS_INFO_THROTTLE(2, "User %d lost.", uID);
					continue;
				}else if(!rUser.isVisible()){
					ROS_INFO_THROTTLE(2, "User %d is invisible.", uID);
					continue;
				}
			}

			UserId bestID = FindAvailablePerson(aUsers);
			//PublicAvailableUser(aUsers);
			PublicAllAvailableUsers(aUsers);
			if(bestID == -1)continue;
			const UserData& rUser = aUsers[bestID];
			if(rUser.isVisible() == false)continue;//不再视野范围内，不发布TF变换

			//ROS_INFO_THROTTLE(2,"Available UserID %d",rUser.getId());
			DetectRaiseHand(aUsers[bestID]);
			PublishTransforms(aUsers[bestID], "camera_depth_frame");//publish skeleton TF
		}

		// 关闭UserTracker跟踪
		userTracker.destroy();

		// 关闭NITE环境
		NiTE::shutdown();
		return 0;
	}

	int FindAvailablePerson(const Array<UserData>& aUsers)
	{
		int bestUId = -1;
		float dist = 1000000;
		for( int i = 0; i < aUsers.getSize(); ++ i )
		{
			const nite::Skeleton& rSkeleton = aUsers[i].getSkeleton();
			if( rSkeleton.getState() == nite::SKELETON_TRACKED && aUsers[i].isVisible()==true)
			{
				const SkeletonJoint& joint = rSkeleton.getJoint(nite::JOINT_TORSO);
				if(joint.getPosition().z<dist && joint.getPosition().x <=3000 && joint.getPosition().x >=-3000)
				{
					dist = joint.getPosition().z;
					bestUId = i;
				}
			}
		}
		return bestUId;
	}

	bool PublicAvailableUser(const Array<UserData>& aUsers)
	{
		int bestID = -1;
		float x,y,z,bestx,besty,bestz;
		bestz = 1000000;
		for( int i = 0; i < aUsers.getSize(); ++ i )
		{
			if(aUsers[i].isVisible()==true){
				x = aUsers[i].getCenterOfMass().x;
				y = aUsers[i].getCenterOfMass().y;
				z = aUsers[i].getCenterOfMass().z;
				if(z<bestz && x <=3000 && x >=-3000){
					bestx = x;besty = y;bestz = z;
					bestID = i;
				}
			}
		}
		if(bestID != -1)
		{
			std_msgs::String str_msg;
			char position[50];
			sprintf(position,"%f,%f,%f",bestz/1000,-bestx/1000,besty/1000);
			std::string str(position);
			str_msg.data = str;
			bodypose_msg_pub.publish(str_msg);

		}

		return true;
	}

	bool PublicAllAvailableUsers(const Array<UserData>& aUsers)
	{
		bool is_pub = false;
		float x,y,z,bestx,besty,bestz;
		bestz = 1000000;
		std::string users_info = "";

		for( int i = 0; i < aUsers.getSize(); ++ i )
		{
			if(aUsers[i].isVisible()==true){
				is_pub = true;
				ostringstream ss;
				ss<<aUsers[i].getId()<<","
					<<aUsers[i].getCenterOfMass().z/1000<<","
					<<-aUsers[i].getCenterOfMass().x/1000<<","
					<<aUsers[i].getCenterOfMass().y/1000<<";";
				users_info += ss.str();
				ss.clear();
			}
		}


		if(is_pub)
		{
			users_info = users_info.substr(0,users_info.size()-1);
			std_msgs::String str_msg;
			str_msg.data = users_info;
			bodypose_msg_pub.publish(str_msg);

		}

		return true;
	}

	bool PublicAllAvailableUsersSkeleton(const Array<UserData>& aUsers)
	{
		bool is_pub = false;
		float x,y,z,bestx,besty,bestz;
		bestz = 1000000;
		std::string users_info = "";
		for( int i = 0; i < aUsers.getSize(); ++ i )
		{

			if(aUsers[i].isVisible()==true){
				is_pub = true;
				ostringstream ss;
				ss<<aUsers[i].getId()<<","
					<<aUsers[i].getCenterOfMass().z/1000<<","
					<<-aUsers[i].getCenterOfMass().x/1000<<","
					<<aUsers[i].getCenterOfMass().y/1000<<";";
				users_info += ss.str();
				ss.clear();
			}
		}


		if(is_pub)
		{
			users_info = users_info.substr(0,users_info.size()-1);
			std_msgs::String str_msg;
			str_msg.data = users_info;
			bodypose_msg_pub.publish(str_msg);

		}

		return true;
	}
};

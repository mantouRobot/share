/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, Willow Garage, Inc.
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
 *   * Neither the name of the Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
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
#ifndef DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS
#define DEPTH_IMAGE_PROC_DEPTH_CONVERSIONS

#include <sensor_msgs/Image.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <image_geometry/pinhole_camera_model.h>
#include "depth_traits.h"
#include <limits>

#include <tf/transform_listener.h>

namespace depth_proc {

typedef sensor_msgs::PointCloud2 PointCloud;

float ployp[3] = { 0.037276029192477515, 1.0218646632132087,
		-0.00532943751654565 };
float p_orig[3] = { 0, 0, -2 };

// Handles float or uint16 depths
template<typename T>
void convert(const sensor_msgs::ImageConstPtr& depth_msg,
		PointCloud::Ptr& cloud_msg,
		const image_geometry::PinholeCameraModel& model,
		/*tf::StampedTransform& tf_to_pixel,*/
		double range_max = 0.0) {
	// Use correct principal point from calibration
	float center_x = model.cx();
	float center_y = model.cy();

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = DepthTraits<T>::toMeters(T(1));
	float constant_x = unit_scaling / model.fx();
	float constant_y = unit_scaling / model.fy();

	ROS_INFO("%f,%f,%f,%f,%f",model.cx(),model.cy(),model.fx(),model.fy(),unit_scaling);
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);

//  tf::Transform tf_matrix(tf_to_pixel.getRotation(), tf_to_pixel.getOrigin());
	//tf::Transform tf_matrix(tf_to_pixel.getRotation(), tf_to_pixel.getOrigin());

	for (int v = 0; v < (int) cloud_msg->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) cloud_msg->width;
				++u, ++iter_x, ++iter_y, ++iter_z) {
			T depth = depth_row[u];

			// Missing points denoted by NaNs
			if (!DepthTraits<T>::valid(depth)) {
				if (range_max != 0.0) {
					depth = DepthTraits<T>::fromMeters(range_max);
				} else {
					*iter_x = *iter_y = *iter_z = bad_point;
					continue;
				}
			}

			// Fill in XYZ
			*iter_x = (u - center_x) * depth * constant_x;
			*iter_y = (v - center_y) * depth * constant_y;
			*iter_z = DepthTraits<T>::toMeters(depth);

			float OA0 = sqrt((-*iter_x - p_orig[0]) * (-*iter_x - p_orig[0])
							+ (*iter_z - p_orig[2]) * (*iter_z - p_orig[2]));
			float OA = OA0 - fabs(p_orig[2]);
			*iter_z = ployp[0] * OA * OA + ployp[1] * OA + ployp[2];

//						tf::Vector3 v(*iter_x, *iter_y, *iter_z);
//						v = tf_matrix * v;
//						*iter_x = v.getX();
//						*iter_y = v.getY();
//						*iter_z = v.getZ();
		}
	}
}

// Handles float or uint16 depths
template<typename T>
void convert2laser(const sensor_msgs::ImageConstPtr& depth_msg,
		sensor_msgs::LaserScan& laser_msg,
		const image_geometry::PinholeCameraModel& model, float max_height_,
		float min_height_, tf::StampedTransform& tf_to_pixel) {
	// Use correct principal point from calibration
	float center_x = model.cx();
	float center_y = model.cy();

	// Combine unit conversion (if necessary) with scaling by focal length for computing (X,Y)
	double unit_scaling = DepthTraits<T>::toMeters(T(1));
	float constant_x = unit_scaling / model.fx();
	float constant_y = unit_scaling / model.fy();
	float bad_point = std::numeric_limits<float>::quiet_NaN();

	float pc_x, pc_y, pc_z;
	//sensor_msgs::PointCloud2Iterator<float> iter_x(*cloud_msg, "x");
	//sensor_msgs::PointCloud2Iterator<float> iter_y(*cloud_msg, "y");
	//sensor_msgs::PointCloud2Iterator<float> iter_z(*cloud_msg, "z");
	const T* depth_row = reinterpret_cast<const T*>(&depth_msg->data[0]);
	int row_step = depth_msg->step / sizeof(T);

	float base_footprint_x, base_footprint_y, base_footprint_z;
	tf::Transform tf_matrix(tf_to_pixel.getRotation(), tf_to_pixel.getOrigin());
//    ROS_INFO("TF_R:%.2f, %.2f, %.2f, %.2f", tf_matrix.getRotation().getX(), tf_matrix.getRotation().getY(), tf_matrix.getRotation().getZ(), tf_matrix.getRotation().getW());
//    ROS_INFO("TF_T:%.2f, %.2f, %.2f", tf_matrix.getOrigin().getX(), tf_matrix.getOrigin().getY(), tf_matrix.getOrigin().getZ());

	for (int v = 0; v < (int) depth_msg->height; ++v, depth_row += row_step) {
		for (int u = 0; u < (int) depth_msg->width; ++u) {
			T depth = depth_row[u];
			// Missing points denoted by NaNs
			if (!DepthTraits<T>::valid(depth))
				continue;
			// Fill in XYZ
			pc_x = (u - center_x) * depth * constant_x;
			pc_y = (v - center_y) * depth * constant_y;
			pc_z = DepthTraits<T>::toMeters(depth);
			float OA0 = sqrt(
					(-pc_x - p_orig[0]) * (-pc_x - p_orig[0])
							+ (pc_z - p_orig[2]) * (pc_z - p_orig[2]));
			float OA = OA0 - fabs(p_orig[2]);
			pc_z = ployp[0] * OA * OA + ployp[1] * OA + ployp[2];
			//0.09712 0.03725
//            base_footprint_x = pc_z + 0.09712;
//            base_footprint_y = -pc_x + 0.03725;
//            base_footprint_z = -pc_y;

			tf::Vector3 v(pc_x, pc_y, pc_z);
			v = tf_matrix * v;
			base_footprint_x = v.x();
			base_footprint_y = v.y();
			base_footprint_z = v.z();
//            ROS_INFO("test_v:%.2f, %.2f, %.2f", base_footprint_x, base_footprint_y, base_footprint_z);

			if (std::isnan(base_footprint_x) || std::isnan(base_footprint_y)
					|| std::isnan(base_footprint_z))
				continue;
			if (base_footprint_z > max_height_
					|| base_footprint_z < min_height_)
				continue;
			double range = hypot(base_footprint_x, base_footprint_y);
			if (range < laser_msg.range_min)
				continue;
			double angle = atan2(base_footprint_y, base_footprint_x);
			if (angle < laser_msg.angle_min || angle > laser_msg.angle_max)
				continue;
			//overwrite range at laserscan ray if new range is smaller
			int index = (angle - laser_msg.angle_min)
					/ laser_msg.angle_increment;
			if (range < laser_msg.ranges[index])
				laser_msg.ranges[index] = range;
		}
	}
}

} // namespace depth_image_proc

#endif

/*
 *  Single Marker Pose Estimation using ARToolkit
 *  Copyright (C) 2013, I Heart Engineering
 *  Copyright (C) 2010, CCNY Robotics Lab
 *  William Morris <bill@iheartengineering.com>
 *  Ivan Dryanovski <ivan.dryanovski@gmail.com>
 *  Gautier Dumonteil <gautier.dumonteil@gmail.com>
 *  http://www.iheartengineering.com
 *  http://robotics.ccny.cuny.edu
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef AR_POSE_AR_SINGLE_H
#define AR_POSE_AR_SINGLE_H

//GENERAL
#include <string.h>
#include <stdarg.h>

//AR TOOL KIT
#include <artoolkit/AR/param.h>
#include <artoolkit/AR/ar.h>
#include <artoolkit/AR/video.h>

//ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf/transform_broadcaster.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <visualization_msgs/Marker.h>
#include <ar_pose/ARMarker.h>

//OPENCV
#include <opencv/cv.h>
#include <opencv/highgui.h>

//new cv_bridge API in Groovy
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

//Vikit for camera calibration
#include <vikit/cameras/camera_geometry_base.h>
#include <vikit/cameras/ncamera.h>
#include <vikit/cameras/pinhole_projection.h>
#include <vikit/cameras/radial_tangential_distortion.h>
#include <vikit/params_helper.h>


const double AR_TO_ROS = 0.001;

namespace ar_pose
{
  class ARSinglePublisher
  {
  public:
    ARSinglePublisher (ros::NodeHandle & n);
    ~ARSinglePublisher (void);

  private:
    void arInit();
    bool loadCameraParameters(const std::string& camera_calib_file);
    void getTransformationCallback (const sensor_msgs::ImageConstPtr &);
    void camInfoCallback (const sensor_msgs::CameraInfoConstPtr &);

    ros::NodeHandle n_;
    ros::Publisher pub_tag_pose_;
    ros::Publisher pub_camera_pose_;
    tf::TransformBroadcaster broadcaster_;
    tf::Transform  transform_quad_camera_

    ar_pose::ARMarker ar_pose_marker_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber sub_image_;
    ros::Subscriber sub_camera_info_;
    sensor_msgs::CameraInfo cam_info_;

    // **** parameters
    std::string camera_frame_;
    std::string marker_frame_;
    bool publish_tf_;
    bool publish_visual_markers_;
    bool use_history_;
    int threshold_;
    double marker_width_;        // Size of the AR Marker in mm

    ARParam cam_param_;         // Camera Calibration Parameters
    int patt_id_;               // AR Marker Pattern
    char pattern_filename_[FILENAME_MAX];
    bool reverse_transform_;    // Reverse direction of transform marker -> cam

    double marker_center_[2];   // Physical Center of the Marker
    double marker_trans_[3][4]; // Marker Transform

    cv::Mat camera_matrix_;
    cv::Mat dist_coeffs_;

    // **** for visualisation in rviz
    ros::Publisher pub_rviz_marker;
    ros::Publisher arMarkerPub_;
    visualization_msgs::Marker rvizMarker_;

    bool marker_prev_visualized_;
    bool get_camera_info_;
    cv::Size sz_;
    cv_bridge::CvImagePtr capture_;
  };                            // end class ARSinglePublisher
}                               // end namespace ar_pose

#endif

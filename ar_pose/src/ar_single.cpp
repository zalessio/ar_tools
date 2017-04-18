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

#include "ar_pose/ar_single.h"

int main (int argc, char **argv)
{
  ros::init (argc, argv, "ar_single");
  ros::NodeHandle n;
  ar_pose::ARSinglePublisher arSingle(n);
  ros::spin();
  return 0;
}

namespace ar_pose
{
  ARSinglePublisher::ARSinglePublisher (ros::NodeHandle & n)
  :n_ (n), it_ (n_), marker_prev_visualized_(false),
   camera_matrix_(3, 3, CV_32FC1),
   dist_coeffs_(4, 1, CV_32FC1)
  {
    std::string local_path;
    std::string package_path = ros::package::getPath (ROS_PACKAGE_NAME);
    std::string default_path = "data/patt.ruag";
    ros::NodeHandle n_param ("~");
    XmlRpc::XmlRpcValue xml_marker_center;

      
    ROS_INFO("Starting ArSinglePublisher");
    // **** get parameters
    if (!n_param.getParam("publish_tf", publish_tf_))
      publish_tf_ = true;
    ROS_INFO ("\tPublish transforms: %d", publish_tf_);

    if (!n_param.getParam("publish_visual_markers", publish_visual_markers_))
      publish_visual_markers_ = true;
    ROS_INFO ("\tPublish visual markers: %d", publish_visual_markers_);

    if (!n_param.getParam("threshold", threshold_))
      threshold_ = 100;
    ROS_INFO ("\tThreshold: %d", threshold_);

    if (!n_param.getParam("marker_width", marker_width_))
      marker_width_ = 80.0;
    ROS_INFO ("\tMarker Width: %.1f", marker_width_);

    if (!n_param.getParam("reverse_transform", reverse_transform_))
      reverse_transform_ = false;
    ROS_INFO("\tReverse Transform: %d", reverse_transform_);

    if (!n_param.getParam("marker_frame", marker_frame_))
      marker_frame_ = "ar_marker";
    ROS_INFO ("\tMarker frame: %s", marker_frame_.c_str());
    
    if (!n_param.getParam("camera_frame", camera_frame_))
      camera_frame_ = "camera";
    ROS_INFO ("\tCamera frame: %s", camera_frame_.c_str());

    // If mode=0, we use arGetTransMat instead of arGetTransMatCont
    // The arGetTransMatCont function uses information from the previous image
    // frame to reduce the jittering of the marker
    if (!n_param.getParam("use_history", use_history_))
      use_history_ = true;
    ROS_INFO("\tUse history: %d", use_history_);

  	//allow patterns to be loaded from outside the package
  	n_param.param ("marker_pattern", local_path, default_path);
  	if (local_path.compare(0,5,"data/") == 0)
    {
  	  //to keep working on previous implementations, check if first 5 chars equal "data/"
  	  sprintf (pattern_filename_, "%s/%s", package_path.c_str (), local_path.c_str ());
  	}
  	else
    {
  	  //for new implementations, can pass a path outside the package_path
  	  sprintf (pattern_filename_, "%s", local_path.c_str ());
  	}

    n_param.param ("marker_center_x", marker_center_[0], 0.0);
    n_param.param ("marker_center_y", marker_center_[1], 0.0);
    ROS_INFO ("\tMarker Center: (%.1f,%.1f)", marker_center_[0], marker_center_[1]);

    // **** subscribe
    bool use_camera_info;
    n_param.param ("use_camera_info", use_camera_info, false);
    if (!use_camera_info)
    {
      std::string camera_calib_file;
      n_param.param<std::string>("camera_calib_file", camera_calib_file, "");
      if (!loadCameraParameters(camera_calib_file))
      {
        ROS_ERROR("Not all camera paramters provided.");
        get_camera_info_ = false;
        ros::shutdown();
      }
        
      arInit();
      get_camera_info_ = true;
    }
    else
    {
      ROS_INFO ("Subscribing to info topic");
      sub_camera_info_ = n_.subscribe ("camera_info", 1, &ARSinglePublisher::camInfoCallback, this);
      get_camera_info_ = false;
    }

    ROS_INFO ("Subscribing to image topic");
    sub_image_ = it_.subscribe ("image_raw", 1, &ARSinglePublisher::getTransformationCallback, this);

    // **** advertsie
    pub_tag_pose_    = n_.advertise<geometry_msgs::PoseStamped>("tag_pose", 0);
    pub_camera_pose_ = n_.advertise<geometry_msgs::PoseStamped>("camera_pose", 0);
    pub_ar_marker    = n_.advertise<ar_pose::ARMarker>("ar_pose_marker", 0);
    if(publish_visual_markers_)
      pub_rviz_marker = n_.advertise<visualization_msgs::Marker>("visualization_marker", 0);
  }

  ARSinglePublisher::~ARSinglePublisher (void)
  {
    //cvReleaseImage(&capture); //Don't know why but crash when release the image
    arVideoCapStop ();
    arVideoClose ();
  }
    
  bool ARSinglePublisher::loadCameraParameters(const std::string& camera_calib_file)
  {
    const std::shared_ptr<vk::cameras::NCamera> cams = vk::cameras::NCamera::loadFromYaml(camera_calib_file);
    std::shared_ptr<vk::cameras::CameraGeometryBase> vk_cam = cams->getCameraShared(0);
        
    camera_matrix_.setTo(0);
    camera_matrix_.at<float>(0, 0) = vk_cam->getIntrinsicParameters()[vk::cameras::PinholeProjection< vk::cameras::RadialTangentialDistortion > ::IntrinsicParameters::kFocalLengthX];
    camera_matrix_.at<float>(1, 1) = vk_cam->getIntrinsicParameters()[vk::cameras::PinholeProjection< vk::cameras::RadialTangentialDistortion > ::IntrinsicParameters::kFocalLengthY];
    camera_matrix_.at<float>(0, 2) = vk_cam->getIntrinsicParameters()[vk::cameras::PinholeProjection< vk::cameras::RadialTangentialDistortion > ::IntrinsicParameters::kPricipalPointX];
    camera_matrix_.at<float>(1, 2) = vk_cam->getIntrinsicParameters()[vk::cameras::PinholeProjection< vk::cameras::RadialTangentialDistortion > ::IntrinsicParameters::kPrincipalPointY];
    camera_matrix_.at<float>(2, 2) = 1.;
        
    dist_coeffs_.at<float>(0, 0) = vk_cam->getDistortionParameters()[vk::cameras::RadialTangentialDistortion::DistortionParameters::kRadialDistortionFactor1];
    dist_coeffs_.at<float>(0, 1) = vk_cam->getDistortionParameters()[vk::cameras::RadialTangentialDistortion::DistortionParameters::kRadialDistortionFactor2];
    dist_coeffs_.at<float>(0, 2) = vk_cam->getDistortionParameters()[vk::cameras::RadialTangentialDistortion::DistortionParameters::kTangentialDistortionFactor1];
    dist_coeffs_.at<float>(0, 3) = vk_cam->getDistortionParameters()[vk::cameras::RadialTangentialDistortion::DistortionParameters::kTangentialDistortionFactor2];
        
    cv::Size size(vk_cam->imageHeight(), vk_cam->imageWidth());
      
    cam_param_.xsize = size.width();
    cam_param_.ysize = size.height();
    cam_param_.mat[0][0] = camera_matrix_.at<float>(0, 0);
    cam_param_.mat[0][1] = 0.0;
    cam_param_.mat[0][2] = camera_matrix_.at<float>(0, 2);
    cam_param_.mat[0][3] = 0.0;
    cam_param_.mat[1][0] = 0.0;
    cam_param_.mat[1][1] = camera_matrix_.at<float>(1, 1);
    cam_param_.mat[1][2] = camera_matrix_.at<float>(1, 2);
    cam_param_.mat[1][3] = 0.0;
    cam_param_.mat[2][0] = 0.0;
    cam_param_.mat[2][1] = 0.0;
    cam_param_.mat[2][2] = 1.0;
    cam_param_.mat[2][3] = 0.0;
    cam_param_.dist_factor[0] = dist_coeffs_.at<float>(0, 0);
    cam_param_.dist_factor[1] = dist_coeffs_.at<float>(0, 1);
    cam_param_.dist_factor[2] = dist_coeffs_.at<float>(0, 2);
    cam_param_.dist_factor[3] = dist_coeffs_.at<float>(0, 3);

    Eigen::Matrix<double, 4, 4> T_C_B_ = cams->getTransformationVector().at(0).getTransformationMatrix();
    tf::Matrix3x3 rotation(T_C_B_(0,0),T_C_B_(0,1),T_C_B_(0,2), T_C_B_(1,0),T_C_B_(1,1),T_C_B_(1,2), T_C_B_(2,0),T_C_B_(2,1),T_C_B_(2,2));
    tf::Vector3 translation(T_C_B_(0,3),T_C_B_(1,3),T_C_B_(2,3));
        
    tf::Matrix3x3 real_rotation = rotation.transpose();
    tf::Vector3 real_translation = real_rotation*(-translation);
    tf::Quaternion q;
    real_rotation.getRotation(q);
    transform_quad_camera_.setOrigin(real_translation);
    transform_quad_camera_.setRotation(q);
      
    return true;
  }

  void ARSinglePublisher::camInfoCallback (const sensor_msgs::CameraInfoConstPtr & cam_info)
  {
    if (!get_camera_info_)
    {
      cam_info_ = (*cam_info);

      cam_param_.xsize = cam_info_.width;
      cam_param_.ysize = cam_info_.height;

      cam_param_.mat[0][0] = cam_info_.P[0];
      cam_param_.mat[1][0] = cam_info_.P[4];
      cam_param_.mat[2][0] = cam_info_.P[8];
      cam_param_.mat[0][1] = cam_info_.P[1];
      cam_param_.mat[1][1] = cam_info_.P[5];
      cam_param_.mat[2][1] = cam_info_.P[9];
      cam_param_.mat[0][2] = cam_info_.P[2];
      cam_param_.mat[1][2] = cam_info_.P[6];
      cam_param_.mat[2][2] = cam_info_.P[10];
      cam_param_.mat[0][3] = cam_info_.P[3];
      cam_param_.mat[1][3] = cam_info_.P[7];
      cam_param_.mat[2][3] = cam_info_.P[11];

      cam_param_.dist_factor[0] = cam_info_.K[2];       // x0 = cX from openCV calibration
      cam_param_.dist_factor[1] = cam_info_.K[5];       // y0 = cY from openCV calibration
      if ( cam_info_.distortion_model == "plumb_bob" && cam_info_.D.size() == 5)
        cam_param_.dist_factor[2] = -100*cam_info_.D[0];// f = -100*k1 from CV. Note, we had to do mm^2 to m^2, hence 10^8->10^2
      else
        cam_param_.dist_factor[2] = 0;                  // We don't know the right value, so ignore it
      cam_param_.dist_factor[3] = 1.0;                  // scale factor, should probably be >1, but who cares...

      arInit();
      get_camera_info_ = true;
    }
  }

  void ARSinglePublisher::arInit ()
  {
    arInitCparam (&cam_param_);

    ROS_INFO ("*** Camera Parameter ***");
    arParamDisp (&cam_param_);

    // load pattern file
    ROS_INFO ("Loading pattern");
    patt_id_ = arLoadPatt (pattern_filename_);
    if (patt_id_ < 0)
    {
      ROS_ERROR ("Pattern file load error: %s", pattern_filename_);
      ROS_BREAK ();
    }

    sz_ = cvSize (cam_param_.xsize, cam_param_.ysize);
    // FIXME: Why is this not in the object
    cv_bridge::CvImagePtr capture_;
  }

  void ARSinglePublisher::getTransformationCallback (const sensor_msgs::ImageConstPtr & image_msg)
  {
    ARUint8 *dataPtr;
    ARMarkerInfo *marker_info;
    int marker_num;
    int i, k;

    /* Get the image from ROSTOPIC
     * NOTE: the dataPtr format is BGR because the ARToolKit library was
     * build with V4L, dataPtr format change according to the
     * ARToolKit configure option (see config.h).*/
    try
    {
      capture_ = cv_bridge::toCvCopy (image_msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }

    //TODO undistort only interesting point???
    cv::Mat img2;
    cv::undistort(capture_->image, img2, camera_matrix_, dist_coeffs_);
    dataPtr = (ARUint8 *) ((IplImage) img2).imageData;

    // detect the markers in the video frame
    if(arDetectMarker(dataPtr, threshold_, &marker_info, &marker_num) < 0)
    {
      ROS_FATAL("arDetectMarker failed");
      ROS_BREAK();
    }

    // check for known patterns
    k = -1;
    for (i = 0; i < marker_num; i++)
    {
      if (marker_info[i].id == patt_id_)
      {
        ROS_DEBUG ("Found pattern: %d ", patt_id_);

        // make sure you have the best pattern (highest confidence factor)
        if (k == -1)
          k = i;
        else if (marker_info[k].cf < marker_info[i].cf)
          k = i;
      }
    }

    if(k != -1)
    {
      // **** get the transformation between the marker and the real camera
      double ar_quat[4], ar_pos[3];

      if (!use_history_ || !marker_prev_visualized_)
        arGetTransMat(&marker_info[k], marker_center_, marker_width_, marker_trans_);
      else
        arGetTransMatCont(&marker_info[k], marker_trans_, marker_center_, marker_width_, marker_trans_);

      marker_prev_visualized_ = true;

      //arUtilMatInv (marker_trans_, cam_trans);
      arUtilMat2QuatPos (marker_trans_, ar_quat, ar_pos);

      // **** convert to ROS frame
      double quat[4], pos[3];

      pos[0] = ar_pos[0] * AR_TO_ROS;
      pos[1] = ar_pos[1] * AR_TO_ROS;
      pos[2] = ar_pos[2] * AR_TO_ROS;

      quat[0] = -ar_quat[0];
      quat[1] = -ar_quat[1];
      quat[2] = -ar_quat[2];
      quat[3] = ar_quat[3];

      ROS_DEBUG (" POSE: Pos x: %3.5f  y: %3.5f  z: %3.5f", pos[0], pos[1], pos[2]);
      ROS_DEBUG ("       Quat qx: %3.5f qy: %3.5f qz: %3.5f qw: %3.5f", quat[0], quat[1], quat[2], quat[3]);
        
      tf::Quaternion rotation (quat[0], quat[1], quat[2], quat[3]);
      tf::Vector3 origin (pos[0], pos[1], pos[2]);
      tf::Transform t (rotation, origin);

      // **** publish the marker
      ar_pose_marker_.header.frame_id = image_msg->header.frame_id;
      ar_pose_marker_.header.stamp    = image_msg->header.stamp;
      ar_pose_marker_.id              = marker_info->id;
      ar_pose_marker_.pose.pose.position.x = pos[0];
      ar_pose_marker_.pose.pose.position.y = pos[1];
      ar_pose_marker_.pose.pose.position.z = pos[2];
      ar_pose_marker_.pose.pose.orientation.x = quat[0];
      ar_pose_marker_.pose.pose.orientation.y = quat[1];
      ar_pose_marker_.pose.pose.orientation.z = quat[2];
      ar_pose_marker_.pose.pose.orientation.w = quat[3];
      ar_pose_marker_.confidence = marker_info->cf;
      pub_ar_marker.publish(ar_pose_marker_);
      ROS_DEBUG ("Published ar_single marker");

      geometry_msgs::PoseStamped tag_pose_;
      tag_pose_.header.frame_id = marker_frame_.c_str();
      tag_pose_.header.stamp    = image_msg->header.stamp;
      tag_pose_.pose.position.x = pos[0];
      tag_pose_.pose.position.y = pos[1];
      tag_pose_.pose.position.z = pos[2];
      tag_pose_.pose.orientation.x = quat[0];
      tag_pose_.pose.orientation.y = quat[1];
      tag_pose_.pose.orientation.z = quat[2];
      tag_pose_.pose.orientation.w = quat[3];
      pub_tag_pose_.publish(tag_pose_);
      ROS_DEBUG ("Published ar_pose tag");
        
      if(reverse_transform_)
      {
        tf::Transform tInv = t.inverse();
        geometry_msgs::PoseStamped camera_pose_;
        camera_pose_.header.frame_id = camera_frame_.c_str();
        camera_pose_.header.stamp    = image_msg->header.stamp;
        camera_pose_.pose.position.x = tInv.getOrigin().x();
        camera_pose_.pose.position.y = tInv.getOrigin().y();
        camera_pose_.pose.position.z = tInv.getOrigin().z();
        camera_pose_.pose.orientation.x = tInv.getRotation().x();
        camera_pose_.pose.orientation.y = tInv.getRotation().y();
        camera_pose_.pose.orientation.z = tInv.getRotation().z();
        camera_pose_.pose.orientation.w = tInv.getRotation().w();
        pub_camera_pose_.publish(camera_pose_);
        ROS_DEBUG ("Published ar_pose camera");
      }

      // **** publish transform between camera and marker
      if(publish_tf_)
      {
        if(reverse_transform_)
        {
          tf::Transform tInv = t.inverse();
          tf::StampedTransform markerToCam (tInv, image_msg->header.stamp, marker_frame_.c_str(), camera_frame_.c_str());
          broadcaster_.sendTransform(markerToCam);
        }
        else
        {
          tf::StampedTransform camToMarker (t, image_msg->header.stamp, camera_frame_.c_str(), marker_frame_.c_str());
          broadcaster_.sendTransform(camToMarker);
        }
      }

      // **** publish visual marker
      if(publish_visual_markers_)
      {
        tf::Vector3 markerOrigin (0, 0, 0.25 * marker_width_ * AR_TO_ROS);
        tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin);
        tf::Transform markerPose = t * m; // marker pose in the camera frame

        tf::poseTFToMsg(markerPose, rvizMarker_.pose);

        rvizMarker_.header.frame_id = image_msg->header.frame_id;
        rvizMarker_.header.stamp = image_msg->header.stamp;
        rvizMarker_.id = 1;

        rvizMarker_.scale.x = 1.0 * marker_width_ * AR_TO_ROS;
        rvizMarker_.scale.y = 1.0 * marker_width_ * AR_TO_ROS;
        rvizMarker_.scale.z = 0.5 * marker_width_ * AR_TO_ROS;
        rvizMarker_.ns = "basic_shapes";
        rvizMarker_.type = visualization_msgs::Marker::CUBE;
        rvizMarker_.action = visualization_msgs::Marker::ADD;
        rvizMarker_.color.r = 0.0f;
        rvizMarker_.color.g = 1.0f;
        rvizMarker_.color.b = 0.0f;
        rvizMarker_.color.a = 1.0;
        rvizMarker_.lifetime = ros::Duration(1.0);

        pub_rviz_marker.publish(rvizMarker_);
        ROS_DEBUG ("Published visual marker");
      }
    }
    else
    {
      geometry_msgs::PoseStamped tag_pose_;
      tag_pose_.header.frame_id = marker_frame_.c_str();
      tag_pose_.header.stamp    = image_msg->header.stamp;
      tag_pose_.pose.position.x = 0;
      tag_pose_.pose.position.y = 0;
      tag_pose_.pose.position.z = 0;
      tag_pose_.pose.orientation.x = 0;
      tag_pose_.pose.orientation.y = 0;
      tag_pose_.pose.orientation.z = 0;
      tag_pose_.pose.orientation.w = 0;
      pub_tag_pose_.publish(tag_pose_);
      
      if(reverse_transform_)
        pub_camera_pose_.publish(tag_pose_);

      marker_prev_visualized_ = false;
      ROS_DEBUG ("Failed to locate marker");
    }
  }
}                               // end namespace ar_pose

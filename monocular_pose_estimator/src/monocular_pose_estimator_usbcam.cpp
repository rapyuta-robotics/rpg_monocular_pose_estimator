// This file is part of RPG-MPE - the RPG Monocular Pose Estimator
//
// RPG-MPE is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// RPG-MPE is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with RPG-MPE.  If not, see <http://www.gnu.org/licenses/>.

/*
 * monocular_pose_estimator.cpp
 *
 * Created on: Jul 29, 2013
 * Author: Karl Schwabe
 *         Wolf Vollprecht <wolf.vollprecht@rapyuta-robotics.com>
 */

/** \file monocular_pose_estimator_node.cpp
 * \brief File containing the main function of the package
 *
 * This file is responsible for the flow of the program.
 *
 */

#include "monocular_pose_estimator/monocular_pose_estimator_usbcam.h"

namespace monocular_pose_estimator {

/**
 * Constructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::MPENode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), have_camera_info_(false) {
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.
  
  dynamic_reconfigure::Server<
      monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_;
  cb_ = boost::bind(&MPENode::dynamicParametersCallback, this, _1, _2);
  dr_server_.setCallback(cb_);

  // Initialize subscribers
  std::string video_device_id("/dev/video0");
  nh_private_.getParam("video_device_id", video_device_id);

  nh_private_.param("camera_brightness", brightness_, -1); // 0-255, -1 "leave alone"
  nh_private_.param("camera_contrast", contrast_, -1); // 0-255, -1 "leave alone"
  nh_private_.param("camera_saturation", saturation_, -1); // 0-255, -1 "leave alone"
  nh_private_.param("camera_sharpness", sharpness_, -1); // 0-255, -1 "leave alone"
  
  // enable/disable autofocus
  nh_private_.param("camera_autofocus", autofocus_, false);
  nh_private_.param("camera_focus", focus_, -1); // 0-255, -1 "leave alone"
  
  // enable/disable autoexposure
  nh_private_.param("camera_autoexposure", autoexposure_, false);
  nh_private_.param("camera_exposure", exposure_, 100);
  nh_private_.param("camera_gain", gain_, -1); // 0-100?, -1 "leave alone"

  video_capture_.start(video_device_id.c_str(), 640, 480, 30);

  setCameraParameters();

  video_capture_.start_capturing();

  // Initialize pose publisher
  pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>(
      "estimated_pose", 1);

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);

  // Create the marker positions from the test points
  List4DPoints positions_of_markers_on_object;

  // Read in the marker positions from the YAML parameter file
  XmlRpc::XmlRpcValue points_list;
  if (!nh_private_.getParam("marker_positions", points_list)) {
    ROS_ERROR(
        "%s: No reference file containing the marker positions, or the file is "
        "improperly formatted. Use the 'marker_positions_file' parameter in "
        "the launch file.",
        ros::this_node::getName().c_str());
    ros::shutdown();
  } else {
    positions_of_markers_on_object.resize(points_list.size());
    for (int i = 0; i < points_list.size(); i++) {
      Eigen::Matrix<double, 4, 1> temp_point;
      temp_point(0) = points_list[i]["x"];
      temp_point(1) = points_list[i]["y"];
      temp_point(2) = points_list[i]["z"];
      temp_point(3) = 1;
      positions_of_markers_on_object(i) = temp_point;
    }
  }

  trackable_object_.setMarkerPositions(positions_of_markers_on_object);
  ROS_INFO("The number of markers on the object are: %d",
           (int)positions_of_markers_on_object.size());

  std::string camera_info_url;
  if (!nh_private_.getParam("camera_info_url", camera_info_url)) {
    ROS_ERROR("No camera info url set!");
    ros::shutdown();
  } else {
    std::string camera_name;
    camera_calibration_parsers::readCalibration(camera_info_url, camera_name,
                                                cam_info_);
    storeCamInfo();
    ROS_INFO("Camera info loaded for %s", camera_name.c_str());
  }

  int param_setter_tick = 250;
  ros::Rate rate(30);
  while (ros::ok()) {
    run();
    ros::spinOnce();
    rate.sleep();
    if(param_setter_tick == 300) {
      this->setCameraParameters();
      param_setter_tick = 0;
    }
    param_setter_tick++;
  }
}

/**
 * Destructor of the Monocular Pose Estimation Node class
 *
 */
MPENode::~MPENode() {}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void MPENode::storeCamInfo() {
  // Calibrated camera
  trackable_object_.camera_matrix_K_ = cv::Mat(3, 3, CV_64F);
  trackable_object_.camera_matrix_K_.at<double>(0, 0) = cam_info_.K[0];
  trackable_object_.camera_matrix_K_.at<double>(0, 1) = cam_info_.K[1];
  trackable_object_.camera_matrix_K_.at<double>(0, 2) = cam_info_.K[2];
  trackable_object_.camera_matrix_K_.at<double>(1, 0) = cam_info_.K[3];
  trackable_object_.camera_matrix_K_.at<double>(1, 1) = cam_info_.K[4];
  trackable_object_.camera_matrix_K_.at<double>(1, 2) = cam_info_.K[5];
  trackable_object_.camera_matrix_K_.at<double>(2, 0) = cam_info_.K[6];
  trackable_object_.camera_matrix_K_.at<double>(2, 1) = cam_info_.K[7];
  trackable_object_.camera_matrix_K_.at<double>(2, 2) = cam_info_.K[8];
  trackable_object_.camera_distortion_coeffs_ = cam_info_.D;

  have_camera_info_ = true;
  ROS_INFO("Camera calibration information obtained.");
}

/**
 * The callback function that is executed every time an image is received. It
 *runs the main logic of the program.
 *
 * \param image_msg the ROS message containing the image to be processed
 */
void MPENode::run() {
  // Check whether already received the camera calibration data
  if (!have_camera_info_) {
    ROS_WARN("No camera info yet...");
    return;
  }

  // Get time at which the image was taken. This time is used to stamp the
  // estimated pose and also calculate the position of where to search for the
  // makers in the image
  ros::Time time_of_frame = ros::Time::now();

  // Mat(int _rows, int _cols, int _type, void* _data, size_t _step=AUTO_STEP);
  cv::Mat image =
      cv::Mat(480, 640, CV_8U, (void*) video_capture_.get_image_pointer());

  const bool found_body_pose =
      trackable_object_.estimateBodyPose(image, time_of_frame.toSec());

  if (found_body_pose)  // Only output the pose, if the pose was updated (i.e. a
                        // valid pose was found).
  {
    // Eigen::Matrix4d transform = trackable_object.getPredictedPose();
    Matrix6d cov = trackable_object_.getPoseCovariance();
    Eigen::Matrix4d transform = trackable_object_.getPredictedPose();

    ROS_DEBUG_STREAM("The transform: \n" << transform);
    ROS_DEBUG_STREAM("The covariance: \n" << cov);

    // Convert transform to PoseWithCovarianceStamped message
    // This position is converted to NED
    predicted_pose_.header.stamp = time_of_frame;
    predicted_pose_.pose.pose.position.y = transform(0, 3);
    predicted_pose_.pose.pose.position.x = transform(1, 3);
    predicted_pose_.pose.pose.position.z = -transform(2, 3);
    auto ned_quat = Eigen::Quaterniond(0.0, std::sqrt(2) / 2.0, std::sqrt(2) / 2.0, 0.0);
    Eigen::Quaterniond orientation = ned_quat *
        Eigen::Quaterniond(transform.block<3, 3>(0, 0));
    predicted_pose_.pose.pose.orientation.x = orientation.x();
    predicted_pose_.pose.pose.orientation.y = orientation.y();
    predicted_pose_.pose.pose.orientation.z = orientation.z();
    predicted_pose_.pose.pose.orientation.w = orientation.w();

    // Add covariance to PoseWithCovarianceStamped message
    for (unsigned i = 0; i < 6; ++i) {
      for (unsigned j = 0; j < 6; ++j) {
        predicted_pose_.pose.covariance.elems[j + 6 * i] = cov(i, j);
      }
    }

    // Publish the pose
    pose_pub_.publish(predicted_pose_);
  } else {  // If pose was not updated
    ROS_WARN("Unable to resolve a pose.");
  }

  // publish visualization image
  if (image_pub_.getNumSubscribers() > 0) {
    cv::Mat visualized_image = image.clone();
    cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);
    trackable_object_.augmentImage(visualized_image, found_body_pose);

    // Publish image for visualization
    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = visualized_image;

    image_pub_.publish(visualized_image_msg.toImageMsg());
  }
}

void MPENode::setCameraParameters() {
  // set camera parameters
  if (brightness_ >= 0)
    video_capture_.set_v4l_parameter("brightness", brightness_);
  if (contrast_ >= 0)
    video_capture_.set_v4l_parameter("contrast", contrast_);
  if (saturation_ >= 0)
    video_capture_.set_v4l_parameter("saturation", saturation_);
  if (sharpness_ >= 0)
    video_capture_.set_v4l_parameter("sharpness", sharpness_);
  if (gain_ >= 0)
    video_capture_.set_v4l_parameter("gain", gain_);

  // check auto white balance
  // if (auto_white_balance_) {
  //   video_capture_.set_v4l_parameter("white_balance_temperature_auto", 1);
  // } else {
  //   video_capture_.set_v4l_parameter("white_balance_temperature_auto", 0);
  //   video_capture_.set_v4l_parameter("white_balance_temperature", white_balance_);
  // }

  // check auto exposure
  if (!autoexposure_) {
    // turn down exposure control (from max of 3)
    video_capture_.set_v4l_parameter("exposure_auto", 1);
    video_capture_.set_v4l_parameter("exposure_absolute", exposure_);
  }

  // check auto focus
  if (autofocus_) {
    video_capture_.set_auto_focus(1);
    video_capture_.set_v4l_parameter("focus_auto", 1);
  } else {
    video_capture_.set_v4l_parameter("focus_auto", 0);
    if (focus_ >= 0)
      video_capture_.set_v4l_parameter("focus_absolute", focus_);
  }
}

/**
 * The dynamic reconfigure callback function. This function updates the variable
 * within the program whenever they are changed using dynamic reconfigure.
 */
void MPENode::dynamicParametersCallback(
    monocular_pose_estimator::MonocularPoseEstimatorConfig& config,
    uint32_t level) {
  ROS_WARN("CHANGING PARAMETERS!");
  trackable_object_.detection_threshold_value_ = config.threshold_value;
  trackable_object_.blur_size_ = config.blur_size;
  trackable_object_.min_blob_area_ = config.min_blob_area;
  trackable_object_.max_blob_area_ = config.max_blob_area;
  trackable_object_.max_width_height_distortion_ =
      config.max_width_height_distortion;
  trackable_object_.max_circular_distortion_ = config.max_circular_distortion;
  trackable_object_.roi_border_thickness_ = config.roi_border_thickness;

  trackable_object_.setBackProjectionPixelTolerance(
      config.back_projection_pixel_tolerance);
  trackable_object_.setNearestNeighbourPixelTolerance(
      config.nearest_neighbour_pixel_tolerance);
  trackable_object_.setCertaintyThreshold(config.certainty_threshold);
  trackable_object_.setValidCorrespondenceThreshold(
      config.valid_correspondence_threshold);

  brightness_ = config.camera_brightness;
  contrast_ = config.camera_contrast;
  saturation_ = config.camera_saturation;
  sharpness_ = config.camera_sharpness;
  focus_ = config.camera_focus;
  exposure_ = config.camera_exposure;
  gain_ = config.camera_gain;

  autofocus_ = config.camera_autofocus;
  autoexposure_ = config.camera_autoexposure;

  setCameraParameters();

  ROS_INFO("Parameters changed");
}

}  // namespace monocular_pose_estimator

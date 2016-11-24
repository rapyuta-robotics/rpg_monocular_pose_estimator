#ifndef POINT_DETECTION_NODE_H_
#define POINT_DETECTION_NODE_H_

#include "ros/ros.h"

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <camera_calibration_parsers/parse.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv/cvwimage.h>

#include <dynamic_reconfigure/server.h>
#include <monocular_pose_estimator/MonocularPoseEstimatorConfig.h>

#include <mpe_usb_cam/usb_cam.h>

#include "monocular_pose_estimator_lib/led_detector.h"
#include "monocular_pose_estimator_lib/pose_estimator.h"

namespace monocular_pose_estimator
{

class PointDetectionNode
{
private:

  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  image_transport::Publisher image_pub_; //!< The ROS image publisher that publishes the visualisation image

  usb_cam::UsbCam video_capture_; //!< OpenCV video capture device

  dynamic_reconfigure::Server<monocular_pose_estimator::MonocularPoseEstimatorConfig> dr_server_; //!< The dynamic reconfigure server

  bool have_camera_info_; //!< The boolean variable that indicates whether the camera calibration parameters have been obtained from the camera
  sensor_msgs::CameraInfo cam_info_; //!< Variable to store the camera calibration parameters

  PoseEstimator trackable_object_; //!< Declaration of the object whose pose will be estimated

  // camera variables
  int brightness_,
      contrast_,
      saturation_,
      sharpness_,
      focus_,
      exposure_,
      gain_;

  bool autofocus_,
       autoexposure_;

  void setCameraParameters();

  cv::Rect region_of_interest_; //!< OpenCV rectangle that defines the region of interest to be processd to find the LEDs in the image

  // convenience function for making all of the relevent LED detection calls
  void detectLEDs(cv::Mat image);

public:

  PointDetectionNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private);
  PointDetectionNode() : PointDetectionNode( ros::NodeHandle(), ros::NodeHandle("~") ){}
  ~PointDetectionNode();

  void run();
  void storeCamInfo();
  void dynamicParametersCallback(monocular_pose_estimator::MonocularPoseEstimatorConfig &config, uint32_t level);
};

} // monocular_pose_estimator namespace

#endif /* POINT_DETECTION_NODE_H_ */

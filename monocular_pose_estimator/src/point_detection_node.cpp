#include "monocular_pose_estimator/point_detection_node.hpp"

namespace monocular_pose_estimator {

/**
 * Constructor of the Point Detection Node class
 *
 */
PointDetectionNode::PointDetectionNode(const ros::NodeHandle& nh, const ros::NodeHandle& nh_private)
    : nh_(nh), nh_private_(nh_private), have_camera_info_(false) {
  // Set up a dynamic reconfigure server.
  // This should be done before reading parameter server values.

  dynamic_reconfigure::Server<
      monocular_pose_estimator::MonocularPoseEstimatorConfig>::CallbackType cb_;
  cb_ = boost::bind(&PointDetectionNode::dynamicParametersCallback, this, _1, _2);
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

  // Initialize image publisher for visualization
  image_transport::ImageTransport image_transport(nh_);
  image_pub_ = image_transport.advertise("image_with_detections", 1);

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
 * Destructor of the Point Detection Node class
 *
 */
PointDetectionNode::~PointDetectionNode() {}

/**
 * The callback function that retrieves the camera calibration information
 *
 * \param msg the ROS message containing the camera calibration information
 *
 */
void PointDetectionNode::storeCamInfo() {
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
void PointDetectionNode::run() {
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
      cv::Mat(480, 640, CV_8U, (void*) video_capture_.grab_image_pointer());

  // publish visualization image
  if (image_pub_.getNumSubscribers() > 0) {
    cv::Mat visualized_image = image.clone();
    cv::cvtColor(visualized_image, visualized_image, CV_GRAY2RGB);

    // Publish image for visualization
    cv_bridge::CvImage visualized_image_msg;
    visualized_image_msg.encoding = sensor_msgs::image_encodings::BGR8;
    visualized_image_msg.image = visualized_image;

    image_pub_.publish(visualized_image_msg.toImageMsg());
  }
}

void PointDetectionNode::setCameraParameters() {
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
void PointDetectionNode::dynamicParametersCallback(
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

int main(int argc, char* argv[])
{
  ros::init(argc, argv, "point_detection_node");

  monocular_pose_estimator::PointDetectionNode point_detection_node;

  ros::spin();

  return 0;
}

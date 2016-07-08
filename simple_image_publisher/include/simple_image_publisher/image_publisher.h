#ifndef SIMPLE_IMAGE_PUBLISHER__IMAGE_PUBLISHER_H_
#define SIMPLE_IMAGE_PUBLISHER__IMAGE_PUBLISHER_H_

#include <ros/ros.h>
#include <string>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <camera_info_manager/camera_info_manager.h>

namespace simple_image_publisher { 

class ImagePublisher {
public:
    ImagePublisher(ros::NodeHandle &nh, ros::NodeHandle &nh_private) : 
        it_(nh),
        info_manager_(nh, "camera"),
        pub_(it_.advertiseCamera("camera/image", 1))
    {
        nh_private.getParam("camera_info_url", info_url_);
        nh_private.getParam("image_file", image_file_);
        double update_rate;
        nh_private.param<double>("update_rate", update_rate, 5.0);
        rate_.reset(new ros::Rate(update_rate));
    }

    void load() {
        std::cout << info_url_ << std::endl;
        if(info_manager_.validateURL(info_url_)) {
            info_manager_.loadCameraInfo(info_url_);
            info_ = info_manager_.getCameraInfo();
            info_.header.frame_id = "world";
        } else {
            std::cout << "invalid camera info" << std::endl;
            return;
        }
        
        std::cout << "image_file = " << image_file_ << std::endl;
        cv::Mat image = cv::imread(image_file_, CV_LOAD_IMAGE_COLOR);
        std::cout << "image[height, width] = (" <<image.rows << ", " << image.cols << ")" << std::endl;
        cv::waitKey(30);
        image_msg_ = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();
    }

    void publish() {
        info_.header.stamp = ros::Time::now();
        pub_.publish(*image_msg_, info_);
        
        rate_->sleep();
    }

private:
    image_transport::ImageTransport it_;
    camera_info_manager::CameraInfoManager info_manager_;
    image_transport::CameraPublisher pub_;
    sensor_msgs::CameraInfo info_;
    boost::shared_ptr<ros::Rate> rate_;
    sensor_msgs::ImagePtr image_msg_;
    std::string info_url_, image_file_;

};

} //namespace simple_image_publisher

#endif //SIMPLE_IMAGE_PUBLISHER__IMAGE_PUBLISHER_H_

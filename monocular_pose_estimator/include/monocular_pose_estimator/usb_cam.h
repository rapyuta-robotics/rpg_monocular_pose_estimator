// This is a modified version of usb_cam, the classic ROS usb cam driver. 
// It is tuned to provide a black-and-white image from a YUYV capable USB cam.
// Modified by Wolf Vollprecht <wolf.vollprecht@rapyuta-robotics.com 

/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2014, Robert Bosch LLC.
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
 *   * Neither the name of the Robert Bosch nor the names of its
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
 *
 *********************************************************************/
#ifndef USB_CAM_USB_CAM_H
#define USB_CAM_USB_CAM_H

#include <asm/types.h> /* for videodev2.h */

extern "C" {
#include <linux/videodev2.h>
}

#include <string>
#include <sstream>

#include <sensor_msgs/Image.h>

namespace usb_cam {

class UsbCam {
 public:
  UsbCam();
  ~UsbCam();

  // start camera
  void start(const std::string& dev, int image_width, int image_height, int framerate);
  // shutdown camera
  void shutdown(void);

  // grabs a new image from the camera
  void grab_image(sensor_msgs::Image* image);

  // enables/disable auto focus
  void set_auto_focus(int value);

  // Set video device parameters
  void set_v4l_parameter(const std::string& param, int value);
  void set_v4l_parameter(const std::string& param, const std::string& value);

  void stop_capturing();
  void start_capturing();
  bool is_capturing();
  char* get_image_pointer();

 private:
  typedef struct {
    int width;
    int height;
    int bytes_per_pixel;
    int image_size;
    char* image;
    int is_new;
  } camera_image_t;

  struct buffer {
    void* start;
    size_t length;
  };

  void process_image(const void* src, int len, camera_image_t* dest);
  int read_frame();
  void uninit_device();
  void init_read(unsigned int buffer_size);
  void init_mmap();
  void init_userp(unsigned int buffer_size);
  void init_device(int image_width, int image_height, int framerate);
  void close_device();
  void open_device();
  void grab_image();
  bool is_capturing_;

  std::string camera_dev_;
  int fd_;
  buffer* buffers_;
  unsigned int n_buffers_;
  struct SwsContext* video_sws_;
  camera_image_t* image_;
};
}

#endif

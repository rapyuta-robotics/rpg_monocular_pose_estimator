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
#define __STDC_CONSTANT_MACROS
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <fcntl.h> /* low-level i/o */
#include <unistd.h>
#include <errno.h>
#include <malloc.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <sys/ioctl.h>
#include <ros/ros.h>
#include <boost/lexical_cast.hpp>
#include <sensor_msgs/fill_image.h>

#include <chrono>
#include <iostream>

using namespace std;

#include <monocular_pose_estimator/usb_cam.h>

#define CLEAR(x) memset(&(x), 0, sizeof(x))

namespace usb_cam {

static void errno_exit(const char* s) {
  ROS_ERROR("%s error %d, %s", s, errno, strerror(errno));
  exit(EXIT_FAILURE);
}

static int xioctl(int fd, int request, void* arg) {
  int r;

  do
    r = ioctl(fd, request, arg);
  while (-1 == r && EINTR == errno);

  return r;
}

static void yuyv2mono8(char* RAW, char* MONO, int NumPixels) {
  int i, j;
  for (i = 0, j = 0; i < (NumPixels << 1); i += 2, j += 1) {
    MONO[j] = RAW[i];
  }
}

UsbCam::UsbCam()
    : io_(IO_METHOD_MMAP),
      fd_(-1),
      buffers_(NULL),
      n_buffers_(0),
      video_sws_(NULL),
      image_(NULL),
      is_capturing_(false) {}

UsbCam::~UsbCam() { shutdown(); }

void UsbCam::process_image(const void* src, int len, camera_image_t* dest) {
  yuyv2mono8((char*)src, dest->image, dest->width * dest->height);
}

int UsbCam::read_frame() {
  struct v4l2_buffer buf;
  unsigned int i;
  int len;

  CLEAR(buf);

  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (-1 == xioctl(fd_, VIDIOC_DQBUF, &buf)) {
    switch (errno) {
      case EAGAIN:
        return 0;

      case EIO:
      /* Could ignore EIO, see spec. */
      /* fall through */

      default:
        errno_exit("VIDIOC_DQBUF");
    }
  }
  assert(buf.index < n_buffers_);
  len = buf.bytesused;

  // this is where most of the time is wasted
  process_image(buffers_[buf.index].start, len, image_);

  if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) errno_exit("VIDIOC_QBUF");

  return 1;
}

bool UsbCam::is_capturing() { return is_capturing_; }

void UsbCam::stop_capturing(void) {
  if (!is_capturing_) return;

  is_capturing_ = false;
  enum v4l2_buf_type type;

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(fd_, VIDIOC_STREAMOFF, &type))
    errno_exit("VIDIOC_STREAMOFF");
}

void UsbCam::start_capturing(void) {
  if (is_capturing_) return;

  unsigned int i;
  enum v4l2_buf_type type;

  for (i = 0; i < n_buffers_; ++i) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (-1 == xioctl(fd_, VIDIOC_QBUF, &buf)) 
      errno_exit("VIDIOC_QBUF");
  }

  type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  if (-1 == xioctl(fd_, VIDIOC_STREAMON, &type)) 
    errno_exit("VIDIOC_STREAMON");

  is_capturing_ = true;
}

void UsbCam::uninit_device(void) {
  unsigned int i;

  for (i = 0; i < n_buffers_; ++i)
    if (-1 == munmap(buffers_[i].start, buffers_[i].length))
      errno_exit("munmap");

  free(buffers_);
}

void UsbCam::init_mmap(void) {
  struct v4l2_requestbuffers req;

  CLEAR(req);

  req.count = 4;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (-1 == xioctl(fd_, VIDIOC_REQBUFS, &req)) {
    if (EINVAL == errno) {
      ROS_ERROR_STREAM(camera_dev_ << " does not support memory mapping");
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_REQBUFS");
    }
  }

  if (req.count < 2) {
    ROS_ERROR_STREAM("Insufficient buffer memory on " << camera_dev_);
    exit(EXIT_FAILURE);
  }

  buffers_ = (buffer*)calloc(req.count, sizeof(*buffers_));

  if (!buffers_) {
    ROS_ERROR("Out of memory");
    exit(EXIT_FAILURE);
  }

  for (n_buffers_ = 0; n_buffers_ < req.count; ++n_buffers_) {
    struct v4l2_buffer buf;

    CLEAR(buf);

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = n_buffers_;

    if (-1 == xioctl(fd_, VIDIOC_QUERYBUF, &buf)) errno_exit("VIDIOC_QUERYBUF");

    buffers_[n_buffers_].length = buf.length;
    buffers_[n_buffers_].start =
        mmap(NULL /* start anywhere */, buf.length,
             PROT_READ | PROT_WRITE /* required */,
             MAP_SHARED /* recommended */, fd_, buf.m.offset);

    if (MAP_FAILED == buffers_[n_buffers_].start) errno_exit("mmap");
  }
}

void UsbCam::init_device(int image_width, int image_height, int framerate) {
  struct v4l2_capability cap;
  unsigned int min;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCAP, &cap)) {
    if (EINVAL == errno) {
      ROS_ERROR_STREAM(camera_dev_ << " is no V4L2 device");
      exit(EXIT_FAILURE);
    } else {
      errno_exit("VIDIOC_QUERYCAP");
    }
  }

  if (!(cap.capabilities & V4L2_CAP_VIDEO_CAPTURE)) {
    ROS_ERROR_STREAM(camera_dev_ << " is no video capture device");
    exit(EXIT_FAILURE);
  }

  if (!(cap.capabilities & V4L2_CAP_STREAMING)) {
    ROS_ERROR_STREAM(camera_dev_ << " does not support streaming i/o");
    exit(EXIT_FAILURE);
  }

  /* Select video input, video standard and tune here. */
  struct v4l2_format fmt;
  memset(&fmt, 0, sizeof fmt);

  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = 640;
  fmt.fmt.pix.height = 480;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
  fmt.fmt.pix.field = V4L2_FIELD_ANY;

  int ret = ioctl(fd_, VIDIOC_S_FMT, &fmt);
  if (ret < 0) {
    printf("Unable to set format: %s (%d).\n", strerror(errno), errno);
  }
  printf("Video format set: width: %u height: %u buffer size: %u\n",
         fmt.fmt.pix.width, fmt.fmt.pix.height, fmt.fmt.pix.sizeimage);

  std::cout << "Ok, this worked ... " << std::endl;

  /* Note VIDIOC_S_FMT may change width and height. */

  image_width = fmt.fmt.pix.width;
  image_height = fmt.fmt.pix.height;

  struct v4l2_streamparm stream_params;
  memset(&stream_params, 0, sizeof(stream_params));
  stream_params.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (xioctl(fd_, VIDIOC_G_PARM, &stream_params) < 0)
    errno_exit("Couldn't query v4l fps!");

  ROS_DEBUG("Capability flag: 0x%x", stream_params.parm.capture.capability);

  stream_params.parm.capture.timeperframe.numerator = 1;
  stream_params.parm.capture.timeperframe.denominator = framerate;
  if (xioctl(fd_, VIDIOC_S_PARM, &stream_params) < 0)
    ROS_WARN("Couldn't set camera framerate");
  else
    ROS_DEBUG("Set framerate to be %i", framerate);

  init_mmap();
}

void UsbCam::close_device(void) {
  if (-1 == close(fd_)) errno_exit("close");

  fd_ = -1;
}

void UsbCam::open_device(void) {
  struct stat st;

  if (-1 == stat(camera_dev_.c_str(), &st)) {
    ROS_ERROR_STREAM("Cannot identify '" << camera_dev_ << "': " << errno
                                         << ", " << strerror(errno));
    exit(EXIT_FAILURE);
  }

  if (!S_ISCHR(st.st_mode)) {
    ROS_ERROR_STREAM(camera_dev_ << " is no device");
    exit(EXIT_FAILURE);
  }

  fd_ = open(camera_dev_.c_str(), O_RDWR | O_NONBLOCK, 0);

  if (-1 == fd_) {
    ROS_ERROR_STREAM("Cannot open '" << camera_dev_ << "': " << errno << ", "
                                     << strerror(errno));
    exit(EXIT_FAILURE);
  }
}

void UsbCam::start(const std::string& dev, int image_width, 
                   int image_height, int framerate) {
  camera_dev_ = dev;

  io_ = IO_METHOD_MMAP;

  // fill_tab(240);

  open_device();
  init_device(640, 480, 30);
  start_capturing();

  image_ = (camera_image_t*)calloc(1, sizeof(camera_image_t));

  image_->width = image_width;
  image_->height = image_height;
  image_->bytes_per_pixel = 1;

  image_->image_size = image_->width * image_->height * image_->bytes_per_pixel;
  image_->is_new = 0;
  image_->image = (char*)calloc(image_->image_size, sizeof(char));
  memset(image_->image, 0, image_->image_size * sizeof(char));
}

void UsbCam::shutdown(void) {
  stop_capturing();
  uninit_device();
  close_device();

  if (image_) free(image_);
  image_ = NULL;
}

void UsbCam::grab_image(sensor_msgs::Image* msg) {
  // grab the image
  grab_image();
  // stamp the image
  msg->header.stamp = ros::Time::now();
  // fill the info
  fillImage(*msg, "mono8", image_->height, image_->width, image_->width,
            image_->image);
}

char* UsbCam::get_image_pointer() {
  // grab the image
  grab_image();
  return image_->image;
}

void UsbCam::grab_image() {
  fd_set fds;
  struct timeval tv;
  int r;

  FD_ZERO(&fds);
  FD_SET(fd_, &fds);

  /* Timeout. */
  tv.tv_sec = 5;
  tv.tv_usec = 0;

  r = select(fd_ + 1, &fds, NULL, NULL, &tv);

  if (-1 == r) {
    if (EINTR == errno) return;
    errno_exit("select");
  }

  if (0 == r) {
    ROS_ERROR("select timeout");
    exit(EXIT_FAILURE);
  }

  read_frame();
  image_->is_new = 1;
}

// enables/disables auto focus
void UsbCam::set_auto_focus(int value) {
  struct v4l2_queryctrl queryctrl;
  struct v4l2_ext_control control;

  memset(&queryctrl, 0, sizeof(queryctrl));
  queryctrl.id = V4L2_CID_FOCUS_AUTO;

  if (-1 == xioctl(fd_, VIDIOC_QUERYCTRL, &queryctrl)) {
    if (errno != EINVAL) {
      perror("VIDIOC_QUERYCTRL");
      return;
    } else {
      ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
      return;
    }
  } else if (queryctrl.flags & V4L2_CTRL_FLAG_DISABLED) {
    ROS_INFO("V4L2_CID_FOCUS_AUTO is not supported");
    return;
  } else {
    memset(&control, 0, sizeof(control));
    control.id = V4L2_CID_FOCUS_AUTO;
    control.value = value;

    if (-1 == xioctl(fd_, VIDIOC_S_CTRL, &control)) {
      perror("VIDIOC_S_CTRL");
      return;
    }
  }
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param, int value) {
  set_v4l_parameter(param, boost::lexical_cast<std::string>(value));
}

/**
* Set video device parameter via call to v4l-utils.
*
* @param param The name of the parameter to set
* @param param The value to assign
*/
void UsbCam::set_v4l_parameter(const std::string& param,
                               const std::string& value) {
  // build the command
  std::stringstream ss;
  ss << "v4l2-ctl --device=" << camera_dev_ << " -c " << param << "=" << value
     << " 2>&1";
  std::string cmd = ss.str();

  // capture the output
  std::string output;
  int buffer_size = 256;
  char buffer[buffer_size];
  FILE* stream = popen(cmd.c_str(), "r");
  if (stream) {
    while (!feof(stream))
      if (fgets(buffer, buffer_size, stream) != NULL) output.append(buffer);
    pclose(stream);
    // any output should be an error
    if (output.length() > 0) ROS_WARN("%s", output.c_str());
  } else
    ROS_WARN("usb_cam_node could not run '%s'", cmd.c_str());
}

}

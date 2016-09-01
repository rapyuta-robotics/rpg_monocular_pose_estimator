# Changes from the original code

This document lists the changes that have been committed in rapyutas repository
in order to speed up the monocular pose estimation on a resource constrained 
computing device.

# IMPORTANT NOTES

- Make sure to have v4l-utils installed

## Speeding up LED detection

The original code used a Gaussian blur which is an "expensive" operation as many
pixels have to be read in order to compute the value for the target pixel. 
The Gaussian blur has been replaced with a much cheaper Box blur operation.
However, even the Box blur operation might not be necessary and one can think
about removing it.

Some checks for circularity and width-height distortion have been removed as 
they were deemed not necessary (for indoor use!). Adding them back in should be 
trivial.

## Camera driver inside the MPE node

The original code used ROS to communicate with the camera. The communication was 
optimized by using nodelets.
However, there were two conversion steps from the camera to the ROS message and 
to OpenCV (through CV bridge) which involve two copies from the original camera
buffer. Additionally, the color conversion from RGB to Grayscale was done in 
OpenCV. The color conversion from RGB to grayscale is not a cheap operation as
it involves all pixels.

In order to get full control from the camera image to the MPE node the usb_cam 
implementation was used and modified. usb_cam uses video4linux2 as a backend, which
is a standard linux driver. The usb_cam package was modified and most options 
were removed such that it always chooses YUYV as recording format. The grayscale
image is directly extracted from the Y-data involving no additional steps. After
extracting the grayscale the buffer is never copied again -- the pointer to the 
data is directly passed into a CV::Mat. This also means that the image get's
destroyed after the LED detection is applied, which sometimes shows up as artifacts
when viewing the debug image in `rqt_image_view`.

Another change from the standard usb_cam package is that the parameters have been
added to dynamic_reconfigure which makes it very easy to tune exposure settings 
for example.

## Further improvement directions

If a NVIDIA GPU becomes available to rapyuta, using it could lead to better per-
formance in the Matrix-computation heavy operations (like box-blur).
Also, there is a OpenCV Tegra package which can be installed, and should lead
to a speed up in some OpenCV operations (has not been explored yet).

http://docs.nvidia.com/gameworks/content/technologies/mobile/opencv_tegra_examples.htm
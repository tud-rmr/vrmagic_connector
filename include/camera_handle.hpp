#ifndef VRMAGIC_CAMERA_HANDLE_H
#define VRMAGIC_CAMERA_HANDLE_H

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "vrmusbcam2.h"

class VrMagicCameraHandle {
 public:
  VrMagicCameraHandle();
  ~VrMagicCameraHandle();

  void init();
  void grabFrame(VRmDWORD port, sensor_msgs::Image& img, const ros::Time& triggerTime);

 private:
  VRmUsbCamDevice device;
  VRmImageFormat targetFormat;

  VRmImage* sourceImg;
  VRmImage* targetImg;

  int width;
  int height;

  void initSensors();
  void setProperties();
  void setTargetFormat();
  void startCamera();
  void prepareGrabbing();
};

#endif

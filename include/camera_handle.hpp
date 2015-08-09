#ifndef VRMAGIC_CAMERA_HANDLE_H
#define VRMAGIC_CAMERA_HANDLE_H

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "vrmusbcam2.h"

namespace vrmagic {

const std::string frameId("VRMAGIC");

struct Config {
  VRmDWORD portLeft;
  VRmDWORD portRight;

  float gainLeft;
  float gainRight;

  Config() : portLeft(1), portRight(3), gainLeft(5.f), gainRight(5.f) {}
};

class CameraHandle {
 public:
  CameraHandle(VrmConfig conf);
  ~CameraHandle();

  void grabFrameLeft(sensor_msgs::Image& img, const ros::Time& triggerTime);
  void grabFrameRight(sensor_msgs::Image& img, const ros::Time& triggerTime);
  void grabFrame(VRmDWORD port, sensor_msgs::Image& img, const ros::Time& triggerTime);

 private:
  VRmUsbCamDevice device;
  VRmImageFormat sourceFormat;
  VRmImageFormat targetFormat;

  VrmConfig config;

  void initCamera();

  void initSensors();
  void setProperties();
  void startCamera();
};
}
#endif

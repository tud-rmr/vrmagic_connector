#ifndef VRMAGIC_CAMERA_HANDLE_H
#define VRMAGIC_CAMERA_HANDLE_H

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include "vrmusbcam2.h"

namespace vrmagic {

struct Config {
  /////////////
  // Globals //
  /////////////

  std::string frameId;

  // In ms. When locking the image, an error is thrown after the timeout
  // if the image has not been unlocked until then.
  int timeout;

  // Sensor configuration
  VRmDWORD portLeft;
  VRmDWORD portRight;

  bool setGain;
  int gainLeft;
  int gainRight;

  // Default values
  Config() : frameId("VRMAGIC"), timeout(5000), portLeft(1), portRight(3), setGain(false) {}
};

class CameraHandle {
 public:
  CameraHandle(Config conf);
  ~CameraHandle();

  void grabFrameLeft(sensor_msgs::Image& img, const ros::Time& triggerTime);
  void grabFrameRight(sensor_msgs::Image& img, const ros::Time& triggerTime);

 private:
  VRmUsbCamDevice device;
  VRmImageFormat sourceFormat;
  VRmImageFormat targetFormat;

  Config conf;

  void initCamera();
  void openDevice();
  void initSensors();
  void getSourceFormat();
  void setTargetFormat();

  void setProperties();
  void checkAndSanitizeConfig();
  void checkAndSanitizeProperty(int& value, VRmPropId property, std::string name);
  void checkAndSanitizeIntProperty(int& value, VRmPropAttribsI attr, std::string name);
  void setPropertyLeftAndRight(VRmPropId property, int valueLeft, int valueRight);
  void setSingleProperty(VRmDWORD port, VRmPropId property, int value);
  void setGain();
  void setTriggerMode();

  void startCamera();

  void grabFrame(VRmDWORD port, sensor_msgs::Image& img, const ros::Time& triggerTime);
};
}
#endif

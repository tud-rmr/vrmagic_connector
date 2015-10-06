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
  bool enableLogging;

  // In ms. When locking the image, an error is thrown after the timeout
  // if the image has not been unlocked until then.
  int timeout;

  //////////////////////////
  // Sensor configuration //
  //////////////////////////

  int portLeft;
  int portRight;

  // Default values
  Config() : frameId("VRMAGIC"), enableLogging(false), timeout(5000), portLeft(1), portRight(2) {}
};

void cameraShutdown();

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
  void getSourceFormat();
  void setTargetFormat();

  void startCamera();

  void grabFrame(VRmDWORD port, sensor_msgs::Image& img, const ros::Time& triggerTime);
};
}
#endif

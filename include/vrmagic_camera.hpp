#ifndef VRMAGIC_CAMERA_H
#define VRMAGIC_CAMERA_H

#include <string>

#include <boost/thread.hpp>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "vrmusbcam2.h"

const std::string frame_id = "VRMAGIC";
const unsigned int NUMBER_OF_SENSORS = 2;
const unsigned int width = 754;
const unsigned int height = 480;

class VRGrabException : public std::string {
 public:
  VRGrabException(const char *err) : std::string(err) {}
};

class VRControlException : public std::string {
 public:
  VRControlException(const char *err) : std::string(err) {}
};

class VrMagicCamera {
 public:
  explicit VrMagicCamera(const ros::NodeHandle &nh);

  ~VrMagicCamera();

  void initCamera();
  void broadcastFrame();
  void grabFrame(VRmDWORD port, sensor_msgs::Image &img, const ros::Time &triggerTime);
  void spin();

 private:
  VRmUsbCamDevice device;
  VRmImageFormat target_format;

  ros::NodeHandle nh;
  ros::NodeHandle leftNs;   // private node handle
  ros::NodeHandle rightNs;  // private node handle

  image_transport::ImageTransport *it_left;
  image_transport::ImageTransport *it_right;

  image_transport::CameraPublisher camPubLeft;
  image_transport::CameraPublisher camPubRight;

  camera_info_manager::CameraInfoManager *cinfo_left;
  camera_info_manager::CameraInfoManager *cinfo_right;

  sensor_msgs::Image leftImageMsg;
  sensor_msgs::Image rightImageMsg;

  sensor_msgs::CameraInfo leftCamInfo;
  sensor_msgs::CameraInfo rightCamInfo;

  std::string camera_name_right;
  std::string camera_name_left;

  std::string cameraConfUrlLeft;
  std::string cameraConfUrlRight;

  int cameraPort;
  int portLeft;
  int portRight;
};

#endif
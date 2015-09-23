#ifndef VRMAGIC_NODE_H
#define VRMAGIC_NODE_H

#include <string>

#include <ros/ros.h>

#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/fill_image.h>

#include <camera_info_manager/camera_info_manager.h>
#include <image_transport/image_transport.h>

#include "camera_handle.hpp"

namespace vrmagic {

static std::string camera_calibration_path = "package://vrmagic_camera/calibrations/${NAME}.yaml";

class VrMagicNode {
 public:
  explicit VrMagicNode(const ros::NodeHandle &nh, vrmagic::CameraHandle *cam_);

  ~VrMagicNode();

  void initCamera();
  void broadcastFrame();
  void spin();

 private:
  vrmagic::CameraHandle *cam;

  ros::NodeHandle nh;
  ros::NodeHandle leftNs;
  ros::NodeHandle rightNs;

  image_transport::ImageTransport *itLeft;
  image_transport::ImageTransport *itRight;

  image_transport::CameraPublisher camPubLeft;
  image_transport::CameraPublisher camPubRight;

  camera_info_manager::CameraInfoManager *cinfoLeft;
  camera_info_manager::CameraInfoManager *cinfoRight;

  sensor_msgs::Image leftImageMsg;
  sensor_msgs::Image rightImageMsg;

  sensor_msgs::CameraInfo leftCamInfo;
  sensor_msgs::CameraInfo rightCamInfo;

  std::string camera_name_right;
  std::string camera_name_left;

  std::string cameraConfUrlLeft;
  std::string cameraConfUrlRight;
};
}

#endif

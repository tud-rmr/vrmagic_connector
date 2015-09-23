#include "vrmagic_node.hpp"

#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>

using camera_info_manager::CameraInfoManager;

namespace vrmagic {

VrMagicNode::VrMagicNode(const ros::NodeHandle &nh_, CameraHandle *cam_) {
  nh = nh_;
  cam = cam_;

  leftNs = ros::NodeHandle(nh, "left");
  rightNs = ros::NodeHandle(nh, "right");

  itLeft = new image_transport::ImageTransport(leftNs);
  itRight = new image_transport::ImageTransport(rightNs);

  camPubLeft = image_transport::ImageTransport(leftNs).advertiseCamera("image_raw", 2);
  camPubRight = image_transport::ImageTransport(rightNs).advertiseCamera("image_raw", 2);

  cinfoLeft = new CameraInfoManager(leftNs, "vrmagic_left", camera_calibration_path);
  cinfoRight = new CameraInfoManager(rightNs, "vrmagic_right", camera_calibration_path);

  cinfoLeft->loadCameraInfo(camera_calibration_path);
  cinfoRight->loadCameraInfo(camera_calibration_path);

  ROS_INFO("Left calibrated: %s", cinfoLeft->isCalibrated() ? "true" : "false");
  ROS_INFO("Right calibrated: %s", cinfoRight->isCalibrated() ? "true" : "false");
}

VrMagicNode::~VrMagicNode() {
  delete itLeft;
  delete itRight;

  delete cinfoLeft;
  delete cinfoRight;
}

void VrMagicNode::broadcastFrame() {
  ros::Time triggerTime = ros::Time::now();

  cam->grabFrameLeft(leftImageMsg, triggerTime);
  cam->grabFrameRight(rightImageMsg, triggerTime);

  leftCamInfo = cinfoLeft->getCameraInfo();
  leftCamInfo.header.stamp = triggerTime;
  leftCamInfo.header.frame_id = leftImageMsg.header.frame_id;
  leftCamInfo.width = leftImageMsg.width;

  rightCamInfo = cinfoRight->getCameraInfo();
  rightCamInfo.header.stamp = triggerTime;
  rightCamInfo.header.frame_id = rightImageMsg.header.frame_id;
  rightCamInfo.width = rightImageMsg.width;

  camPubLeft.publish(leftImageMsg, leftCamInfo);
  camPubRight.publish(rightImageMsg, rightCamInfo);
}

void VrMagicNode::spin() { broadcastFrame(); }
}
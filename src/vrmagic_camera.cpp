#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>

#include "vrmagic_camera.hpp"

using camera_info_manager::CameraInfoManager;

VrMagicCamera::VrMagicCamera(const ros::NodeHandle &nh_, VrMagicCameraHandle cam_) {
  nh = nh_;
  cam = cam_;

  leftNs = ros::NodeHandle(nh, "left");
  rightNs = ros::NodeHandle(nh, "right");

  it_left = new image_transport::ImageTransport(leftNs);
  it_right = new image_transport::ImageTransport(rightNs);

  camPubLeft = image_transport::ImageTransport(leftNs).advertiseCamera("image_raw", 2);
  camPubRight = image_transport::ImageTransport(rightNs).advertiseCamera("image_raw", 2);

  cinfo_left = new CameraInfoManager(leftNs, camera_name_left, cameraConfUrlLeft);
  cinfo_right = new CameraInfoManager(rightNs, camera_name_right, cameraConfUrlRight);
}

VrMagicCamera::~VrMagicCamera() {
  delete it_left;
  delete it_right;

  delete cinfo_left;
  delete cinfo_right;
}

void VrMagicCamera::broadcastFrame() {
  ros::Time triggerTime = ros::Time::now();

  cam.grabFrame(1, leftImageMsg, triggerTime);
  cam.grabFrame(3, rightImageMsg, triggerTime);

  leftCamInfo.header.stamp = triggerTime;
  leftCamInfo.header.frame_id = frameId;
  leftCamInfo.width = leftImageMsg.width;

  rightCamInfo.header.stamp = triggerTime;
  rightCamInfo.header.frame_id = frameId;
  rightCamInfo.width = rightImageMsg.width;

  camPubLeft.publish(leftImageMsg, leftCamInfo);
  camPubRight.publish(rightImageMsg, rightCamInfo);
}

void VrMagicCamera::spin() {
  while (ros::ok()) {
    try {
      broadcastFrame();
    } catch (VRGrabException &ex) {
      std::cerr << ex << std::endl;
    }
    ros::spinOnce();

    // boost::lock_guard<boost::mutex> lock(timerAccess);
    // fpsLimit.sleep();
  }
}
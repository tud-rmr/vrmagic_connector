#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>

#include "vrmagic_node.hpp"

using camera_info_manager::CameraInfoManager;
using vrmagic::CameraHandle;

VrMagicNode::VrMagicNode(const ros::NodeHandle &nh_, CameraHandle *cam_) {
  nh = nh_;
  cam = cam_;

  leftNs = ros::NodeHandle(nh, "left");
  rightNs = ros::NodeHandle(nh, "right");

  itLeft = new image_transport::ImageTransport(leftNs);
  itRight = new image_transport::ImageTransport(rightNs);

  camPubLeft = image_transport::ImageTransport(leftNs).advertiseCamera("image_raw", 2);
  camPubRight = image_transport::ImageTransport(rightNs).advertiseCamera("image_raw", 2);

  cinfoLeft = new CameraInfoManager(leftNs, camera_name_left, cameraConfUrlLeft);
  cinfoRight = new CameraInfoManager(rightNs, camera_name_right, cameraConfUrlRight);
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

  leftCamInfo.header.stamp = triggerTime;
  leftCamInfo.header.frame_id = leftImageMsg.header.frame_id;
  leftCamInfo.width = leftImageMsg.width;

  rightCamInfo.header.stamp = triggerTime;
  rightCamInfo.header.frame_id = rightImageMsg.header.frame_id;
  rightCamInfo.width = rightImageMsg.width;

  camPubLeft.publish(leftImageMsg, leftCamInfo);
  camPubRight.publish(rightImageMsg, rightCamInfo);
}

void VrMagicNode::spin() {
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

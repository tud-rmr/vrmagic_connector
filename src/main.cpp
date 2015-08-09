#include <string>

#include <boost/move/move.hpp>

#include <ros/ros.h>

#include "vrmagic_node.hpp"
#include "camera_handle.hpp"

using std::string;

using vrmagic::CameraHandle;

static string LEFT_PORT = "left/port";
static string RIGHT_PORT = "right/port";

static string LEFT_GAIN = "left/gain";
static string RIGHT_GAIN = "right/gain";

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vrmagic_camera");
  ros::NodeHandle nh("vrmagic");

  vrmagic::Config config;

  // Set ports
  nh.getParam(LEFT_PORT, config.portLeft);
  nh.getParam(RIGHT_PORT, config.portRight);

  // Set gain
  if (nh.hasParam(LEFT_GAIN) || nh.hasParam(RIGHT_GAIN)) {
    nh.getParam(LEFT_GAIN, config.gainLeft);
    nh.getParam(RIGHT_GAIN, config.gainRight);
    config.setGain = true;
  }

  // I use auto_ptr here as ROS normally does not support C++11, which would give the better
  // unique_ptr. The reason why no raw pointer is to give the node the clear ownership of the camera
  std::auto_ptr<CameraHandle> cam(new CameraHandle(config));

  VrMagicNode node(nh, boost::move(cam));

  node.spin();
}

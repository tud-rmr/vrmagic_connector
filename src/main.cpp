#include <memory>

#include <boost/move/move.hpp>

#include <ros/ros.h>

#include "vrmagic_node.hpp"
#include "camera_handle.hpp"

using vrmagic::CameraHandle;

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vrmagic_camera");
  ros::NodeHandle nh("vrmagic");

  /*
    nh.param<int>("camera_port", cameraPort, 1);
    nh.param<int>("sensor_left", portLeft, 1);
    nh.param<int>("sensor_right", portRight, 3);
    nh.param("camera_settings_url_left", cameraConfUrlLeft, std::string("vrmagic_left.yaml"));
    nh.param("camera_settings_url_right", cameraConfUrlRight, std::string("vrmagic_left.yaml"));
  */
  vrmagic::Config config;
  config.setGain = true;
  config.gainLeft = 42;
  config.gainRight = 1338;

  // I use auto_ptr here as ROS normally does not support C++11, which would give the better
  // unique_ptr. The reason why no raw pointer is to give the node the clear ownership of the camera
  std::auto_ptr<CameraHandle> cam(new CameraHandle(config));

  VrMagicNode node(nh, boost::move(cam));

  node.spin();
}

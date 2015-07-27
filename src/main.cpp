#include <ros/ros.h>

#include "vrmagic_camera.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vrmagic_camera");
  ros::NodeHandle nh("vrmagic");

  VrMagicCamera node(nh);

  node.spin();
}

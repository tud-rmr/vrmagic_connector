#include <ros/ros.h>

#include "vrmagic_node.hpp"
#include "camera_handle.hpp"

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
  VrmConfig config;
  VrMagicCameraHandle *cam = new VrMagicCameraHandle(config);

  VrMagicNode node(nh, cam);

  node.spin();

  delete cam;
}

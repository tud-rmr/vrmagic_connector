#include <ros/ros.h>

#include "vrmagic_camera.hpp"
#include "camera_handle.hpp"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "vrmagic_camera");
  ros::NodeHandle nh("vrmagic");

  VrMagicCameraHandle cam;
  cam.init();

  ros::Time triggerTime = ros::Time::now();
  sensor_msgs::Image img;

  cam.grabFrame(1, img, triggerTime);

  // VrMagicCamera node(nh);
  // try {
  //   node.initCamera();
  // } catch (VRControlException &ex) {
  //   std::cerr << ex << std::endl;
  //   ros::shutdown();
  // }

  // node.spin();
}

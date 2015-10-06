// Copyright(c) 2015 Jan-Christoph Klie.

#include <signal.h>

#include <string>

#include <ros/ros.h>

#include "vrmagic_node.hpp"
#include "camera_handle.hpp"

using std::string;

using namespace vrmagic;

static const string ENABLE_LOGGING = "enable_logging";

static const string LEFT = "left/";
static const string RIGHT = "right/";

static const string LEFT_PORT = LEFT + "port";
static const string RIGHT_PORT = RIGHT + "port";

static const int LEFT_PORT_DEFAULT = 1;
static const int RIGHT_PORT_DEFAULT = 3;

static sig_atomic_t volatile g_request_shutdown = 0;

// Replacement SIGINT handler
static void mySigIntHandler(int sig) { g_request_shutdown = 1; }

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "vrmagic_camera", ros::init_options::NoSigintHandler);

  atexit(vrmagic::cameraShutdown);
  signal(SIGINT, mySigIntHandler);
  signal(SIGSEGV, mySigIntHandler);

  ros::NodeHandle nh("vrmagic");

  vrmagic::Config config;

  nh.param<bool>(ENABLE_LOGGING, config.enableLogging, false);

  // Set ports
  nh.param<int>(LEFT_PORT, config.portLeft, LEFT_PORT_DEFAULT);
  nh.param<int>(RIGHT_PORT, config.portRight, RIGHT_PORT_DEFAULT);

  CameraHandle* cam = new CameraHandle(config);

  VrMagicNode node(nh, cam);

  while (!g_request_shutdown) {
    node.spin();
    ros::spinOnce();
  }

  ROS_INFO("After shutdown");

  vrmagic::cameraShutdown();

  delete cam;

  ros::shutdown();
}

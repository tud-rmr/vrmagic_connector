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

static const string LEFT_GAIN = LEFT + "gain";
static const string RIGHT_GAIN = RIGHT + "gain";

static const string LEFT_EXPOSURE = LEFT + "exposure";
static const string RIGHT_EXPOSURE = RIGHT + "exposure";

static const int LEFT_PORT_DEFAULT = 1;
static const int RIGHT_PORT_DEFAULT = 3;

static sig_atomic_t volatile g_request_shutdown = 0;

/**
 * Ros does not allow to get float, just double
 * @param  nh    The node handle to get from
 * @param  key   Key of the param
 * @param  value Reference to the value which will be set if key exists
 * @return       True if value was found and set, else false
 */
static bool getFloatParam(const ros::NodeHandle nh, const string& key, float& value) {
  double tmp;
  bool status = nh.getParam(key, tmp);
  value = static_cast<float>(tmp);
  return status;
}

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

  // Set gain
  config.setGain = nh.getParam(LEFT_GAIN, config.gainLeft);
  config.setGain |= nh.getParam(RIGHT_GAIN, config.gainRight);

  // Set exposure
  config.setExposure = getFloatParam(nh, LEFT_EXPOSURE, config.exposureLeft);
  config.setExposure |= getFloatParam(nh, RIGHT_EXPOSURE, config.exposureRight);

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

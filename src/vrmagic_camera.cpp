#include <iostream>
#include <sstream>

#include <ros/ros.h>
#include <ros/console.h>

#include "vrmagic_camera.hpp"

using camera_info_manager::CameraInfoManager;

VrMagicCamera::VrMagicCamera(const ros::NodeHandle &nh_) {
  nh = nh_;

  leftNs = ros::NodeHandle(nh, "left");
  rightNs = ros::NodeHandle(nh, "right");

  nh.param<int>("camera_port", cameraPort, 1);
  nh.param<int>("sensor_left", portLeft, 1);
  nh.param<int>("sensor_right", portRight, 3);
  nh.param("camera_settings_url_left", cameraConfUrlLeft, std::string("vrmagic_left.yaml"));
  nh.param("camera_settings_url_right", cameraConfUrlRight, std::string("vrmagic_left.yaml"));

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

  if (!VRmUsbCamStop(device)) std::cerr << "VRmUsbCamStop failed." << std::endl;

  if (!VRmUsbCamCloseDevice(device)) std::cerr << "VRmUsbCamCloseDevice failed." << std::endl;

  VRmUsbCamCleanup();
}

void VrMagicCamera::initCamera() {
  atexit(VRmUsbCamCleanup);

  VRmDWORD libversion;
  if (!VRmUsbCamGetVersion(&libversion)) {
    ROS_ERROR("VRmUsbCamGetVersion failed.");
    throw VRControlException("VRmUsbCamGetVersion failed.");
  } else {
    ROS_INFO("VR Magic lib has version %d", libversion);
  }

  VRmDWORD size;
  if (!VRmUsbCamGetDeviceKeyListSize(&size)) {
    ROS_ERROR("VRmUsbCamGetDeviceKeyListSize failed.");
    throw VRControlException("VRmUsbCamGetDeviceKeyListSize failed.");
  } else {
    ROS_INFO("Found %d devices!", size);
  }

  if (size > 1) {
    ROS_ERROR("Found too many devices!");
    throw VRControlException("Found too many devices!");
  }

  VRmDeviceKey *devKey;
  if (!VRmUsbCamGetDeviceKeyListEntry(0, &devKey))
    throw VRControlException("VRmUsbCamGetDeviceKeyListEntry failed.");

  if (devKey->m_busy) throw VRControlException("device busy");

  if (!VRmUsbCamOpenDevice(devKey, &device))
    throw VRControlException("VRmUsbCamOpenDevice failed.");

  ROS_INFO("Device %s [%s] opened", devKey->mp_product_str, devKey->mp_manufacturer_str);

  if (!VRmUsbCamFreeDeviceKey(&devKey)) std::cerr << "VRmUsbCamFreeDeviceKey failed." << std::endl;

  VRmPropId mode = VRM_PROPID_GRAB_MODE_TRIGGERED_SOFT;
  if (!VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_MODE_E, &mode))
    throw VRControlException(
        "failed to set software trigger (VRM_PROPID_GRAB_MODE_TRIGGERED_SOFT).");

  // init camera
  VRmDWORD port = 0;  // id of Sensor //for multisensorunits

  VRmBOOL supported;
  // check number of connected sensors
  VRmDWORD num_sensorports = 0;
  VRmUsbCamGetSensorPortListSize(device, &num_sensorports);
  std::cout << "Found " << num_sensorports << " sensors" << std::endl;

  if (num_sensorports != NUMBER_OF_SENSORS) {
    throw VRControlException("This program needs exactly 2 sensors attached!");
  }

  VRmDWORD ports[NUMBER_OF_SENSORS];
  // for this demo all sensors are used
  for (VRmDWORD ii = 0; ii < NUMBER_OF_SENSORS; ii++) {
    VRmUsbCamGetSensorPortListEntry(device, ii, &port);

    // on single sensor devices this property does not exist
    VRmPropId sensor_enable = (VRmPropId)(VRM_PROPID_GRAB_SENSOR_ENABLE_1_B - 1 + port);
    VRmUsbCamGetPropertySupported(device, sensor_enable, &supported);
    if (supported) {
      VRmBOOL enable = 1;
      VRmUsbCamSetPropertyValueB(device, sensor_enable, &enable);
      // now get all sensor port
      VRmUsbCamGetSensorPortListEntry(device, ii, &ports[ii]);
      ROS_INFO("PORT: %d is used", ports[ii]);
    }
  }

  VRmUsbCamGetPropertySupported(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &supported);
  float value = 0.f;
  VRmUsbCamGetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
  std::cout << "ExposureTime: " << value << "ms, changeable: " << (supported ? "true" : "false")
            << std::endl;

  if (supported) {
    value = 5.f;
    VRmUsbCamSetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
    std::cout << "ExposureTime changed to: " << value << "ms" << std::endl;
  }

  if (!VRmUsbCamStart(device)) throw VRControlException("VRmUsbCamStart failed.");
}

void VrMagicCamera::broadcastFrame() {
  VRmRetVal success = VRmUsbCamSoftTrigger(device);

  if (!success) throw VRGrabException("VRmUsbCamSoftTrigger failed.");

  ros::Time triggerTime = ros::Time::now();

  grabFrame(1, leftImageMsg, triggerTime);
  grabFrame(3, rightImageMsg, triggerTime);

  leftCamInfo.header.stamp = triggerTime;
  leftCamInfo.header.frame_id = frame_id;
  leftCamInfo.width = leftImageMsg.width;

  rightCamInfo.header.stamp = triggerTime;
  rightCamInfo.header.frame_id = frame_id;
  rightCamInfo.width = rightImageMsg.width;

  camPubLeft.publish(leftImageMsg, leftCamInfo);
  camPubRight.publish(rightImageMsg, rightCamInfo);
}

void VrMagicCamera::grabFrame(VRmDWORD port,
                              sensor_msgs::Image &img,
                              const ros::Time &triggerTime) {
  img.width = width;
  img.height = height;
  img.step = width * 2;
  img.encoding = sensor_msgs::image_encodings::MONO16;
  img.data.resize(height * img.step);
  img.header.stamp = triggerTime;
  img.header.frame_id = frame_id;

  VRmImage *VRimg = NULL;
  VRmRetVal success = VRmUsbCamLockNextImageEx(device, port, &VRimg, NULL);

  if (!success) {
    std::stringstream err;
    err << "VRmUsbCamLockNextImageEx failed for port" << port << ".";
    throw VRGrabException(err.str().c_str());
  }

  for (unsigned int i = 0; i < height * width; i++) {
    img.data[i * 2 + 1] = VRimg->mp_buffer[i * 2] >> 6;
    img.data[i * 2] = (VRimg->mp_buffer[i * 2] << 2) | (VRimg->mp_buffer[i * 2 + 1] & 0x3);
  }

  if (!VRmUsbCamUnlockNextImage(device, &VRimg))
    throw VRGrabException("VRmUsbCamUnlockNextImage failed.");

  if (!VRmUsbCamFreeImage(&VRimg)) throw VRGrabException("VRmUsbCamFreeImage failed.");
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
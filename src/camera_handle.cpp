#include <iostream>
#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include "camera_handle.hpp"

static const int NUMBER_OF_SENSORS = 2;
static const float EXPOSURE_TIME = 5.f;
static const int PORT_LEFT = 1;
static const int PORT_RIGHT = 3;
static const std::string frameId("VRMAGIC");

VrMagicCameraHandle::VrMagicCameraHandle() {}

VrMagicCameraHandle::~VrMagicCameraHandle() {
  VRmUsbCamStop(device);
  VRmUsbCamCloseDevice(device);
}

void VrMagicCameraHandle::init() {
  atexit(VRmUsbCamCleanup);

  //-- scan for devices --

  // update
  VRmUsbCamUpdateDeviceKeyList();
  VRmDWORD size = 0;
  VRmUsbCamGetDeviceKeyListSize(&size);

  ROS_INFO("Found %d devices", size);

  if (size != 1) {
    ROS_INFO("Found more or none devices...");
    ROS_INFO("This programm can just deal with one device!");
    ROS_INFO("Will exit now....");
    exit(EXIT_FAILURE);
  }

  device = 0;
  VRmDeviceKey* p_device_key = 0;
  // open device
  VRmUsbCamGetDeviceKeyListEntry(0, &p_device_key);
  if (!p_device_key->m_busy) {
    VRmUsbCamOpenDevice(p_device_key, &device);
  }

  // display error when no camera has been found
  if (!device) {
    ROS_FATAL("No suitable VRmagic device found!");
    exit(EXIT_FAILURE);
  }

  ROS_INFO("Open deivce succesful");

  // init camera
  VRmImageFormat targetFormat;
  VRmDWORD port = 0;  // id of Sensor //for multisensorunits

  VRmBOOL supported;
  // check number of connected sensors
  VRmDWORD num_sensorports = 0;
  VRmUsbCamGetSensorPortListSize(device, &num_sensorports);
  ROS_INFO("Found %d sensors", num_sensorports);

  if (num_sensorports != 2) {
    ROS_FATAL("This program needs exactly 2 sensors attached!");
    exit(EXIT_FAILURE);
  }

  VRmDWORD ports[NUMBER_OF_SENSORS];
  // for this demo all sensors are used
  for (VRmDWORD ii = 0; ii < NUMBER_OF_SENSORS; ii++) {
    VRmUsbCamGetSensorPortListEntry(device, ii, &port);
    ROS_INFO("Found PORT: %d", port);

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

  setProperties();
  setTargetFormat();
  startCamera();
  prepareGrabbing();
}

void VrMagicCameraHandle::setProperties() {
  VRmBOOL supported;
  VRmUsbCamGetPropertySupported(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &supported);
  float value = 0.f;
  VRmUsbCamGetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
  ROS_INFO("ExposureTime: %f ms, changeable: %s", value, (supported ? "true" : "false"));

  if (supported) {
    value = EXPOSURE_TIME;
    VRmUsbCamSetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value);
    ROS_INFO("ExposureTime changed to: %f ms", value);
  }
}

void VrMagicCameraHandle::setTargetFormat() {
  VRmDWORD number_of_targetFormats, i;
  VRmSTRING imageFormatString;

  VRmUsbCamGetTargetFormatListSizeEx2(device, PORT_LEFT, &number_of_targetFormats);
  for (i = 0; i < number_of_targetFormats; ++i) {
    VRmUsbCamGetTargetFormatListEntryEx2(device, PORT_LEFT, i, &targetFormat);

    VRmUsbCamGetStringFromColorFormat(targetFormat.m_color_format, &imageFormatString);
    ROS_INFO("Supported: %s", imageFormatString);
    if (targetFormat.m_color_format == VRM_BGR_3X8) {
      ROS_INFO("BGR8 found !");
      break;
    }
  }
}

void VrMagicCameraHandle::startCamera() {
  ROS_INFO("Start camera");
  VRmUsbCamResetFrameCounter(device);
  VRmUsbCamStart(device);
}

void VrMagicCameraHandle::prepareGrabbing() {
  ROS_INFO("Prepare grabbing");
  VRmUsbCamNewImage(&targetImg, targetFormat);

  VRmSTRING imageFormatString;
  VRmUsbCamGetStringFromColorFormat(targetFormat.m_color_format, &imageFormatString);

  ROS_INFO("Color format: %s", imageFormatString);
  ROS_INFO("Width: %d", targetImg->m_image_format.m_width);
  ROS_INFO("Height: %d", targetImg->m_image_format.m_height);
  ROS_INFO("Image modifier: %d", targetImg->m_image_format.m_image_modifier);
  ROS_INFO("Image pitch: %d", targetImg->m_pitch);
}

void VrMagicCameraHandle::grabFrame(VRmDWORD port,
                                    sensor_msgs::Image& img,
                                    const ros::Time& triggerTime) {
  VRmDWORD framesDropped;

  img.width = targetImg->m_image_format.m_width;
  img.height = targetImg->m_image_format.m_height;
  img.step = targetImg->m_pitch;
  img.encoding = sensor_msgs::image_encodings::BGR8;
  img.data.resize(img.width * img.step);
  img.header.stamp = triggerTime;
  img.header.frame_id = frameId;

  VRmRetVal success = VRmUsbCamLockNextImageEx(device, port, &sourceImg, NULL);
  VRmUsbCamUnlockNextImage(device, &sourceImg);
  VRmUsbCamConvertImage(sourceImg, targetImg);

  for (unsigned int y = 0; y < img.height; y++) {
    for (unsigned int x = 0; x < img.width * 3; x++) {
      img.data[y * img.width * 3 + x] = targetImg->mp_buffer[y * img.step + x];
    }
  }

  if (framesDropped) {
    ROS_WARN("Frames dropped: %d", framesDropped);
  }
}
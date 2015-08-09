#include <iostream>
#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <ros/console.h>

#include "camera_handle.hpp"

namespace vrmagic {

static const VRmColorFormat TARGET_COLOR_FORMAT = VRM_BGR_3X8;

// Macros

#define VRM_CHECK(C)                                              \
  if (!(C)) {                                                     \
    ROS_ERROR("%s : Line %d", VRmUsbCamGetLastError(), __LINE__); \
    exit(EXIT_FAILURE);                                           \
  }

// Helper functions

static VRmPropId portnumToPropId(VRmDWORD port) {
  switch (port) {
    case 1:
      return VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1;
    case 2:
      return VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_2;
    case 3:
      return VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_3;
    case 4:
      return VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_4;
    default:
      ROS_FATAL("Cannot convert port to prop id: %d", port);
  }
}

// Member functions

CameraHandle::CameraHandle(VrmConfig conf) {
  // VRmUsbCamEnableLogging();
  this->config = conf;

  initCamera();
  setProperties();
  startCamera();
}

CameraHandle::~CameraHandle() {
  VRmUsbCamStop(device);
  VRmUsbCamCloseDevice(device);
}

void CameraHandle::initCamera() {
  atexit(VRmUsbCamCleanup);
  VRmDWORD size = 0;

  ROS_INFO("Scanning for devices");
  VRM_CHECK(VRmUsbCamGetDeviceKeyListSize(&size));
  ROS_INFO("Found %d devices", size);

  device = 0;
  VRmDeviceKey* p_device_key = 0;
  for (VRmDWORD i = 0; i < size && !device; ++i) {
    VRM_CHECK(VRmUsbCamGetDeviceKeyListEntry(i, &p_device_key));
    if (!p_device_key->m_busy) {
      VRM_CHECK(VRmUsbCamOpenDevice(p_device_key, &device));
    }
    VRM_CHECK(VRmUsbCamFreeDeviceKey(&p_device_key));
  }

  ROS_INFO("Trying to open device");

  if (!device) {
    ROS_FATAL("No suitable VRmagic device found!");
    exit(-1);
  }

  ROS_INFO("Device opened");

  // Activate the sensors of desire
  VRmPropId sensorLeftProp = portnumToPropId(config.portLeft);
  VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensorLeftProp);
  VRmPropId sensorRightProp = portnumToPropId(config.portRight);
  VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensorRightProp);

  // Get source format of the camera
  VRmImageFormat sourceFormat;

  VRM_CHECK(VRmUsbCamGetSourceFormatEx(device, config.portLeft, &sourceFormat));

  const char* source_color_format_str;
  VRM_CHECK(
      VRmUsbCamGetStringFromColorFormat(sourceFormat.m_color_format, &source_color_format_str));

  ROS_INFO("Selected source format: %d x %d (%s)",
           sourceFormat.m_width,
           sourceFormat.m_height,
           source_color_format_str);

  // select a target format from the list of formats we can convert the source images to.
  // we search for the screen's pixelformat for rendering
  VRmDWORD number_of_target_formats, i;
  VRM_CHECK(
      VRmUsbCamGetTargetFormatListSizeEx2(device, config.portLeft, &number_of_target_formats));
  for (i = 0; i < number_of_target_formats; ++i) {
    VRM_CHECK(VRmUsbCamGetTargetFormatListEntryEx2(device, config.portLeft, i, &targetFormat));
    if (targetFormat.m_color_format == TARGET_COLOR_FORMAT) break;
  }

  // Check for right target format
  if (targetFormat.m_color_format != TARGET_COLOR_FORMAT) {
    const char* screen_color_format_str;
    VRM_CHECK(VRmUsbCamGetStringFromColorFormat(TARGET_COLOR_FORMAT, &screen_color_format_str));
    ROS_FATAL("%s not found in target format list.", screen_color_format_str);
    exit(-1);
  }

  const char* target_color_format_str;
  VRM_CHECK(
      VRmUsbCamGetStringFromColorFormat(targetFormat.m_color_format, &target_color_format_str));
  ROS_INFO("Selected target format: %d x %d (%s)",
           targetFormat.m_width,
           targetFormat.m_height,
           target_color_format_str);
}

void CameraHandle::setProperties() {
  VRmBOOL supported;
  VRM_CHECK(VRmUsbCamGetPropertySupported(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &supported));
  float value = 0.f;
  VRM_CHECK(VRmUsbCamGetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value));
  ROS_INFO("ExposureTime: %f ms, changeable: %s", value, (supported ? "true" : "false"));

  if (supported) {
    value = EXPOSURE_TIME;
    VRM_CHECK(VRmUsbCamSetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value));
    ROS_INFO("ExposureTime changed to: %f ms", value);
  }
}

void CameraHandle::startCamera() {
  ROS_INFO("Start camera");

  VRM_CHECK(VRmUsbCamResetFrameCounter(device));
  VRM_CHECK(VRmUsbCamStart(device));
}

void CameraHandle::grabFrameLeft(sensor_msgs::Image& img, const ros::Time& triggerTime) {
  grabFrame(config.portLeft, img, triggerTime);
}

void CameraHandle::grabFrameRight(sensor_msgs::Image& img, const ros::Time& triggerTime) {
  grabFrame(config.portRight, img, triggerTime);
}

void CameraHandle::grabFrame(VRmDWORD port, sensor_msgs::Image& img, const ros::Time& triggerTime) {
  VRmImage* sourceImg = 0;
  VRmDWORD framesDropped = 0;

  if (VRmUsbCamLockNextImageEx2(device, port, &sourceImg, &framesDropped, 5000)) {
    VRmImage* targetImage = 0;
    VRM_CHECK(VRmUsbCamNewImage(&targetImage, targetFormat));
    VRM_CHECK(VRmUsbCamConvertImage(sourceImg, targetImage));

    // Fill in the image message with the converted frame from the camera
    img.width = targetImage->m_image_format.m_width;
    img.height = targetImage->m_image_format.m_height;
    img.step = img.width * 3;  // width * byte per pixel
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.data.resize(img.height * img.step);
    img.header.stamp = triggerTime;
    img.header.frame_id = frameId;

    // Convert from strided image to rectangular
    for (unsigned int y = 0; y < img.height; y++) {
      for (unsigned int x = 0; x < img.width * 3; x++) {
        img.data[y * img.width * 3 + x] = targetImage->mp_buffer[y * targetImage->m_pitch + x];
      }
    }

    VRM_CHECK(VRmUsbCamUnlockNextImage(device, &sourceImg));
  } else {
    ROS_FATAL("Could not lock image: %s", VRmUsbCamGetLastError());
  }
}
}
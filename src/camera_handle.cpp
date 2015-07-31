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
static const VRmColorFormat TARGET_COLOR_FORMAT = VRM_BGR_3X8;

#define VRMEXECANDCHECK(C)                                        \
  if (!(C)) {                                                     \
    ROS_ERROR("%s : Line %d", VRmUsbCamGetLastError(), __LINE__); \
    exit(EXIT_FAILURE);                                           \
  }

#define CHECK(C, M)                               \
  if (!(C)) {                                     \
    ROS_ERROR(M ": %s", VRmUsbCamGetLastError()); \
    exit(EXIT_FAILURE);                           \
  }

VrMagicCameraHandle::VrMagicCameraHandle() {
  // VRmUsbCamEnableLogging();
  initCamera();
  startCamera();
}

VrMagicCameraHandle::~VrMagicCameraHandle() {
  VRmUsbCamStop(device);
  VRmUsbCamCloseDevice(device);
}

void VrMagicCameraHandle::initCamera() {
  atexit(VRmUsbCamCleanup);
  VRmDWORD size = 0;

  ROS_INFO("Scanning for devices");
  VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListSize(&size));
  ROS_INFO("Found %d devices", size);

  device = 0;
  VRmDeviceKey* p_device_key = 0;
  for (VRmDWORD i = 0; i < size && !device; ++i) {
    VRMEXECANDCHECK(VRmUsbCamGetDeviceKeyListEntry(i, &p_device_key));
    if (!p_device_key->m_busy) {
      VRMEXECANDCHECK(VRmUsbCamOpenDevice(p_device_key, &device));
    }
    VRMEXECANDCHECK(VRmUsbCamFreeDeviceKey(&p_device_key));
  }

  ROS_INFO("Trying to open device");

  if (!device) {
    ROS_FATAL("No suitable VRmagic device found!");
    exit(-1);
  }

  ROS_INFO("Device opened");

  // Activate the sensors of desire
  VRmPropId sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_1;
  VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);
  sensor = VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_3;
  VRmUsbCamSetPropertyValueE(device, VRM_PROPID_GRAB_SENSOR_PROPS_SELECT_E, &sensor);

  // Get source format of the camera
  VRmImageFormat sourceFormat;

  VRMEXECANDCHECK(VRmUsbCamGetSourceFormatEx(device, PORT_LEFT, &sourceFormat));

  const char* source_color_format_str;
  VRMEXECANDCHECK(
      VRmUsbCamGetStringFromColorFormat(sourceFormat.m_color_format, &source_color_format_str));

  ROS_INFO("Selected target format: %d x %d (%s)",
           sourceFormat.m_width,
           sourceFormat.m_height,
           source_color_format_str);

  // select a target format from the list of formats we can convert the source images to.
  // we search for the screen's pixelformat for rendering
  VRmDWORD number_of_target_formats, i;
  VRMEXECANDCHECK(
      VRmUsbCamGetTargetFormatListSizeEx2(device, PORT_LEFT, &number_of_target_formats));
  for (i = 0; i < number_of_target_formats; ++i) {
    VRMEXECANDCHECK(VRmUsbCamGetTargetFormatListEntryEx2(device, PORT_LEFT, i, &targetFormat));
    if (targetFormat.m_color_format == TARGET_COLOR_FORMAT) break;
  }

  // Check for right target format
  if (targetFormat.m_color_format != TARGET_COLOR_FORMAT) {
    const char* screen_color_format_str;
    VRMEXECANDCHECK(
        VRmUsbCamGetStringFromColorFormat(TARGET_COLOR_FORMAT, &screen_color_format_str));
    ROS_FATAL("%s not found in target format list.", screen_color_format_str);
    exit(-1);
  }

  const char* target_color_format_str;
  VRMEXECANDCHECK(
      VRmUsbCamGetStringFromColorFormat(targetFormat.m_color_format, &target_color_format_str));
  ROS_INFO("Selected target format: %d x %d (%s)",
           targetFormat.m_width,
           targetFormat.m_height,
           target_color_format_str);
}

void VrMagicCameraHandle::setProperties() {
  VRmBOOL supported;
  CHECK(VRmUsbCamGetPropertySupported(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &supported),
        "Cannot check property support");
  float value = 0.f;
  CHECK(VRmUsbCamGetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value),
        "Cannot get property");
  ROS_INFO("ExposureTime: %f ms, changeable: %s", value, (supported ? "true" : "false"));

  if (supported) {
    value = EXPOSURE_TIME;
    CHECK(VRmUsbCamSetPropertyValueF(device, VRM_PROPID_CAM_EXPOSURE_TIME_F, &value),
          "Cannot get property");
    ROS_INFO("ExposureTime changed to: %f ms", value);
  }
}

void VrMagicCameraHandle::setTargetFormat() {
  VRmDWORD number_of_targetFormats, i;
  VRmSTRING imageFormatString;

  CHECK(VRmUsbCamGetTargetFormatListSizeEx2(device, PORT_LEFT, &number_of_targetFormats),
        "Cannot get target format list");
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

  VRMEXECANDCHECK(VRmUsbCamResetFrameCounter(device));
  VRMEXECANDCHECK(VRmUsbCamStart(device));
}

char hexmap[] = {'0', '1', '2', '3', '4', '5', '6', '7', '8', '9', 'a', 'b', 'c', 'd', 'e', 'f'};

std::string hexStr(unsigned char* data, int len) {
  std::string s(len * 2, ' ');
  for (int i = 0; i < len; ++i) {
    s[2 * i] = hexmap[(data[i] & 0xF0) >> 4];
    s[2 * i + 1] = hexmap[data[i] & 0x0F];
  }
  return s;
}

void VrMagicCameraHandle::grabFrame(VRmDWORD port,
                                    sensor_msgs::Image& img,
                                    const ros::Time& triggerTime) {
  VRmImage* sourceImg = 0;
  VRmDWORD framesDropped = 0;

  if (VRmUsbCamLockNextImageEx2(device, port, &sourceImg, &framesDropped, 5000)) {
    // std::string deb = hexStr(targetImg->mp_buffer, 100);

    VRmImage* targetImage = 0;
    VRMEXECANDCHECK(VRmUsbCamNewImage(&targetImage, targetFormat));
    VRMEXECANDCHECK(VRmUsbCamConvertImage(sourceImg, targetImage));

    // Fill in the image message with the converted frame from the camera
    img.width = targetImage->m_image_format.m_width;
    img.height = targetImage->m_image_format.m_height;
    img.step = targetImage->m_pitch;
    img.encoding = sensor_msgs::image_encodings::BGR8;
    img.data.resize(img.height * img.step);
    img.header.stamp = triggerTime;
    img.header.frame_id = frameId;

    for (unsigned int y = 0; y < img.height; y++) {
      for (unsigned int x = 0; x < img.width * 3; x++) {
        img.data[y * img.width * 3 + x] = targetImage->mp_buffer[y * targetImage->m_pitch + x];
      }
    }

    VRMEXECANDCHECK(VRmUsbCamUnlockNextImage(device, &sourceImg));
  } else {
    ROS_FATAL("Could not lock image: %s", VRmUsbCamGetLastError());
  }
}

void VrMagicCameraHandle::fillImageMessage(sensor_msgs::Image& img) {}
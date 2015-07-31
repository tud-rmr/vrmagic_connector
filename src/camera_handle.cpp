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

#define CHECK(C, M)                               \
  if (!(C)) {                                     \
    ROS_ERROR(M ": %s", VRmUsbCamGetLastError()); \
    exit(EXIT_FAILURE);                           \
  }

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

  VRmDeviceKey* devKey = 0;
  CHECK(VRmUsbCamGetDeviceKeyListEntry(0, &devKey), "Could not get device key list");

  if (!devKey->m_busy) {
    CHECK(VRmUsbCamOpenDevice(devKey, &device), "Could not open device");
    ROS_INFO("Opened device:");
    ROS_INFO("Manufacturer : %s", devKey->mp_manufacturer_str);
    ROS_INFO("Product: %s", devKey->mp_product_str);
    ROS_INFO("S/N: %d", devKey->m_serial);
  }

  // display error when no camera has been found
  if (!device) {
    ROS_FATAL("No suitable VRmagic device found!");
    exit(EXIT_FAILURE);
  }

  ROS_INFO("Opened device succesfully");

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
      CHECK(VRmUsbCamSetPropertyValueB(device, sensor_enable, &enable), "Cannot open port");
      // now get all sensor port
      CHECK(VRmUsbCamGetSensorPortListEntry(device, ii, &ports[ii]), "Cannot get port entry");
      ROS_INFO("PORT: %d will be used", ports[ii]);
    }
  }

  setProperties();
  setTargetFormat();
  startCamera();
  prepareGrabbing();
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

  CHECK(VRmUsbCamResetFrameCounter(device), "Error at resetting frame counter");
  CHECK(VRmUsbCamStart(device), "Error while starting camera");
}

void VrMagicCameraHandle::prepareGrabbing() {
  ROS_INFO("Prepare grabbing");
  CHECK(VRmUsbCamNewImage(&targetImg, targetFormat), "Could not create new image");

  VRmSTRING imageFormatString;
  CHECK(VRmUsbCamGetStringFromColorFormat(targetFormat.m_color_format, &imageFormatString),
        "Could not generate string from color format");

  ROS_INFO("Color format: %s", imageFormatString);
  ROS_INFO("Width: %d", targetImg->m_image_format.m_width);
  ROS_INFO("Height: %d", targetImg->m_image_format.m_height);
  ROS_INFO("Image modifier: %d", targetImg->m_image_format.m_image_modifier);
  ROS_INFO("Image pitch: %d", targetImg->m_pitch);
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
  VRmDWORD framesDropped;
  VRmRetVal success;

  img.width = targetImg->m_image_format.m_width;
  img.height = targetImg->m_image_format.m_height;
  img.step = targetImg->m_pitch;
  img.encoding = sensor_msgs::image_encodings::BGR8;
  img.data.resize(img.height * img.step);
  img.header.stamp = triggerTime;
  img.header.frame_id = frameId;

  ROS_INFO("Port %d", port);

  CHECK(VRmUsbCamLockNextImageEx(device, port, &sourceImg, &framesDropped),
        "Error at locking image");

  success = VRmUsbCamConvertImage(sourceImg, targetImg);
  if (!success) {
    ROS_ERROR("Error at converting image: %s", VRmUsbCamGetLastError());
  }

  std::string deb = hexStr(targetImg->mp_buffer, 100);

  // ROS_INFO("Hex: %s", deb.c_str());

  for (unsigned int y = 0; y < img.height; y++) {
    for (unsigned int x = 0; x < img.width * 3; x++) {
      img.data[y * img.width * 3 + x] = targetImg->mp_buffer[y * img.step + x];
    }
  }

  if (framesDropped) {
    // ROS_WARN("Frames dropped: %d", framesDropped);
  }

  success = VRmUsbCamUnlockNextImage(device, &sourceImg);
  if (!success) {
    ROS_ERROR("Error at unlocking image: %s", VRmUsbCamGetLastError());
  }
}

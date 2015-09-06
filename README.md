# vrmagic_connector

ROS package to connect a VRMagic USB camera into the ROS ecosystem. 

## Installation

In general, you need to install the VRmUsbCam DevKit found on the [VRMagic homepage](https://www.vrmagic.com/en/imaging/downloads/). The installation guide can be found in the extracted archive in the README. 

### Manual install 

As some problems occured while installing the DevKit side by side with ROS Jade, the following is a solution on how to circumvent problems with a conflicting Boost version. Try to install it according to the README first, as that is much less hacky.

Append the following line to the file /etc/apt/sources.list:

	deb http://repository.imaging.vrmagic.com/packages/imaging/repository0/linux/debian precise contrib

Then install the following packages
 
	sudo apt-get update
	sudo apt-get install vrmagic-linux-pc-camera-runtime 

Then, you either have to extract the lib and include folder from the *vrmagic-linux-pc-camera-sdk* (deb is just a zip archive) or install the deb I built. Problem was that the Camera SDK the vendor offers has dependencies to a version of Boost which is not compatible with my version of ROS. Therefore, one has to manually copy header and library to somewhere where CMake can find it, or use the package I built which can be installed side-by-side with ROS. Just leave me a mail and I hand it over. I do not want to check it in version control due to license issues. I asked the VRMagic support to publish a seperate development package without the Boost requirement, and wait for an answer. I do not tested my repackaged deb thourougly, so install it on your own risk. It worked for me. No warranty.

## Running demo

There is a launch file in the *launch* directory. It starts the node, opens the camera, and publishes images. You can start it with

	roslaunch vrmagic_camera camera.launch

from the catkin_ws root folder. This file also can be used to alter some parameters, like gain. In the future, more parameters will be supported. The images published can be checked with the ros imageview package. Just either run one of 

	rosrun image_view image_view image:=/vrmagic/left/image_raw
	rosrun image_view image_view image:=/vrmagic/right/image_raw

for the left or right image. You maybe have to include the namespace of your ROS core, depending on the configuration. If you do not find the correct topic name, just look at the published topics with tools like `rqt`.

## Properties

VRMagic USB camera has many properties, like gain or exposure, which can be set with the DevKit. There are two kinds of properties: Camera properties and sensor properties. Camera properties, like enabling the status LED, can only be set for the camera board itself. Sensor properties can be set for each sensor (left and right). The properties can currently be set via ROS parameter server. 

When setting properties, make sure that your board supports them. Not all boards support all properties. Attempting to set properties which are not supported results in a warning in the log. Setting properties which are supported but have a desired value outside of the specified range **are set to default**. 

Properties are right now not reset on boot, just overwritten if specified. An example launch file can be found in `launch/camera.launch`.

The properties currently supported are described in the following tables.

### Camera properties

|Name   	|Path  	|Type  	|Min   	|Max  	| Default 	| Description 	|
|---		|---	|---	|---	|---	| ---		| --- 			|
|   		|   	|   	|   	|   	|			|				|

	
### Sensor properties

Each of these properties has to be prefixed with either 'left' or 'right'. So to set the exposure of the left sensor, one has to set 'left/exposure'.

|Name   	|Path  		|Type  	|Min   	|Max  	| Default 	| Description 	|
|---		|---		|---	|---	|---	| ---		| --- 			|
| Gain 		|/gain 		| Int  	|   	|   	|			| Gain is an electronic amplification of the video signal. |
| Exposure	|/exposure	| Float	|   	| 64.0  | 32.0		| Exposure time [ms] is the length of time a camera's shutter is open when taking a photograph.|
| Port 		|/port 		| Int  	|   	|   	|			| The port number of the sensor, can be found on the white cable .|

### Add properties

Right now, not all properties available to a camera board can be set with this package. The follwing procedure describes how to add support for additional properties:

1. Add the property, a boolean flag to indicate whether it has to be set, and a default value in the constructor to the `Config` struct in `include/camera_handle.hpp`

2. Look up the property enum member in `vrmusbcam2props.h`. This file can either be found in the DevKit installation, normally in `/opt`, in `/usr/include` if installed with the custom *deb, or just extract the DevKit deb from the Vendor archive and find it there.

3. Add a function setXXX(void) and the respective declaration to the CameraHandle class. Call it in `CameraHandle::setProperties`. The body of the function should look for a sensor property like

		setPropertyLeftAndRight(conf.${propName}Left, conf.${propName}Right, ${propIdYouLookedUp});

4. If you want to set the parameter via launch file, extract it like done in `main.cpp` by checking whether the parameter has been set, and then set the config which is later passed to the CameraHandle of the connector node.

## Features missing 

- Dynamic reconfigure: Make the parameters changeable on runtime. Right now, they only can be altered on node startup
- Parameters for other camera boards. 
- Software trigger for boards which support it. That might improve the framerate.

# vrmagic_connector

ROS package to connect a VRMagic USB camera into the ROS ecosystem. 

## Installation

Append the following line to the file /etc/apt/sources.list:

	deb http://repository.imaging.vrmagic.com/packages/imaging/repository0/linux/debian precise contrib

Then install the following packages
 
	sudo apt-get update
	sudo apt-get install vrmagic-linux-pc-camera-runtime 

Then, you either have to extract the lib and include folder from the *vrmagic-linux-pc-camera-sdk* (deb is just a zip archive) or install the deb I built. Problem was that the Camera SDK the vendor offers has dependencies to a version of Boost which is not compatible with the version of ROS. Therefore, one has to manually copy header and library to somewhere where CMake can find it, or use the package I built which can be installed side-by-side with ROS. Just leave me a mail and I hand it over. I do not want to check it in due to license issues. I asked the VRMagic support to publish a seperate development package without the Boost requirement, and wait for an answer. I do not tested my repackaged deb thourougly, so install it on your own risk. It worked for me.

## Running demo

There is a launch file in the *launch* directory. It starts the node, opens the camera, and publishes images. You can start it with

	roslaunch vrmagic_camera camera.launch

from the catkin_ws root folder. This file also can be used to alter some parameters, like gain. In the future, more parameters will be supported. The images published can be checked with the ros imageview package. Just either run one of 

	rosrun image_view image_view image:=/vrmagic/left/image_raw
	rosrun image_view image_view image:=/vrmagic/right/image_raw

for the left or right image.

## Properties

There are two kinds of properties: Camera properties and sensor properties. Camera properties, like enabling the status LED, can only be set for the camera board itself. Sensor properties can be set for each sensor (left and right). The properties can currently be set via ROS parameter server. An example can be seen in the demo launch file. When setting properties, make sure that your board supports them. Not all boards support all properties. Attempting to set properties which are not supported results in a warning in the log. Setting properties which are supported but have a desired value outside of the specified range are set to default. Properties are right now not reset during runs.

The properties currently supported are

### Camera properties

|Name   	|Path  	|Type  	|Min   	|Max  	| Default 	| Description 	|
|---		|---	|---	|---	|---	| ---		| --- 			|
|   		|   	|   	|   	|   	|			|				|

	
### Sensor properties

Each of these properties has to be prefixed with either 'left' or 'right'. So to set the exposure of the left sensor, one has to set 'left/exposure'.

|Name   	|Path  	|Type  	|Min   	|Max  	| Default 	| Description 	|
|---		|---	|---	|---	|---	| ---		| --- 			|
| Gain 		|/gain 	|   	|   	|   	|			|				|

### Add properties

Right now, not all available properties can be set via launch file. That is 

# vrmagic_connector

ROS package to connect a VRMagic USB camera into the ROS ecosystem. 

## Installation

In general, you need to install the VRmUsbCam DevKit found on the [VRMagic homepage](https://www.vrmagic.com/en/imaging/downloads/). The installation guide can be found in the extracted archive in the README. 

### Manual install 

As some problems occured while installing the DevKit side by side with ROS Jade, the following is a solution on how to circumvent problems with a conflicting Boost version. Try to install it according to the README first, as that is much less hacky.

-ADD THE FOLLOWING LINE TO "/etc/apt/sources.list" (you will need to be in sudo to open the file with gedit, eventually):

deb http://repository.imaging.vrmagic.com/packages/imaging/repository0/linux/debian precise contrib

-TYPE IN A TERMINAL

$ sudo apt-get update
$ sudo apt-get install vrmagic-linux-pc-camera-runtime
$ sudo apt-get install -f

-INSTALL THE CUSTOM .DEB-FILE libvrmusbcam2-dev_3.5.0.0_amd64.deb (which will be provided by your tutor)

-TYPE IN A TERMINAL

$ cd ~/*YOUR_ROS_WORKSPACE*/src
$ git clone https://github.com/Rentier/vrmagic_connector.git
$ cd ..
$ catkin_make

-NEVER FORGET TO SOURCE YOUR WORKSPACE WITH:

$ source *PATH_OF_YOUR_WORKSPACE*/devel/source.list

-AND NEVER USE catkin_make IN ONE WORKSPACE, IF YOU HAVE AREADY SOURCE'D ANOTHER (don't hesitate to do it anyway, if you know what you are doing)

This is how you start to publish raw pictures from your camera:

-TYPE IN A TERMINAL

$ cd ~/*YOUR_ROS_WORKSPACE*
$ source *PATH_OF_YOUR_WORKSPACE*/devel/source.list
$ roscore

-TYPE IN SECOND TERMINAL

$ cd ~/*YOUR_ROS_WORKSPACE*
$ source *PATH_OF_YOUR_WORKSPACE*/devel/source.list
$ roslaunch vrmagic_camera camera.launch

-TYPE IN A THIRD TERMINAL

$ cd ~/*YOUR_ROS_WORKSPACE*
$ source *PATH_OF_YOUR_WORKSPACE*/devel/source.list
$ rosrun rviz rviz   (In this application you can see the pictures from your cameras)


-TROUBLESHOOTING:

If 'roslaunch vrmagic_camera camera.launch' fails and the terminal mentions problems with the ports, you can open the file "camera.launch" and edit the ports by hand.

ROBOT A - APOLLON - R2D2 > PORTS 1,3
ROBOT B - BOREAS - C3PO > PORTS 1,2


Append the following line to the file /etc/apt/sources.list:

	deb http://repository.imaging.vrmagic.com/packages/imaging/repository0/linux/debian precise contrib

Then install the following packages
 
	sudo apt-get update
	sudo apt-get install vrmagic-linux-pc-camera-runtime 

Then, you either have to extract the lib and include folder from the *vrmagic-linux-pc-camera-sdk* (deb is just a zip archive) or install the deb I built. Problem was that the Camera SDK the vendor offers has dependencies to a version of Boost which is not compatible with my version of ROS. Therefore, one has to manually copy header and library to somewhere where CMake can find it, or use the package I built which can be installed side-by-side with ROS. Just leave me a mail and I hand it over. I do not want to check it in version control due to license issues. I asked the VRMagic support to publish a seperate development package without the Boost requirement, and wait for an answer. I do not tested my repackaged deb thourougly, so install it on your own risk. It worked for me. No warranty.

## Running demo

There is a launch file in the *launch* directory. It starts the node, opens the camera, and publishes images. You can start it with

	roslaunch vrmagic_camera camera.launch

from the catkin_ws root folder. This file also can be used to alter some parameters, like the port numbers of the attached sensors. The images published can be checked with the ros imageview package. Just either run one of 

	rosrun image_view image_view image:=/vrmagic/left/image_raw
	rosrun image_view image_view image:=/vrmagic/right/image_raw

for the left or right image. You maybe have to include the namespace of your ROS core, depending on the configuration. If you do not find the correct topic name, just look at the published topics with tools like `rqt`.

## Calibration

This camera driver supports the standard ROS calibration methods. The calibration is currently stored under `calibration` in the package itself. 

Follow the following tutorial to calibrate the camera: 

http://wiki.ros.org/camera_calibration/Tutorials/StereoCalibration

In our lab we have a big checkerboard with the following parameters: --square 9x7 --size 0.1

    rosrun camera_calibration cameracalibrator.py --size 9x7 --square 0.10 right:=/vrmagic/right/image_raw left:=/vrmagic/left/image_raw right_camera:=/vrmagic/right left_camera:=/vrmagic/left
    
    rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.035 right:=/vrmagic/right/image_raw left:=/vrmagic/left/image_raw right_camera:=/vrmagic/right left_camera:=/vrmagic/left
    
    rosrun camera_calibration cameracheck.py --size 8x6 --square 0.035 stereo:=/vrmagic image:=image_rect

## Stereo proc

In order to run the stereo image processing node, just run

	ROS_NAMESPACE=vrmagic rosrun stereo_image_proc stereo_image_proc

after (assumed the default namespace of the vrmagic node). It then publishes the rectified images under `/vrmagic/{left,right}/image_rect`. You have to have the camera calibrated first. With the stereo proc node running, you can enjoy the stereo view with the *image_view* package. To display both rectified cameras and the disparity image, run

	rosrun image_view stereo_view stereo:=/vrmagic image:=image_rect

## Properties

To set properties like gain, exposure, et al. use CamLab, the GUI which comes with the VRMagic SDK. Set it once, save the properties on the camera, calibrate and then you can use that configuration without needing to change anything.

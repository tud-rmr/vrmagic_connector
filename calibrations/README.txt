Different calibration files are compared in this document.

Each set of calibration files are in different folder named by 
the date in which the calibration was performed.

It is recommended to test the calibration files and compare with this values
since the cameras could have been accidentaly moved.

We specify as well the ROS Command used to perform the test including the 
kind of checkerboard and the distance from camera to checkerboard.

It is desirable an epipolar error of less than 1 pixel (Epipolar Error Pixels)
and a correct estimation of the length of each square in the checkerboard (Epipolar dimension m)

Camera to Checkerboard distance: 1.5m
rosrun camera_calibration cameracheck.py --size 8x6 --square 0.035 stereo:=/vrmagic image:=image_rect
-----------------------------------------------------------------------------------
Calib Files  |   Robot            | Epipolar Error Pixels  | Epipolar dimension m
-----------------------------------------------------------------------------------
01.08.2016   |  Boreas            |     0.45               |       0.036
             |  Apollon           |  not yet tested        |
-----------------------------------------------------------------------------------


Camera to Checkerboard distance: 1.5m
rosrun camera_calibration cameracheck.py --size 8x6 --square 0.035 stereo:=/vrmagic image:=image_rect
-----------------------------------------------------------------------------------
Calib Files  |   Robot            | Epipolar Error Pixels  | Epipolar dimension m
-----------------------------------------------------------------------------------
09.08.2016   |  Boreas            |     0.5                |       0.035
             |  Apollon           |    no calibration file |
-----------------------------------------------------------------------------------


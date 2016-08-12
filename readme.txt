


** Requirements **

sudo apt-get install v4l-utils

** Parameters **
In the file MarkerLocator.py you will find a few parameters.You need to define
the list of markers which you want tracked, and you need to define if you want
the output printed or broadcasted as ROS messaged.

** Files **
CMakeLists.txt             ROS/CATKIN makelist file
fmMake.txt                 Make file related to FroboMind http://frobomind.org
MarkerLocator.py           The main file
MarkerPose.py              Python data structure for detected markers
MarkerTracker.py           Python class for detecting markers
matlab/                    Various matlab files used during development of the tracker
msg/                       Ros message definitions
package.xml                ROS/CATKIN Package file
patterns/                  Contains latex code for generating markers
performancetesting/        Script for estimating the marker location accuracy
PerspectiveTransform.py    Class for making perspective transformations
readme.txt                 This file
setup.py                   ROS/CATKIN Python setup file
TrackerInWindowMode.py     Windowed tracker that is faster but doesn't handle entering and leaving markers well

** ROS **
To run the MarkerLocator.py script as a ROS node you first need to make the
repository using CATKIN make. The reason is that a markerpose.msg file is
defined within this package.

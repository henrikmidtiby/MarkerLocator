


** Parameters **
In the file MarkerLocator.py you will find a few parameters.You need to define
the list of markers which you want tracked, and you need to define if you want
the output printed or broadcasted as ROS messaged.


** Files **
CMakeLists.txt             ROS/CATKIN makelist file
fmMake.txt                 Make file related to FroboMind http://frobomind.org
MarkerLocator.py           The main file
MarkerPose.py
MarkerTracker.py
matlab/
msg/
package.xml                ROS/CATKIN Package file
patterns/
performancetesting/
PerspectiveTransform.py
readme.txt                 This file
setup.py                   ROS/CATKIN Python setup file
TrackerInWindowMode.py

** ROS **
To run the MarkerLocator.py script as a ROS node you first need to make the
repository using CATKIN make. The reason is that a markerpose.msg file is
defined within this package.

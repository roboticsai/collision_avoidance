A ros package which don't lets a manual controlled robot to collide with any obstacle:
1. If object is inside local costmap of robot or say too near, that robot can't move forward.
2. It used depth sensor D415 to detect any obstacle.
3. Developed for covid response robot.

How to run this software:
To upload the arduino sketch from terminal:
./upload_sketch.sh
( This sketch contains all the code to control the dc motor driver/robot base)

To run the realsense package and collision avoidance node:
roslaunch detect_object detect_object.launch

Dependencies and setup:
1. Setup ros melodic/noetic in your pc or embedded linux device.
2. Build this package with catkin_make command.
3. Launch the launch file as above
 

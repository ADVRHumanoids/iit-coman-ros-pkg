iit-coman-ros-pkg
=================

Repository for the compliant humanoid robot COMAN from the Italian Institute of Technology.

#Package list:

coman_urdf: 
-----------
contains the urdf.xacro description of COMAN as well as files that are needed in the simulation in GAZEBO. 
Every time one of these files are changed you have to use the script inside the script/ folder to copy that file in 
coman_gazebo/sdf and in your /home/user/.gazebo/models folders. 
Before using the script check the dependency on BeautifulSoup4:
```
sudo easy_install beautifulsoup4
```
If you have problems with this command maybe you have to install python or the package easy_install.
After this, you have to be sure that coman_urdf is inside your ros path.
Either put it in your ros package path or do something like this:
```
export ROS_PACKAGE_PATH=${ROS_PACKAGE_PATH}:/home/your name/projects/walkman/iit-coman-ros-pkg
```

The script has to be called in script/ folder:
```
./create_urdf_and_sdf.sh
```
if called without any argument the coman sdf with yarp plugins is generated. 
If called with the yarp_accurate_ft
```
./create_urdf_and_sdf.sh yarp_accurate_ft
```
a sdf version of the robot is created containing yarp plugins, and FT sensors are defined as children of underactuated joints, added specifically to create a frame for accurate reading, and to take into account the bias given by the sensor's inertia. 
If called with the ros argument
```
./create_urdf_and_sdf.sh ros
```
an sdf will be created that includes the ros plugins, and the FT sensor will be defined at the actuated joints nearest to the actual position of the FT sensor in the real robot, so that the frame of reading is not accurate and a bias exists given by the uncompensated inertia of the sensor.
By running
```
./create_urdf_and_sdf.sh ros_accurate_ft
```
a sdf version of the robot is created containing ros plugins, and the FT sensor is placed in the correct pose by using the underactuated joint as in the case for the version of the sdf intended to be used with YARP. 
A bare sdf is not generated, but a bare version of the COMAN urdf (coman.urdf), not containing plugins and sensors, is generated, which is intended for use with external libraries needing a kinematically and dinamically accurate description of the robot in urdf format.

coman_gazebo:
-------------
contains files needed for the simulation as well as a launch file to start the simulation. 
To start a simulation:
```
roslaunch coman_gazebo coman_world.launch
```
coman_control:
--------------
contains the files needed to control the robot. For now the available controller is the effort_controllers/JointPositionController. 
To start the controller:
```
roslaunch coman_control coman_control.launch
```
It is also implemented a node that permits to send trajectory to the robot that were previously recorded in a file, 
to run this node:
```
rosrun coman_control send_trajectories
```
in another terminal runs:
```
rosrun rqt_reconfigure rqt_reconfigure
```
remember to run this command in the same folder were is the traj.dat file. The format of this file can be found in coman_control/src.

coman_state_publisher:
----------------------
contains a node that permits to publish inside ROS the state of the real robot. Communication with COMAN occurs 
through xddp_sockets. This package has an external dependency against Robolli libraries. 
We are working to make it independent. 

coman_msgs: 
-----------
contains msgs that are used in coman_state_publisher.

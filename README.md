iit-coman-ros-pkg
=================

Repository for the compliant humanoid robot COMAN from the Italian Institute of Technology.

Package list:
=================

coman_urdf: contains the urdf.xacro description of COMAN as well as files that are needed in the simulation in GAZEBO. Every time one of these files are changed you have to use the script inside the script/ folder to copy that file in coman_gazebo/sdf and in your /home/user/.gazebo/models folders. The actual version of COMAN does not have springs and sensors.

coman_gazebo: contains files needed for the simulation as well as a launch file to start the simulation. 
To start a simulation:
roslaunch coman_gazebo coman_world.launch

coman_control: contains the files needed to control the robot. For now the available controller is the effort_controllers/JointPositionController. 
To start the controller:
roslaunch coman_control coman_control.launch
It is also implemented a node that permits to send trajectory to the robot that were previously recorded in a file (anyway should be better to use rosbags...). To run this node:
rosrun coman_control send_trajectories
in another terminal runs:
rosrun rqt_reconfigure rqt_reconfigure
remember to run this command in the same folder were is the traj.dat file. The format of this file can be found in coman_control/src.

coman_state_publisher: contains a node that permits to publish inside ROS the state of the real robot. Communication with COMAN occurs through xddp_sockets. This package has an external dependency against Robolli libraries. We are working to make it independent. 

coman_msgs: contains msgs that are used in coman_state_publisher.

All the packages are still with the old rosbuild so rosmake them to compile. ROS supported version is Hydro.

For any problem pls contact me:
enrico.mingo@iit.it

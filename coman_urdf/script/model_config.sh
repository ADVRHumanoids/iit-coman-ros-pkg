#!/usr/bin/env bash

####################################################
##
##  FEET SELECTION
##
##
## options:
## 1) round feet
## 2) simple feet
##
####################################################
# TODO checks (must be mutually exclusive)
#
# 1) round feet
export GAZEBO_COMAN_USES_ROUND_FEET_13=false
# 2) simple feet
export GAZEBO_COMAN_USES_SIMPLE_FEET_DRC_sw1_14=true



####################################################
##
##  FOREARMS SELECTION
##
##
## options:
## 1) no forearms
## 2) 7dof forearms + soft hands
##
####################################################
# TODO: checks (must be mutually exclusive).. eventually, a menu!... documentation!
#       at the moment just the only combinations that work are:
# 
# 1) NO_FOREARMS + NO_HANDS
#
#export GAZEBO_COMAN_USES_NO_FOREARMS=true
#export GAZEBO_COMAN_USES_NO_HANDS=true
#export GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14=false
#export GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14=false
#
# 2) 7DOFS_FOREARMS + SOFTHANDS
#
export GAZEBO_COMAN_USES_NO_FOREARMS=false
export GAZEBO_COMAN_USES_NO_HANDS=false
export GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14=true
export GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14=true


####################################################
##
##  MIDDLEWARE SELECTION
##
##
## options:
#  notice how, regardless of the middleware selected
#  a bare version of the robot is created (.urdf)
#  1) yarp
#	The robot is created using yarp plugins for
#	the IMU and F/T sensors. The controlboard
#	plugins used to control joints and get
#	encoder and torque information are to be
#	inserted using the .world file, where the
#	initial configuration for the joints can
#	be specified (for example, look at the 
#	worlds folder)
#  2) ros
#	The sdf is created including ros plugins
#
##
####################################################
#
# 1) yarp
#
export GAZEBO_COMAN_USES_ROS=false
#
# 2) ros
#
export GAZEBO_COMAN_USES_YARP=true

####################################################
##
##  F/T SELECTION
##
##
## options:
## 1) accurate (ft sensors located at exact spot)
#	FT sensors are defined as children of 
#	underactuated joints, added specifically 
#	to create a frame for accurate reading, 
#	and to take into account the bias given
#	by the sensor's inertia. 
## 2) non-accurate (ft sensors at the nearest joint)
##
####################################################
#
# 1/2 (true/false) 
#
export GAZEBO_COMAN_USES_ACCURATE_FT=false


#!/bin/bash

cd ../urdf

echo "Creating bare urdf of coman.urdf.xacro"
rosrun xacro xacro.py coman.urdf.xacro > coman.urdf
echo "...urdf correctly created!"

# TODO checks (must be mutually exclusive)
export GAZEBO_COMAN_USES_SIMPLE_FEET_DRC_sw1_14=true
export GAZEBO_COMAN_USES_ROUND_FEET_13=false

# TODO: checks (must be mutually exclusive).. eventually, a menu!... documentation!
#       at the moment just the only combinations that work are:
# 
#       1) NO_FOREARMS + NO_HANDS
#
#export GAZEBO_COMAN_USES_NO_FOREARMS=true
#export GAZEBO_COMAN_USES_NO_HANDS=true
#export GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14=false
#export GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14=false
#
#       2) 7DOFS_FOREARMS + SOFTHANDS
export GAZEBO_COMAN_USES_NO_FOREARMS=false
export GAZEBO_COMAN_USES_NO_HANDS=false
export GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14=true
export GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14=true

if [ $# -gt 0 ]
  then
  if [ $1 == "ros" ] 
    then
      export GAZEBO_COMAN_USES_ROS=true
      export GAZEBO_COMAN_USES_YARP=false
      export GAZEBO_COMAN_USES_ACCURATE_FT=false
      echo "Creating urdf of coman_robot.urdf.xacro with ROS plugins"
  elif [ $1 == "ros_accurate_ft" ] 
    then
      export GAZEBO_COMAN_USES_ROS=true
      export GAZEBO_COMAN_USES_YARP=false
      export GAZEBO_COMAN_USES_ACCURATE_FT=true
      echo "Creating urdf of coman_robot.urdf.xacro with ROS plugins using accurate ft sensor description"
  elif [ $1 == "yarp_accurate_ft" ] 
    then
      export GAZEBO_COMAN_USES_ROS=false
      export GAZEBO_COMAN_USES_YARP=true
      export GAZEBO_COMAN_USES_ACCURATE_FT=true
      echo "Creating urdf of coman_robot.urdf.xacro with YARP plugins using accurate ft sensor description"
  fi
else
    export GAZEBO_COMAN_USES_ROS=false
    export GAZEBO_COMAN_USES_YARP=true
    export GAZEBO_COMAN_USES_ACCURATE_FT=false
    echo "Creating urdf of coman_robot.urdf.xacro with YARP plugins"
fi

rosrun xacro xacro.py coman_robot.urdf.xacro > coman_robot.urdf
echo "...urdf correctly created!"

echo "Creating sdf of coman_robot.urdf..."
gzsdf print coman_robot.urdf > coman.sdf

python ../script/gazebowtf.py coman.gazebo.wtf > coman2.sdf

mv coman2.sdf coman.sdf

unset GAZEBO_COMAN_USES_YARP
unset GAZEBO_COMAN_USES_ROS
unset GAZEBO_COMAN_USES_ACCURATE_FT
unset GAZEBO_COMAN_USES_ROUND_FEET_13
unset GAZEBO_COMAN_USES_SIMPLE_FEET_DRC_sw1_14
unset GAZEBO_COMAN_USES_NO_FOREARMS
unset GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14
unset GAZEBO_COMAN_USES_NO_HANDS
unset GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14

echo "...sdf correctly created!"

echo "Removing coman_robot.urdf."
rm coman_robot.urdf

echo "Copying coman.sdf in coman_gazebo/sdf."
cp coman.sdf ../../coman_gazebo/sdf

echo "Removing coman.sdf"
rm coman.sdf

cd ..

echo "Copying meshes in coman_gazebo/sdf."
cp -r meshes/ ../coman_gazebo/sdf

cd ../coman_gazebo

echo "Copying all data in coman_gazebo/sdf in ~/.gazebo/models/coman_urdf"
mkdir coman_urdf

cd sdf

cp -r * ../coman_urdf

cd ../

rm -rf ~/.gazebo/models/coman_urdf
cp -r coman_urdf ~/.gazebo/models

rm -rf coman_urdf
echo "Finish! Enjoy COMAN in GAZEBO!"


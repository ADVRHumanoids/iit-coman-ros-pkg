#!/bin/bash

cd ../urdf

echo "Creating bare urdf of coman.urdf.xacro"
rosrun xacro xacro.py coman.urdf.xacro > coman.urdf
echo "...urdf correctly created!"

if [ $# -gt 0 ]
  then
  if [ $1 == "ros" ] 
    then
      GAZEBO_COMAN_USES_ROS=true
      GAZEBO_COMAN_USES_YARP=false
      GAZEBO_COMAN_USES_ACCURATE_FT=false
      echo "Creating urdf of coman_robot.urdf.xacro with ROS plugins"
  elif [ $1 == "ros_accurate_ft" ] 
    then
      GAZEBO_COMAN_USES_ROS=true
      GAZEBO_COMAN_USES_YARP=false
      GAZEBO_COMAN_USES_ACCURATE_FT=true
      echo "Creating urdf of coman_robot.urdf.xacro with ROS plugins using accurate ft sensor description"
  elif [ $1 == "yarp_accurate_ft" ] 
    then
      GAZEBO_COMAN_USES_ROS=false
      GAZEBO_COMAN_USES_YARP=true
      GAZEBO_COMAN_USES_ACCURATE_FT=true
      echo "Creating urdf of coman_robot.urdf.xacro with YARP plugins using accurate ft sensor description"
  fi
else
    GAZEBO_COMAN_USES_ROS=false
    GAZEBO_COMAN_USES_YARP=true
    GAZEBO_COMAN_USES_ACCURATE_FT=false
    echo "Creating urdf of coman_robot.urdf.xacro with YARP plugins"
fi

export GAZEBO_COMAN_USES_YARP
export GAZEBO_COMAN_USES_ROS
export GAZEBO_COMAN_USES_ACCURATE_FT

rosrun xacro xacro.py coman_robot.urdf.xacro > coman_robot.urdf
echo "...urdf correctly created!"

echo "Creating sdf of coman_robot.urdf..."
gzsdf print coman_robot.urdf > coman.sdf

python ../script/gazebowtf.py coman.gazebo.wtf > coman2.sdf

mv coman2.sdf coman.sdf

unset GAZEBO_COMAN_USES_YARP
unset GAZEBO_COMAN_USES_ROS
unset GAZEBO_COMAN_USES_ACCURATE_FT

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

cp -r coman_urdf ~/.gazebo/models

rm -rf coman_urdf
echo "Finish! Enjoy COMAN in GAZEBO!"


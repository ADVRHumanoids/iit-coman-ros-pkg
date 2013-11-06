#!/bin/bash

cd ../urdf

echo "Creating bare urdf of coman.urdf.xacro"
rosrun xacro xacro.py coman.urdf.xacro > coman.urdf
echo "...urdf correctly created!"

if [ $# == "ros" ]
  then
    GAZEBO_COMAN_USES_ROS=true
    GAZEBO_COMAN_USES_YARP=false
    echo "Creating urdf of coman_robot.urdf.xacro with ROS plugins"
   else
    GAZEBO_COMAN_USES_ROS=false
    GAZEBO_COMAN_USES_YARP=true
    echo "Creating urdf of coman_robot.urdf.xacro with YARP plugins"
fi

export GAZEBO_COMAN_USES_YARP
export GAZEBO_COMAN_USES_ROS
rosrun xacro xacro.py coman_robot.urdf.xacro > coman_robot.urdf
echo "...urdf correctly created!"

echo "Creating sdf of coman_robot.urdf..."
gzsdf print coman_robot.urdf > coman.sdf

if [ $GAZEBO_COMAN_USES_YARP ]
  then
    python ../script/gazebowtf.py coman.gazebo.wtf > coman2.sdf
    mv coman2.sdf coman.sdf
fi

unset GAZEBO_COMAN_USES_YARP
unset GAZEBO_COMAN_USES_ROS

echo "...sdf correctly created!"

echo "Removing coman_robot.urdf."
#rm coman_robot.urdf

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


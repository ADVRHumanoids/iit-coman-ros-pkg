#!/bin/bash

cd ../urdf

echo "Creating bare urdf of coman.urdf.xacro"
rosrun xacro xacro.py coman.urdf.xacro > coman.urdf
echo "...urdf correctly created!"

rosrun xacro xacro.py coman_robot.urdf.xacro > coman_robot.urdf
echo "...urdf correctly created!"

echo "Creating sdf of coman_robot.urdf..."
gzsdf print coman_robot.urdf > coman.sdf

python ../script/gazebowtf.py coman.gazebo.wtf > coman2.sdf

mv coman2.sdf coman.sdf

#unset GAZEBO_COMAN_USES_YARP
#unset GAZEBO_COMAN_USES_ROS
#unset GAZEBO_COMAN_USES_ACCURATE_FT
#unset GAZEBO_COMAN_USES_ROUND_FEET_13
#unset GAZEBO_COMAN_USES_SIMPLE_FEET_DRC_sw1_14
#unset GAZEBO_COMAN_USES_NO_FOREARMS
#unset GAZEBO_COMAN_USES_7DOF_FOREARMS_DRC_14
#unset GAZEBO_COMAN_USES_NO_HANDS
#unset GAZEBO_COMAN_USES_SOFTHANDS_DRC_sw1_14

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


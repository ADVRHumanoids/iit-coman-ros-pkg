#!/bin/bash

RED='\033[0;31m'
PURPLE='\033[0;35m'
GREEN='\033[0;32m'
ORANGE='\033[0;33m'
YELLOW='\033[1;33m'
NC='\033[0m'

if [ "$#" -lt 1 ]; then
       printf "${RED}No robot name argument passed!${NC}" 
       echo 
       echo "Try something like:"
       echo "./create_urdf_srdf_sdf.sh coman"
       exit
fi

robot_name="$1"
printf "Robot Name is ${GREEN}${robot_name}${NC}"
echo

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))
cd $SCRIPT_ROOT
cd ../urdf

# check for Gazebo4 gz
IS_GZSDF_GAZEBO4=true;
type gz >/dev/null 2>&1 || { IS_GZSDF_GAZEBO4=false; }

if [ -d config ]; then

    echo "Regenerating database.config for ${robot_name}_gazebo"

cat > ../../${robot_name}_gazebo/database/database.config << EOF
<?xml version='1.0'?>
<database>
<name>$robot_name Gazebo Database</name>
<license>Creative Commons Attribution 3.0 Unported</license>
<models>
EOF

    for i in config/*.urdf.xacro; do
        cd $SCRIPT_ROOT
        cd ../urdf
        if [ -r $i ]; then

            echo "Processing file $i"
            model_name="`python ../script/get_model_params.py ${i} model_name`"
            model_version="`python ../script/get_model_params.py ${i} model_version`"
            model_filename=$(basename $i ".urdf.xacro")
            echo "${model_filename} configures model ${model_name}, version ${model_version}"

            cp $i ${robot_name}_config.urdf.xacro
            mkdir -p ../../${robot_name}_gazebo/database/$model_filename
            echo "<uri>file://${model_filename}</uri>" >> ../../${robot_name}_gazebo/database/database.config

rm -f ../../${robot_name}_gazebo/database/$model_filename/model.config                    
cat >> ../../${robot_name}_gazebo/database/$model_filename/model.config << EOF
<?xml version="1.0"?>
<model>
  <name>$model_name</name>
  <version>$model_version</version>
  <sdf version='1.4'>${robot_name}.sdf</sdf>

  <author>
   <name>Enrico Mingo</name>
   <email>enrico.mingo@iit.it</email>
  </author>

  <description>
    Simulation of the $robot_name Humanoid Robot from IIT.
  </description>
</model>
EOF
                    
            cp ../../${robot_name}_gazebo/database/$model_filename/model.config ../../${robot_name}_gazebo/database/$model_filename/manifest.xml
            cp ../../${robot_name}_gazebo/database/$model_filename/model.config ../../${robot_name}_gazebo/database/$model_filename/${model_filename}.config


            printf "${PURPLE}Creating bare urdf of ${robot_name}.urdf.xacro ...${NC}\n"
            rosrun xacro xacro --check-order ${robot_name}.urdf.xacro > ${model_filename}.urdf
            printf "${GREEN}...${model_filename}.urdf correctly created!${NC}\n"
            echo
            echo

            printf "${PURPLE}Creating capsule urdf of ${robot_name}.urdf.xacro ...${NC}\n"
            robot_capsule_urdf ${model_filename}.urdf --output
            printf "${GREEN}...${model_filename}_capsules.urdf correctly created!${NC}\n"
            echo
            echo

            printf "${PURPLE}Creating sdf of ${robot_name}_robot.urdf.xacro${NC}\n"
            rosrun xacro xacro --check-order ${robot_name}_robot.urdf.xacro > ${model_filename}_robot.urdf
            rosrun xacro xacro --check-order ${robot_name}_robot.urdf.xacro > ${robot_name}_robot.urdf
            if [ $IS_GZSDF_GAZEBO4 == true ]; then
            	gz sdf --print ${robot_name}_robot.urdf > ${robot_name}.sdf
	        else
                gzsdf print ${robot_name}_robot.urdf > ${robot_name}.sdf
            fi
            
	    if [ ${robot_name} = ${model_filename} ];
	    then
		printf "${GREEN} changing ${model_filename}_robot.urdf name to _${model_filename}_robot.urdf\n"
		rosrun xacro xacro --check-order ${robot_name}_robot.urdf.xacro > _${model_filename}_robot.urdf		
	    fi
            rm ${robot_name}_robot.urdf
            printf "${GREEN}...sdf correctly created!${NC}\n"
            echo
            echo

            echo "Installing robot model in ${robot_name}_gazebo/${model_filename}"
            mv ${robot_name}.sdf ../../${robot_name}_gazebo/database/${model_filename}/
            echo
            echo

            printf "${PURPLE}Creating srdf from ${robot_name}.srdf.xacro${NC}\n"
            cd ../../${robot_name}_srdf/srdf
            rosrun xacro xacro --check-order ${robot_name}.srdf.xacro > ${model_filename}.srdf 
            printf "${GREEN}...created ${model_filename}.srdf!${NC}\n"
            echo
            echo

            printf "${PURPLE}Creating capsule srdf of ${robot_name}.srdf.xacro ...${NC}\n"
            cp ${model_filename}.srdf ${model_filename}_capsules.srdf
            printf "${GREEN}...${model_filename}_capsules.srdf correctly created!${NC}\n"
            echo
            echo
            

            cd $SCRIPT_ROOT

            echo
            ./load_acm.py ../../${robot_name}_srdf/srdf/${model_filename}.srdf --output
            ./load_acm.py ../../${robot_name}_srdf/srdf/${model_filename}_capsules.srdf --output
            printf "${RED}skipping computation of default allowed collision detection matrix${NC}\n"
            printf "${YELLOW}Please make sure the ACM are up-to-date by running ${ORANGE}make acm${YELLOW} in the model build folder${NC}\n"


            echo 
            echo
            echo
            printf "${GREEN}Complete! Enjoy ${model_name} ver ${model_version} in GAZEBO!${NC}\n"
            echo
            echo
        fi
    done

    cd $SCRIPT_ROOT

    cd ../urdf
    
    echo "</models>" >> ../../${robot_name}_gazebo/database/database.config
    echo "</database>" >> ../../${robot_name}_gazebo/database/database.config
    
    unset i

    printf "${PURPLE}Creating capsule urdf (for visualization) of ${robot_name}_capsules.urdf ...${NC}\n"
    robot_capsule_urdf_to_rviz ${robot_name}_capsules.urdf --output
    printf "${GREEN}...${robot_name}_capsules.rviz correctly created! You can use view it by calling roslaunch ${robot_name}_urdf ${robot_name}_capsules_slider.launch${NC}\n"
    echo
    echo
    
    rm ${robot_name}_config.urdf.xacro
    mkdir -p ../../${robot_name}_gazebo/database/${robot_name}_urdf/
    cp -r ../meshes/ ../../${robot_name}_gazebo/database/${robot_name}_urdf/
else
    echo "Error: could not find config folder in the urdf path"
fi

cd $SCRIPT_ROOT
cd ../urdf
cp config/${robot_name}.urdf.xacro ${robot_name}_config.urdf.xacro

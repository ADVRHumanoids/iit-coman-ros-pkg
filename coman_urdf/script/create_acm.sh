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
       echo "./create_urdf_srdf_sdf.sh cogimon"
       exit
fi

robot_name="$1"
printf "Robot Name is ${GREEN}${robot_name}${NC}"
echo

# this way the script can be called from any directory
SCRIPT_ROOT=$(dirname $(readlink --canonicalize --no-newline $BASH_SOURCE))
cd $SCRIPT_ROOT
cd ../urdf

if [ -d config ]; then

    for i in config/*.urdf.xacro; do
        cd $SCRIPT_ROOT
        cd ../urdf

        if [ -r $i ]; then

            echo "Processing file $i"
            model_name="`python ../script/get_model_params.py ${i} model_name`"
            model_version="`python ../script/get_model_params.py ${i} model_version`"
            model_filename=$(basename $i ".urdf.xacro")

            cd $SCRIPT_ROOT

            HAS_MOVEIT_CDC=true;
            type moveit_compute_default_collisions >/dev/null 2>&1 || { HAS_MOVEIT_CDC=false; }

            if [ $HAS_MOVEIT_CDC == true ]; then
                echo
                echo
                printf "${RED}computing default allowed collision detection matrix for ${model_filename}...${NC}\n"
                moveit_compute_default_collisions --urdf_path ../urdf/${model_filename}.urdf --srdf_path ../../${robot_name}_srdf/srdf/${model_filename}.srdf --num_trials 100000
                ./save_acm.py ../../${robot_name}_srdf/srdf/${model_filename}.srdf --output
                printf "${GREEN}..finished computing default allowed collision detection matrix for ${model_filename}${NC}\n"

                echo
                echo
                printf "${RED}computing default allowed collision detection matrix for ${model_filename}_capsules...${NC}\n"
                #ATM using cylinders - and not capsules - is more conservative when creating the ACM
                #moveit_compute_default_collisions --urdf_path ../urdf/${model_filename}_capsules.urdf --srdf_path ../../${robot_name}_srdf/srdf/${model_filename}_capsules.srdf --num_trials 75000 --cylinders_to_capsules
                moveit_compute_default_collisions --urdf_path ../urdf/${model_filename}_capsules.urdf --srdf_path ../../${robot_name}_srdf/srdf/${model_filename}_capsules.srdf --num_trials 75000
                ./save_acm.py ../../${robot_name}_srdf/srdf/${model_filename}_capsules.srdf --output
                printf "${GREEN}..finished computing default allowed collision detection matrix for ${model_filename}_capsule${NC}\n"
            else
                echo
                printf "${RED}ERROR! moveit_compute_default_collisions was not found. Exiting${NC}\n"
                exit
            fi

        fi
    done

    cd $SCRIPT_ROOT
    cd ../urdf
    
    unset i
    
else
    echo "Error: could not find config folder in the urdf path"
fi

cd $SCRIPT_ROOT
cd ../urdf

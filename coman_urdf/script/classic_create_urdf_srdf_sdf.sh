    # check for Gazebo4 gz
    IS_GZSDF_GAZEBO4=true;
    type gz >/dev/null 2>&1 || { IS_GZSDF_GAZEBO4=false; }

    cd ../urdf

    echo "Creating bare urdf of coman.urdf.xacro"
    rosrun xacro xacro.py coman.urdf.xacro > coman.urdf
    echo "...urdf correctly created!"

    echo "Creating urdf of coman_robot.urdf.xacro"
    rosrun xacro xacro.py coman_robot.urdf.xacro > coman_robot.urdf
    echo "...urdf correctly created!"

    echo "Creating sdf of coman_robot.urdf..."
    if [ $IS_GZSDF_GAZEBO4 == true ]; then
    	gz sdf --print coman_robot.urdf > coman.sdf
    else
	gzsdf print coman_robot.urdf > coman.sdf
    fi

    python ../script/gazebowtf.py wtf/coman.gazebo.wtf coman_config.urdf.xacro > coman2.sdf

    mv coman2.sdf coman.sdf

    echo "...sdf correctly created!"

    echo "Removing coman_robot.urdf."
    rm coman_robot.urdf


    echo "Moving coman.sdf in coman_gazebo/sdf."
    mv coman.sdf ../../coman_gazebo/sdf/

    echo "Copying meshes in coman_gazebo/sdf."
    cp -r ../meshes/ ../../coman_gazebo/sdf/

    cd ../../coman_gazebo

    echo "Copying all data in coman_gazebo/sdf in ~/.gazebo/models/coman_urdf"

    rm -rf ~/.gazebo/models/coman_urdf
    rm -rf ~/.gazebo/models/coman
    mkdir -p ~/.gazebo/models/coman_urdf
    cp -r sdf ~/.gazebo/models/coman
    cp -r sdf/meshes ~/.gazebo/models/coman/meshes
    cp -r sdf/meshes ~/.gazebo/models/coman_urdf/meshes
    cp -r sdf/conf ~/.gazebo/models/coman_urdf/conf

    echo "Creating srdf from coman.srdf.xacro"
    cd ../coman_srdf/srdf
    rosrun xacro xacro.py coman.srdf.xacro > coman.srdf 
    echo "...created coman.srdf!"
     
    echo "Finish! Enjoy COMAN in GAZEBO!"

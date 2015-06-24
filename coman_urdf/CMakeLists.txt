cmake_minimum_required(VERSION 2.8)
PROJECT(IITComanRosPkg)

add_custom_target(  acm
                    COMMAND ${CMAKE_SOURCE_DIR}/coman_urdf/script/create_acm.sh)

add_custom_target(	remove_old_coman_urdf_gazebo ALL
    				COMMAND ${CMAKE_COMMAND} -E remove_directory 
        			"~/.gazebo/models/coman_urdf/")

add_custom_target(  remove_old_coman_gazebo ALL
                    COMMAND ${CMAKE_COMMAND} -E remove_directory 
                    "~/.gazebo/models/coman/")

add_custom_target(  compile_urdf_sdf ALL
                    COMMAND ${CMAKE_SOURCE_DIR}/coman_urdf/script/create_urdf_srdf_sdf.sh coman)

if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    add_custom_target(	copy_coman_gazebo ALL
        				COMMAND ${CMAKE_COMMAND} -E copy_directory 
            			"${PROJECT_SOURCE_DIR}/coman_gazebo/sdf/" 
            			"~/.gazebo/models/coman/"
            			DEPENDS remove_old_coman_gazebo)
endif()

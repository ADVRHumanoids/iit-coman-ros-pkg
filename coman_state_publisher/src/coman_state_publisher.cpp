#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <broadcast_data.h>
#include <definitions.h>
#include <errno.h>
#include <string>
#include <coman_msgs/ForceTorque.h>
#include <coman_msgs/JointExtraInformation.h>
#include <coman_msgs/PID.h>


#define freq 100
#define mRAD2RAD(X)  (((double) X)/1e5)
#define mNmPerRAD2NmPerRAD(X) ((double)X/1e3)

std::vector<std::string> joints_names = {"WaistYaw", "WaistLat", "WaistSag",
                                         "RHipSag", "LHipSag", "RHipLat", "RHipYaw", "RKneeSag", "RAnkLat", "RAnkSag", "LHipLat", "LHipYaw", "LKneeSag", "LAnkLat", "LAnkSag",
                                         "RShSag", "RShLat", "RShYaw", "RElbj",
                                         "LShSag", "LShLat", "LShYaw", "LElbj"};

void copy_ft_message(const bc_data_t* data, coman_msgs::ForceTorque& ft_msg)
{
    ft_msg.xForceNewtons = (float)data->ft_bc_data.fx/1000.0;
    ft_msg.yForceNewtons = (float)data->ft_bc_data.fy/1000.0;
    ft_msg.zForceNewtons = (float)data->ft_bc_data.fz/1000.0;

    ft_msg.xTorqueNewtonMeters = (float)data->ft_bc_data.tx/1000.0;
    ft_msg.yTorqueNewtonMeters = (float)data->ft_bc_data.ty/1000.0;
    ft_msg.zTorqueNewtonMeters = (float)data->ft_bc_data.tz/1000.0;
}

int main(int argc, char **argv)
{
    ROS_INFO("COMAN STATE PUBLISHER NODE");

    //**** INITIALIZATION OF ROS, MSGS AND PUBLISHERS ****
    ros::init(argc, argv, "coman_state_publisher");

    ros::NodeHandle n;

    sensor_msgs::JointState msg_joints;
    msg_joints.effort.resize(MAX_DSP_BOARDS);
    msg_joints.velocity.resize(MAX_DSP_BOARDS);
    msg_joints.position.resize(MAX_DSP_BOARDS);
    msg_joints.name.resize(MAX_DSP_BOARDS);

    coman_msgs::PID msg_joints_PID;
    msg_joints_PID.P.resize(MAX_DSP_BOARDS);
    msg_joints_PID.I.resize(MAX_DSP_BOARDS);
    msg_joints_PID.D.resize(MAX_DSP_BOARDS);
    msg_joints_PID.name.resize(MAX_DSP_BOARDS);

    coman_msgs::JointExtraInformation msg_joints_Extra;
    msg_joints_Extra.Current.resize(MAX_DSP_BOARDS);
    msg_joints_Extra.name.resize(MAX_DSP_BOARDS);


    for(unsigned int i = 0; i < joints_names.size(); ++i){
        msg_joints.name[i] = joints_names[i];
        msg_joints_PID.name[i] = joints_names[i];
        msg_joints_Extra.name[i] = joints_names[i];
    }

    coman_msgs::ForceTorque msg_forcetorque_left_foot, msg_forcetorque_right_foot;
    msg_forcetorque_left_foot.header.frame_id = "FTLeftFoot";
    msg_forcetorque_right_foot.header.frame_id = "FTRightFoot";

    ros::Publisher joints_pub = n.advertise<sensor_msgs::JointState>("coman/joints_state", 10);
    ros::Publisher PID_pub = n.advertise<coman_msgs::PID>("coman/joints_PID", 10);
    ros::Publisher joints_extra_pub = n.advertise<coman_msgs::JointExtraInformation>("coman/joints_extra_info", 10);
    ros::Publisher ft_left_pub = n.advertise<coman_msgs::ForceTorque>("coman/forcetorque_left_foot", 10);
    ros::Publisher ft_right_pub = n.advertise<coman_msgs::ForceTorque>("coman/forcetorque_right_foot", 10);


    ROS_INFO("OPENING xddp_sockets");

    //**** OPENING XDDP SOCKETS ***
    int xddp_sock = -1;
    int xddp_sock2 = -1;
    do{
#if __XENO__
        xddp_sock = open("/proc/xenomai/registry/rtipc/xddp/boards_bc_data", O_RDWR | O_NONBLOCK);
        xddp_sock2 = open("/proc/xenomai/registry/rtipc/xddp/boards_user_data", O_RDWR | O_NONBLOCK);
#else
        xddp_sock = open("/tmp/boards_bc_data", O_RDWR | O_NONBLOCK);
        xddp_sock2 = open("/tmp/boards_user_data", O_RDWR | O_NONBLOCK);
#endif
        if (xddp_sock < 0 || xddp_sock2 < 0) {
            ROS_ERROR("error in _init: %s\n", strerror (errno));
            assert(errno);
            ROS_INFO("Try to reconnect in 1 sec...");
            sleep(1);
            }
    }while(xddp_sock < 0 || xddp_sock2 < 0);
    if(xddp_sock > 0){
            ts_bc_data_t   bc_data[MAX_DSP_BOARDS];
            pid_gains_t    bc_pid_gains[MAX_DSP_BOARDS];
            ROS_INFO("xddp_sockets OPEN");

            ROS_INFO("START PUBLISHING DATA");
            //**** GETTING DATA AND PUBLISH ****
            ros::Rate loop_rate(freq);
            int nbytes = 0;
            bc_data_t * data = 0;
            pid_gains_t * data2 = 0;
            while(ros::ok())
            {
                do
                {
                    nbytes = read(xddp_sock, (void*)&bc_data, sizeof(bc_data));
                } while (nbytes == 0);
                if (nbytes < 0){
                    continue;}

                for (unsigned int i = 0; i < MAX_DSP_BOARDS; ++i) {
                    data = &bc_data[i].raw_bc_data;
                    if (data->bc_header._board_id > 0) {
                        // valid bId ...
                        switch ( data->bc_header._command ) {
                            case BCAST_MC_DATA_PACKETS : //Joint
                                msg_joints.position[i] = mRAD2RAD(data->mc_bc_data.Position);
                                msg_joints.velocity[i] = mRAD2RAD(data->mc_bc_data.Velocity);
                                msg_joints.effort[i] = data->mc_bc_data.Torque;
                                msg_joints_Extra.Current[i] = data->mc_bc_data.Current;
                                break;
                            case BCAST_FT_DATA_PACKETS : //FT Sensor
                                if(data->bc_header._board_id == 27)
                                    copy_ft_message(data, msg_forcetorque_left_foot); //id 27
                                else if(data->bc_header._board_id == 26)
                                    copy_ft_message(data, msg_forcetorque_right_foot); //id 26
                                break;
                            default:
                                ROS_INFO("dump_log unknown bc_data\n");
                                break;
                        }
                    }
                }

                do
                {
                    nbytes = read(xddp_sock2, (void*)&bc_pid_gains, sizeof(bc_pid_gains));
                } while (nbytes == 0);
                if (nbytes < 0){
                    continue;}

                for (unsigned int i = 0; i < MAX_DSP_BOARDS; ++i) {
                    data2 = &bc_pid_gains[i];
                    msg_joints_PID.P[i] = (double)data2->p * 1e-3; //Here we want Nm/rad when impedance control
                    msg_joints_PID.I[i] = (double)data2->i;
                    msg_joints_PID.D[i] = (double)data2->d;
                }

                //We had 90° of offset to shoulders roll
                msg_joints.position[20] += 90.0*M_PI/180.0;
                msg_joints.position[16] -= 90.0*M_PI/180.0;

                //**** SET THE SAME TIME STAMP TO ALL THE MESSAGES ****
                msg_joints.header.stamp = ros::Time::now();
                msg_joints_Extra.header.stamp = msg_joints.header.stamp;
                msg_forcetorque_left_foot.header.stamp = msg_joints.header.stamp;
                msg_forcetorque_right_foot.header.stamp = msg_joints.header.stamp;
                msg_joints_PID.header.stamp = msg_joints.header.stamp;

                //**** PUBLISH ****
                joints_pub.publish(msg_joints);
                joints_extra_pub.publish(msg_joints_Extra);
                ft_left_pub.publish(msg_forcetorque_left_foot);
                ft_right_pub.publish(msg_forcetorque_right_foot);
                PID_pub.publish(msg_joints_PID);


                ros::spinOnce();
                //loop_rate.sleep();
            }
    }

    close(xddp_sock);
    close(xddp_sock2);
    ROS_INFO("xddp socket closed");
    return 0;
}

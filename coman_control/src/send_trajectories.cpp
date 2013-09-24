#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <dynamic_reconfigure/server.h>
#include <coman_control/send_trajectoriesConfig.h>
#include <fstream>

#define NUMBER_OF_JOINTS 23 //<-- COMAN HAS 23 JOINTS!

using namespace std;

static const string name_space = "COMAN";
static const string joints_controller_names[23] = {
    "WaistSag_position_controller", "WaistLat_position_controller", "WaistYaw_position_controller",
    "RHipSag_position_controller", "RHipLat_position_controller", "RHipYaw_position_controller",
    "RKneeSag_position_controller", "RAnkLat_position_controller", "RAnkSag_position_controller",
    "LHipSag_position_controller", "LHipLat_position_controller", "LHipYaw_position_controller",
    "LKneeSag_position_controller", "LAnkLat_position_controller", "LAnkSag_position_controller",
    "RShSag_position_controller", "RShLat_position_controller", "RShYaw_position_controller",
    "RElbj_position_controller",
    "LShSag_position_controller", "LShLat_position_controller", "LShYaw_position_controller",
    "LElbj_position_controller"};
static const string comand = "command";

double freq = 1000.0;
bool start_trajectory = false;
int trajectory_lines = 0;
//We define all the messages needed for all the joints
vector<std_msgs::Float64> desired_joints_values;

void publication_rate_cb(coman_control::send_trajectoriesConfig& config, uint32_t level)
{
    freq = config.publish_frequency;
    start_trajectory = config.execute_trajectory;
    if(config.reset_joints)
    {
        for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i)
            desired_joints_values[i].data = 0.0;
    }
    if(config.reset_trajectory)
        trajectory_lines = 0;
    ROS_INFO("Requested new publication rate %f. This will be applied to all joints!", freq);
}

void copy_trj_to_joints(const string& line, vector<std_msgs::Float64>& desired_joint_values)
{
    float fs[30];
    std::istringstream(line) >> fs[0] >> fs[1] >> fs[2] >> fs[3] >> fs[4] >> fs[5] >> fs[6]
                             >> fs[7] >> fs[8] >> fs[9] >> fs[10] >> fs[11] >> fs[12] >> fs[13]
                             >> fs[14] >> fs[15] >> fs[16] >> fs[17] >> fs[18] >> fs[19] >> fs[20]
                             >> fs[21] >> fs[22] >> fs[23] >> fs[24] >> fs[25] >> fs[26] >> fs[27]
                             >> fs[28] >> fs[29];

//    for(unsigned int i = 0; i < 29; ++i)
//        cout<<fs[i]<<"  ";
//    cout<<endl;

//WAIST
    desired_joint_values[0].data = fs[19];
    desired_joint_values[1].data = fs[20];
    desired_joint_values[2].data = fs[21];
//RLEG
    desired_joint_values[3].data = fs[7];
    desired_joint_values[4].data = fs[8];
    desired_joint_values[5].data = fs[9];
    desired_joint_values[6].data = fs[10];
    desired_joint_values[7].data = fs[11];
    desired_joint_values[8].data = fs[12];
//LLEG
    desired_joint_values[9].data = fs[13];
    desired_joint_values[10].data = fs[14];
    desired_joint_values[11].data = fs[15];
    desired_joint_values[12].data = fs[16];
    desired_joint_values[13].data = fs[17];
    desired_joint_values[14].data = fs[18];
//RARM
    desired_joint_values[15].data = fs[22];
    desired_joint_values[16].data = fs[23];
    desired_joint_values[17].data = fs[24];
    desired_joint_values[18].data = fs[25];
//LARM
    desired_joint_values[19].data = fs[26];
    desired_joint_values[20].data = fs[27];
    desired_joint_values[21].data = fs[28];
    desired_joint_values[22].data = fs[29];
}

int main(int argc, char **argv)
{
    ROS_INFO("NODE TO SEND TRAJECTORIES...START!");

    //**** INITIALIZATION OF ROS, MSGS AND PUBLISHERS ****
    ros::init(argc, argv, "send_trajectories");

    ros::NodeHandle n;

    for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i){
        std_msgs::Float64 desired_joint_value;
        desired_joint_value.data = 0.0;
        desired_joints_values.push_back(desired_joint_value);}

    //We need 23 publishers also!
    vector<ros::Publisher> desired_joints_values_publishers;
    for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i){
        string topic = name_space + "/" + joints_controller_names[i] + "/" + comand;
        ros::Publisher desired_joints_values_publisher = n.advertise<std_msgs::Float64>(topic, 10);
        desired_joints_values_publishers.push_back(desired_joints_values_publisher);}

    //Dynamic reconfigure stuffs
    dynamic_reconfigure::Server<coman_control::send_trajectoriesConfig> server;
    dynamic_reconfigure::Server<coman_control::send_trajectoriesConfig>::CallbackType f;

    f = boost::bind(&publication_rate_cb, _1, _2);
    server.setCallback(f);

    ROS_INFO("Reading trajectory file");
    ifstream trj_file("trj.dat");
    vector<string> lines;
    while(!trj_file.eof())
    {
        string line;
        getline(trj_file,line);
        lines.push_back(line);
    }
    ROS_INFO("Trajectory has %i lines!", lines.size()-2);

    ros::Rate loop_rate(freq);
    while(ros::ok())
    {
        loop_rate = freq;

        if(start_trajectory)
        {
            if(trajectory_lines < lines.size()-2){
                copy_trj_to_joints(lines[trajectory_lines], desired_joints_values);
                trajectory_lines++;}
            else{
                trajectory_lines = 0;
                start_trajectory = false;
            }
        }

        for(unsigned int i = 0; i < NUMBER_OF_JOINTS; ++i)
            desired_joints_values_publishers[i].publish(desired_joints_values[i]);

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}

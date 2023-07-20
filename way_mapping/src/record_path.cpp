#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "tf/tfMessage.h"
#include "tf2_ros/transform_listener.h"
#include <math.h>
#include <fstream>
#include <ros/package.h>

std::string csv_path = ros::package::getPath("way_mapping") + "/map/simple_path.csv";
std::ofstream ofs;

bool start_record = false;
geometry_msgs::PoseStamped lastPose;

void configInit()
{
    ros::Time currentTime = ros::Time::now();
    lastPose.header.stamp = currentTime;
    lastPose.header.frame_id = "map";

    //Then assign value to "pose", which has member position and orientation
    lastPose.pose.position.x = 0.0;
    lastPose.pose.position.y = 0.0;
    lastPose.pose.position.z = 0.0;

    lastPose.pose.orientation.x = 0.0;
    lastPose.pose.orientation.y = 0.0;
    lastPose.pose.orientation.z = 0.0;
    lastPose.pose.orientation.w = 0.0;

    ofs.open(csv_path, std::ios::out);

}

void keyboardCallback(const std_msgs::String::ConstPtr& msg)
{
    std::string msg_data = msg->data.c_str();
    if(msg_data == "start_recording") {
        start_record = true;
        ROS_INFO("start recording...");
    }else if(msg_data == "stop_recording") {
        start_record = false;
        ROS_INFO("stop recording...");
    }
}

void recordTFPose(const geometry_msgs::TransformStamped &msg, const ros::Publisher &pose_pub) {
    double p_x, p_y, p_z, q_x, q_y, q_z, q_w, distance, theta;
    ros::Time msg_time = msg.header.stamp;

    p_x = msg.transform.translation.x;
    p_y = msg.transform.translation.y;
    p_z = msg.transform.translation.z;

    q_x = msg.transform.rotation.x;
    q_y = msg.transform.rotation.y;
    q_z = msg.transform.rotation.z;
    q_w = msg.transform.rotation.w;

    if(start_record) {
        distance = sqrt(pow(p_x-lastPose.pose.position.x, 2) + pow(p_y-lastPose.pose.position.y, 2));
        if(distance > 0.1) {
            ROS_INFO("Recording current point x: %f, y: %f, z: %f", p_x, p_y, p_z);

            // ROS_INFO("Recording the distance is: %f", distance);

            lastPose.header.stamp = msg_time;

            lastPose.pose.position.x = p_x;
            lastPose.pose.position.y = p_y;
            lastPose.pose.position.z = p_z;

            lastPose.pose.orientation.x = q_x;
            lastPose.pose.orientation.y = q_y;
            lastPose.pose.orientation.z = q_z;
            lastPose.pose.orientation.w = q_w;

            pose_pub.publish(lastPose);


            if(ofs) {
                ofs << std::to_string(p_x)+"," 
                    << std::to_string(p_y)+"," 
                    << std::to_string(p_z)+"," 
                    << std::to_string(q_x)+"," 
                    << std::to_string(q_y)+"," 
                    << std::to_string(q_z)+"," 
                    << std::to_string(q_w)+"\n";
            }else {
                ROS_ERROR("Open error: %s", csv_path.c_str());
            }

        }
        return;
    }

    lastPose.header.stamp = msg_time;

    lastPose.pose.position.x = p_x;
    lastPose.pose.position.y = p_y;
    lastPose.pose.position.z = p_z;

    lastPose.pose.orientation.x = q_x;
    lastPose.pose.orientation.y = q_y;
    lastPose.pose.orientation.z = q_z;
    lastPose.pose.orientation.w = q_w;

    return;

}

int main(int argc, char* argv[])
{
    setlocale(LC_ALL,"");
    ros::init(argc,argv,"record_path");
    ros::NodeHandle nh;
    configInit();

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;


    ros::Subscriber hmi_sub = nh.subscribe("/keyboard_interaction", 1000, keyboardCallback);
    ros::Publisher pose_pub = nh.advertise<geometry_msgs::PoseStamped>("/record_pose", 1000);

    while(ros::ok())
    {
        try{
            transformStamped = tfBuffer.lookupTransform("map", "base_link", ros::Time(0));
            recordTFPose(transformStamped, pose_pub);
        }
        catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }

        
        ros::spinOnce();
    }

    ofs.close();
    
    return 0;
}
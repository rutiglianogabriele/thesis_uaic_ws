#include <iostream>
#include <stdio.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>
#include "std_msgs/String.h"
using namespace std;

std::string timer_command;
int msgSent = 0;

class movePanda
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber subTimer;
    ros::Publisher pub;
    ros::Publisher pubTimer;

public:
    int task = 0;

    movePanda()
    {
        sub = nh.subscribe("/desired_3D_pos", 1000, &movePanda::msgCallback, this);
        pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
    }

    void msgCallback(const franka_msgs::FrankaState &msg)
    {
        // Wanted x,y,z pose
        float wantX;
        float wantY;
        float wantZ;

        switch (task)
        {
        case 0:
            // Default position
            wantX = 0.0;
            wantY = 0.60;
            wantZ = 0.60;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                ROS_INFO("Going to default position");
                task = 1;
            }
            break;
        case 1:
            // Pick Position
            wantX = 0.5;
            wantY = 0.20;
            wantZ = 0.25;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                ROS_INFO("Picking position. Waiting for timer.");
                task = 2;
            }
            break;
        case 2:
            if (msgSent == 0)
            {
                pub_timer("Start");
                msgSent = 1;
            }
            if (timer_command == "Go")
            {
                timer_command = "Wait";
                ROS_INFO("Going to hovering position.");
                task = 3;
            }
            break;
        case 3:
            // Hover position
            wantX = 0.60;
            wantY = 0.00;
            wantZ = 0.60;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                ROS_INFO("To hovering position. Goint to placing position.");
                task = 4;
            }
            break;
        case 4:
            // Place position
            wantX = 0.50;
            wantY = -0.20;
            wantZ = 0.25;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                ROS_INFO("To placing position. Waiting for timer.");
                msgSent = 0;
                task = 5;
            }
            break;
        case 5:
            if (msgSent == 0)
            {
                pub_timer("Start");
                msgSent = 1;
            }
            if (timer_command == "Go")
            {
                timer_command = "wait";
                ROS_INFO("Going to final position. All tasks finished.");
                task = 999;
            }
            break;
        default:
            pubPose(0.0, -0.6, 0.6, 0.0, 0.0, 1.0, 0.0);
        }
    }

    void pubPose(double px, double py, double pz, double ox, double oy, double oz, double ow)
    {
        // Initialization
        geometry_msgs::PoseWithCovarianceStamped msg;

        msg.pose.pose.position.x = px;
        msg.pose.pose.position.y = py;
        msg.pose.pose.position.z = pz;
        msg.pose.pose.orientation.x = ox;
        msg.pose.pose.orientation.y = oy;
        msg.pose.pose.orientation.z = oz;

        // ROS_INFO_STREAM("Sending random velocity command:"<<" linear="<<msg.pose.pose.position.x<<" angular="<<msg.pose.pose.orientation.x);

        // Publish Hardcoded pose
        pub.publish(msg);
    }

    bool isPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
    {
        float allowedError = 0.02;
        if ((currX < wantX + allowedError && currX > wantX - allowedError) && (currY < wantY + allowedError && currY > wantY - allowedError) && (currZ < wantZ + allowedError && currZ > wantZ - allowedError))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void timerCallback(const std_msgs::String &act)
    {
        timer_command = act.data;
    }

    void pub_timer(std::string action)
    {
        // Initialization
        std_msgs::String act;

        act.data = action;

        // Publish Hardcoded pose
        pubTimer.publish(act);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_panda_hardcoded");

    movePanda mp;

    ros::spin();
    return 0;
}
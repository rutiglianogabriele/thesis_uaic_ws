#include <iostream>
#include <stdio.h>
#include "ros/ros.h"
#include <ros/time.h>
#include <stdlib.h>
#include <franka_msgs/FrankaState.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sstream>
#include <chrono>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
using namespace std;

int msgSent = 0;
int setTimer = 0;
int log0, log1, log_2, log3, log4, log5, log6;

std::string object_id;

class movePanda
{
private:
    ros::NodeHandle nh;
    ros::Subscriber sub;
    ros::Subscriber sub_object;
    ros::Subscriber sub_status;
    ros::Subscriber sub_gripper_status;
    ros::Publisher pub;
    ros::Publisher pub_tracking_command;
    ros::Publisher pub_gripper_command;
    ros::Publisher pub_log;

public:
    // Wanted x,y,z object pose
    float wantX;
    float wantY;
    float wantZ;

    // Detection result
    std::string detection_status;
    std::string gripper_status;

    std::stringstream ss;

    double time_passed = 0.0;
    double placing_height;
    double interval_length;
    std::chrono::steady_clock::time_point start_time;
    double task_time;
    std::chrono::steady_clock::time_point end_time;
    double elapsed_time;

    int task = 0;

    movePanda()
    {
        sub = nh.subscribe("/franka_state_controller/franka_states", 1000, &movePanda::msgCallback, this);
        sub_object = nh.subscribe("/object_pose_w", 1000, &movePanda::objectPose, this);
        sub_status = nh.subscribe("/detection_status", 1000, &movePanda::detectionCallback, this);
        sub_gripper_status = nh.subscribe("/gripper_status", 1000, &movePanda::gripperCallback, this);
        pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
        pub_tracking_command = nh.advertise<std_msgs::Int32>("/start_detection", 20);
        pub_gripper_command = nh.advertise<std_msgs::Bool>("/gripper_close", 20);
        pub_log = nh.advertise<std_msgs::String>("/task_log", 20);
    }

    void msgCallback(const franka_msgs::FrankaState &msg)
    {

        // Current end effector coordinates
        float currX = msg.O_T_EE[12];
        float currY = msg.O_T_EE[13];
        float currZ = msg.O_T_EE[14];

        std_msgs::Int32 start_msg;
        start_msg.data = 100;

        std_msgs::Bool gripper_close;

        std_msgs::String log_msg;

        if (setTimer == 0)
        {
            // Logging time needed for each task
            start_time = std::chrono::steady_clock::now();

            setTimer = 1;
        }

        switch (task)
        {
        case 0:
            // Default position
            wantX = 0.5;
            wantY = 0.0;
            wantZ = 0.6;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                gripper_close.data = false;
                pub_gripper_command.publish(gripper_close);
                if (gripper_status == "Gripper Open!")
                {
                    task = 1;
                    log0 = 100;
                }
            }
            break;
        case 1:
            // Here we go to the pre-pick position where the robot is out of the fow of the camera
            if (log0 == 100)
            {
                // Evaluating time needed for each previous task and publishing it for futur records.
                end_time = std::chrono::steady_clock::now();
                elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;

                ss << "Elapsed time for goint to initial pose: " << elapsed_time << " seconds.";

                log_msg.data = ss.str();
                pub_log.publish(log_msg);

                // Clearing log msg for next log
                ss.str("");

                log0 = 0;
            }
            // Default position
            wantX = 0;
            wantY = 0.5;
            wantZ = 0.6;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                task = 2;
            }
            break;
        case 2:
            // When we have reached the pre-pick position we start the object detection by publishing a message to the camera node

            if (msgSent == 0)
            {
                end_time = std::chrono::steady_clock::now();
                ROS_INFO("Starting object detection");
                pub_tracking_command.publish(start_msg);
                msgSent = 1;
            }
            task = 3;
            break;
        case 3:
            // As soon as the camera has finished if we have an object we pick it up with case 4 otherwise we go back to case 1
            if (detection_status == "No objects detected!")
            {
                ROS_INFO("No object detected, going to default position");
                task = 1;
            }
            else if (detection_status == "Objects detected!")
            {
                ROS_INFO("Object detected, goint over the object to pick it up");
                task = 4;
                log_2 = 100;
            }
            break;
        case 4:
            // If we have detected an object we go over it to pick it
            if (log_2 == 100)
            {
                // Evaluating time needed for each previous task amd publishing it for futur records.
                task_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - end_time).count() / 1000.0;
                end_time = std::chrono::steady_clock::now();
                elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
                ss << "Elapsed total time: " << elapsed_time << " seconds. Decting the object needed: " << task_time << " seconds.";

                log_msg.data = ss.str();
                pub_log.publish(log_msg);

                // Clearing log msg for next log
                ss.str("");

                log_2 = 0;
            }
            // Going Over the object
            pubPose(wantX, wantY, wantZ + 0.15, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ + 0.15))
            {
                ROS_INFO("Going to pick the object");
                task = 5;
                log5 = 100;
            }
            break;
        case 5:
            // We send the command to the gripper node to close the gripper.
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            placing_height = wantZ;
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                if (log5 == 100)
                {
                    // Evaluating time needed for each previous task amd publishing it for futur records.
                    task_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - end_time).count() / 1000.0;
                    end_time = std::chrono::steady_clock::now();
                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
                    ss << "Elapsed total time: " << elapsed_time << " seconds. Going to picking position: " << task_time << " seconds.";

                    log_msg.data = ss.str();
                    pub_log.publish(log_msg);

                    // Clearing log msg for next log
                    ss.str("");

                    log5 = 0;
                }
                gripper_close.data = true;
                pub_gripper_command.publish(gripper_close);
            }

            // When we receive feedback from the gripper we continue with the rest of the task.
            if (gripper_status == "Gripper Closed!")
            {
                ROS_INFO("Object picked. Gripper closed. Going over the goal.");
                task = 7;
                log3 = 100;
            }
            break;
        case 7:
            // We go over the default placing position
            if (log3 == 100)
            {
                // Evaluating time needed for each previous task amd publishing it for futur records.
                task_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - end_time).count() / 1000.0;
                end_time = std::chrono::steady_clock::now();
                elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
                ss << "Elapsed total time: " << elapsed_time << " seconds. Grasping the object needed: " << task_time << " seconds.";

                log_msg.data = ss.str();
                pub_log.publish(log_msg);

                // Clearing log msg for next log
                ss.str("");

                log3 = 0;
            }
            // Going Over the object
            pubPose(wantX, wantY, wantZ + 0.15, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ + 0.15))
            {
                ROS_INFO("Going over goal position");
                task = 8;
            }
            break;
        case 8:
            // We go over the place position
            wantX = 0.4;
            wantY = 0.0;
            wantZ = 0.6;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                ROS_INFO("Placing the object");
                task = 9;
                log6 = 100;
            }
            break;
        case 9:
            // We go to the placing position to an height dictated by the height detected previously from the depth-canera
            wantX = 0.4;
            wantY = 0.0;
            wantZ = placing_height;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {
                if (log6 == 100)
                {
                    // Evaluating time needed for each previous task amd publishing it for futur records.
                    task_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - end_time).count() / 1000.0;
                    end_time = std::chrono::steady_clock::now();
                    elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
                    ss << "Elapsed total time: " << elapsed_time << " seconds. Going to placing position needed: " << task_time << " seconds.";

                    log_msg.data = ss.str();
                    pub_log.publish(log_msg);

                    // Clearing log msg for next log
                    ss.str("");

                    log6 = 0;
                }
                end_time = std::chrono::steady_clock::now();
                gripper_close.data = false;
                pub_gripper_command.publish(gripper_close);
            }
            if (gripper_status == "Gripper Open!")
            {
                task = 10;
                log4 = 100;
            }
            break;
        case 10:
            if (log4 == 100)
            {
                // Evaluating time needed for each previous task amd publishing it for futur records.
                task_time = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now() - end_time).count() / 1000.0;
                end_time = std::chrono::steady_clock::now();
                elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;
                ss << "Elapsed total time: " << elapsed_time << " seconds. Releasing the object to goal needed: " << task_time << " seconds.";

                log_msg.data = ss.str();
                pub_log.publish(log_msg);

                // Clearing log msg for next log
                ss.str("");

                log4 = 0;
            }
            wantX = 0.5;
            wantY = 0.0;
            wantZ = 0.6;
            pubPose(wantX, wantY, wantZ, 0.0, 0.0, 1.0, 0.0);
            if (isPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
            {

                ROS_INFO("Mission Finished!!!");
                task = 999;
            }
            break;
        default:
            pubPose(0.5, 0.0, 0.6, 0.0, 0.0, 1.0, 0.0);
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
        float allowedErrorX = 0.03;
        float allowedErrorY = 0.03;
        float allowedErrorZ = 0.06;
        if ((currX < wantX + allowedErrorX && currX > wantX - allowedErrorX) && (currY < wantY + allowedErrorY && currY > wantY - allowedErrorY) && (currZ < wantZ + allowedErrorZ && currZ > wantZ - allowedErrorZ))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void detectionCallback(const std_msgs::String &act)
    {
        detection_status = act.data;
    }

    void gripperCallback(const std_msgs::String &act)
    {
        gripper_status = act.data;
    }

    void objectPose(const geometry_msgs::PoseWithCovarianceStamped &object_pose)
    {
        wantX = object_pose.pose.pose.position.x;
        wantY = object_pose.pose.pose.position.y;
        wantZ = object_pose.pose.pose.position.z;

        std::cout << "POSITION IS X: " << wantX << " Y: " << wantY << " Z: " << wantZ;
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "move_panda_hardcoded");

    movePanda mp;

    ros::spin();
    return 0;
}
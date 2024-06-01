#include <memory>
#include <string>
#include <vector>
#include <ros/ros.h>
#include <std_msgs/Int64.h>
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "std_msgs/Bool.h"
#include <std_srvs/SetBool.h>
#include <chrono>
#include <sstream>
#include <std_msgs/Float64MultiArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <franka_msgs/FrankaState.h>
#include <predictor/PredictedPoses.h>
#include <string>
#include <iostream>
#include <sstream>

class PredictionHandler
{
private:
  ros::NodeHandle nh;

  // Ros publishers
  ros::Publisher pub;
  ros::Publisher pub_gripper_command;
  ros::Publisher pub_log;

  // Ros subscribers
  ros::Subscriber sub;
  ros::Subscriber sub_gripper_status;
  ros::Subscriber subPredictor;

  // Stuff needed by the predictor
  predictor::PredictedPoses predicted_poses;
  geometry_msgs::PoseWithCovarianceStamped single_target_pose;
  std::stringstream buffer;        // My debugging buffer string for printing
  int target_prediction_step = 20; // The prediction step which should be passed onto the inverse kinematics solver
  int target_prediction_order = 0; // 0 to get position prediction, 1 to get velocity prediction. This parameter will probably not be used when a target selector using both the position and velocity prediction is implemented.
  const static int horizon = 60;   // Prediction horizon
  const static int pred_order = 2; // Prediction order                             // position and velocity, for now. In future acceleration and jerk may also be predicted.

public:
  // Saves the status of the gripper. It is used by the gripper callback.
  std::string gripper_status;

  std::stringstream ss;

  // Condition to enter the last phase of the mission, to place the object to goal.
  bool object_grasped;

  // Condition not to pick the same object twice
  float last_picked_y = -0.6;

  // Various set of coordinates
  float currX, currY, currZ; // These will keep track of the current x,y,z pose of the end effector. Needed to check if we reached the objective.
  float defX, defY, defZ;    // These encapsulate the pose of the default pose, the one where the robot hovers waiting to pick the object, out of the detection area of the camera
  float goalX, goalY, goalZ; // these encapsulate the pose of the goal position, where the object will be placed
  float wantX, wantY, wantZ; // they keep track of the predicted pose of the object

  // Needed for the switch case of the mission
  int task = 0;

  // Needed for task logging
  int setTimer = 0;
  int log0, log1;
  std::chrono::steady_clock::time_point start_time, end_time;
  double elapsed_time, task_time;

  PredictionHandler()
  {
    // Define subscribers
    sub = nh.subscribe("/franka_state_controller/franka_states", 1000, &PredictionHandler::frankaCallback, this);
    subPredictor = nh.subscribe("/predicted_poses", 1000, &PredictionHandler::msgCallback, this);
    sub_gripper_status = nh.subscribe("/gripper_status", 1000, &PredictionHandler::gripperCallback, this);

    // Define publishers
    pub = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/desktop/target_pose_cov_stmp_panda_link0", 10);
    pub_gripper_command = nh.advertise<std_msgs::Bool>("/gripper_close", 20);
    pub_log = nh.advertise<std_msgs::String>("/task_log", 20);

    // Needed for the predictor
    predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 1.
    predicted_poses.layout.dim.push_back(std_msgs::MultiArrayDimension()); // Increase dimension of predicted pose matrix by 1, to 2.
    predicted_poses.layout.dim[0].label = "rows, prediction step";
    predicted_poses.layout.dim[0].size = horizon;
    predicted_poses.layout.dim[0].stride = horizon * pred_order;
    predicted_poses.layout.dim[1].label = "columns, prediction order"; // xp,yp,zp,xv,yv,zv
    predicted_poses.layout.dim[1].size = pred_order;
    predicted_poses.layout.dim[1].stride = pred_order;
    predicted_poses.poses.push_back(geometry_msgs::PoseWithCovariance()); // This line has a compilation error. Do I even need to "push_back" as this array?
    predicted_poses.poses.resize(pred_order * horizon);
  }

  void frankaCallback(const franka_msgs::FrankaState &msg)
  {
    // Initiate messaged for the gripper and for task logging
    std_msgs::Bool gripper_close;
    std_msgs::String log_msg;

    // Current end effector coordinates
    currX = msg.O_T_EE[12];
    currY = msg.O_T_EE[13];
    currZ = msg.O_T_EE[14];

    // If we have grasped the object then we finish the mission with this loop
    if (object_grasped == true)
    {
      // Condition to skip all the other commands
      wantY = -0.1;

      // Sending Robot to Goal
      goalX = 0.35;
      goalY = 0.01;
      goalZ = 0.27;

      switch (task)
      {
      // With this case we go over the goal position
      case 0:
        pubPose(goalX, goalY, goalZ + 0.15, 0, 0, 1, 0);
        if (isPoseReached(currX, currY, currZ, goalX, goalY, goalZ + 0.15))
        {
          task = 1;
        }
        break;

      // With this case we go to the goal position
      case 1:
        pubPose(goalX, goalY, goalZ, 0, 0, 1, 0);
        if (isPoseReached(currX, currY, currZ, goalX, goalY, goalZ))
        {
          // We have reached the goal so we open the gripper to release the object
          gripper_close.data = false;
          pub_gripper_command.publish(gripper_close);

          // Since we have closed the gripper we can evaluate the time needed to go from picking position to placing position
          if (log1 == 100)
          {
            end_time = std::chrono::steady_clock::now();
            elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;

            ss << "Elapsed time to place the object to goal: " << elapsed_time << " seconds.";
            log_msg.data = ss.str();
            pub_log.publish(log_msg);

            // Resetting variabled used as logging conditions
            log0 = 0;
            log1 = 100;

            // Clearing log msg for next log
            ss.str("");
          }

          task = 2;
        }
        break;

      // With this case we go over the goal position
      case 2:
        pubPose(goalX, goalY, goalZ + 0.15, 0, 0, 1, 0);
        if (isPoseReached(currX, currY, currZ, goalX, goalY, goalZ + 0.15))
        {
          task = 3;
        }
        break;

      // With this case we go back to default position
      case 3:
        defX = 0.5;
        defY = 0.0;
        defZ = 0.7;

        pubPose(defX, defY, defZ, 0, 0, 1, 0);
        if (isPoseReached(currX, currY, currZ, defX, defY, defZ))
        {
          object_grasped = false;
          ROS_INFO("ARRIVED TO DEFAULT POSE");

          // Reset the timer for next object to be grasped
          setTimer = 0;
          task = 999;
        }
        break;

      default:
        task = 0;
        break;
      }
    }

    // If the target is further than 60cm away from the base of the robot arm in the
    // xy plane use instead a default target position untill the target gets within range
    if (std::sqrt(std::pow(wantX, 2) + std::pow(wantY, 2)) > 0.8 || wantY > 0.0 || wantY < -0.4)
    {
      defX = 0.5;
      defY = 0.0;
      defZ = 0.6;

      // Opening Gripper
      gripper_close.data = false;
      object_grasped = false;
      pub_gripper_command.publish(gripper_close);

      setTimer = 100;

      // Publishing pose to Controller
      pubPose(defX, defY, defZ, 0, 0, 1, 0);
    }
    // If the target is within the grasping area, between -0.4 < y < 0 then we proceed grasping the object.
    else if (wantY > -0.4 && wantY <= 0 && object_grasped == false)
    {

      pubPose(wantX, wantY, wantZ, 0, 0, 1, 0);

      // We start the timer to count the time needed to pick the object since it has entered the grasping zone
      if (setTimer == 100)
      {
        // Logging time needed for each task
        start_time = std::chrono::steady_clock::now();
        log0 = 100;
        setTimer = 1;
      }
      // If we correctly reached the object then we can close the gripper and move on with the rest of the mission
      if (isMovingPoseReached(currX, currY, currZ, wantX, wantY, wantZ))
      {
        // We have reached the object so we close the gripper to pick it up
        gripper_close.data = true;
        pub_gripper_command.publish(gripper_close);

        // We save current y to make sure not the pick the same object twice
        last_picked_y = currY;

        // We save the state of the grasping phase as condition to move on onto the other phases of the mission
        object_grasped = true;

        // Since we have closed the gripper we can count the time needed to pick it since it has entered the grasping zone
        if (log0 == 100)
        {
          end_time = std::chrono::steady_clock::now();
          elapsed_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.0;

          // Reset timer to count the time needed for the next step
          start_time = std::chrono::steady_clock::now();

          // Create a string to publish
          ss << "Elapsed time to grasp the object: " << elapsed_time << " seconds.";
          log_msg.data = ss.str();
          pub_log.publish(log_msg);

          // Reset the log condition
          log0 = 0;
          log1 = 100;

          // Clearing log msg for next log
          ss.str("");
        }
      }
    }
  }

  bool isMovingPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
  {
    float allowedErrorX = 0.03;
    float allowedErrorY = 0.07;
    float allowedErrorZ = 0.05;
    if ((currX < wantX + allowedErrorX && currX > wantX - allowedErrorX) && (currY < wantY + allowedErrorY && currY > wantY - allowedErrorY) && (currZ < wantZ + allowedErrorZ && currZ > wantZ - allowedErrorZ))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool isPoseReached(float currX, float currY, float currZ, float wantX, float wantY, float wantZ)
  {
    float allowedErrorX = 0.03;
    float allowedErrorY = 0.03;
    float allowedErrorZ = 0.04;
    if ((currX < wantX + allowedErrorX && currX > wantX - allowedErrorX) && (currY < wantY + allowedErrorY && currY > wantY - allowedErrorY) && (currZ < wantZ + allowedErrorZ && currZ > wantZ - allowedErrorZ))
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  void msgCallback(const predictor::PredictedPoses &msg)
  {
    single_target_pose.header = msg.header;
    wantX = msg.poses[target_prediction_step * pred_order + target_prediction_order].pose.position.x;
    wantY = msg.poses[target_prediction_step * pred_order + target_prediction_order].pose.position.y;
    wantZ = msg.poses[target_prediction_step * pred_order + target_prediction_order].pose.position.z;
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
    msg.pose.pose.orientation.w = ow;

    // Publish Hardcoded pose
    pub.publish(msg);
  }

  void gripperCallback(const std_msgs::String &act)
  {
    gripper_status = act.data;
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "prediction_handler");
  PredictionHandler ph;
  ros::spin();
  return 0;
}
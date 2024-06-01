// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE

/*
 * Modified version for active inference by Gabriele Rutigliano
 *
 * Author: Corrado Pezzato
 * Date 09-09-2019
 *
 * This script implements an active inference controller for the Panda Franka
 * Emika through the ROS framework.
 *
 * Unbiased AIC Task Space
 */

#include <franka_aic/AIC_controller.h>
#include <controller_interface/controller_base.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <franka/robot_state.h>
#include <franka/gripper.h>
#include <franka/robot.h>
#include <sstream>
#include <vector>
#include <iterator>
#include <array>
#include <cmath>
#include <memory>
#include <string>

#include <hardware_interface/hardware_interface.h>

#include <tf/transform_listener.h>

namespace franka_aic
{
  // It's a problem that the function is static, seen as if multiple instances of the
  // AICController class is made, the callback function might update something for
  // all the other instances of the class
  // as long as there are no static variables I think there isn't a problem, I hope!

  bool AICController::init(hardware_interface::RobotHW *robot_hw, ros::NodeHandle &node_handle)
  {
    std::vector<std::string> joint_names; // TODO: Why is definition not in .h file?
    std::string arm_id;                   // TODO: Why is definition not in .h file?

    // HIRO Subscriber stuff
    _target_sub_task_space = node_handle.subscribe("/desktop/target_pose_cov_stmp_panda_link0", 10, &AICController::targetCartesianPoseCb, this,
                                                   ros::TransportHints().reliable().tcpNoDelay());

    // // get parameters and trajectory method selection
    int traj_method_int = 1;
    node_handle.getParam("/hiro_panda/trajectory_method", traj_method_int);
    traj_method = (TrajectoryMethod)traj_method_int;

    // Publishers
    pub_limited_j_cmds = node_handle.advertise<sensor_msgs::JointState>("aic_limited_j_cmds", 10);
    pub_j_cmds = node_handle.advertise<sensor_msgs::JointState>("aic_j_cmds", 10);
    pub_j_f_cmds = node_handle.advertise<sensor_msgs::JointState>("aic_j_f_cmds", 10);
    pub_mu = node_handle.advertise<sensor_msgs::JointState>("aic_theta_hat_star", 10);
    pub_mu_dot = node_handle.advertise<sensor_msgs::JointState>("aic_theta_dot_hat_star", 10);
    pub_mu_d = node_handle.advertise<sensor_msgs::JointState>("aic_theta_r", 10);
    pub_u = node_handle.advertise<sensor_msgs::JointState>("aic_u_0", 10);
    pub_seq = 0;

    _limited_j_cmds.header.seq = pub_seq;
    _limited_j_cmds.header.frame_id = "joint_space";
    _limited_j_cmds.name.resize(7);
    _limited_j_cmds.position.resize(7);
    _limited_j_cmds.velocity.resize(7);
    _limited_j_cmds.effort.resize(7);

    _j_cmds.header.seq = pub_seq;
    _j_cmds.header.frame_id = "joint_space";
    _j_cmds.name.resize(7);
    _j_cmds.position.resize(7);
    _j_cmds.velocity.resize(7);
    _j_cmds.effort.resize(7);

    _mu.header.seq = pub_seq;
    _mu.header.frame_id = "joint_space";
    _mu.name.resize(7);
    _mu.position.resize(7);
    _mu.velocity.resize(7);
    _mu.effort.resize(7);

    _mu_dot.header.seq = pub_seq;
    _mu_dot.header.frame_id = "joint_space";
    _mu_dot.name.resize(7);
    _mu_dot.position.resize(7);
    _mu_dot.velocity.resize(7);
    _mu_dot.effort.resize(7);

    _mu_d.header.seq = pub_seq;
    _mu_d.header.frame_id = "joint_space";
    _mu_d.name.resize(7);
    _mu_d.position.resize(7);
    _mu_d.velocity.resize(7);
    _mu_d.effort.resize(7);

    _u.header.seq = pub_seq;
    _u.header.frame_id = "joint_space";
    _u.name.resize(7);
    _u.position.resize(7);
    _u.velocity.resize(7);
    _u.effort.resize(7);

    _limited_j_cmds.name = jointNames;
    _j_cmds.name = jointNames;
    _mu.name = jointNames;
    _mu_dot.name = jointNames;
    _mu_d.name = jointNames;
    _u.name = jointNames;

    ROS_WARN("UNBIASED AICController - Task Space: Ready to start the magic?! :)");

    if (!node_handle.getParam("arm_id", arm_id))
    {
      ROS_ERROR("UNBIASED AICController: Could not read parameter arm_id");
      return false;
    }

    if (!node_handle.getParam("joint_names", joint_names) || joint_names.size() != 7)
    {
      ROS_ERROR("UNBIASED AICController: Invalid or no joint_names parameters provided, aborting controller init!");
      return false;
    }

    // joint names check
    if (!node_handle.getParam("joint_names", joint_names))
    {
      ROS_ERROR("PandaPoseController: Could not parse joint names");
      return false;
    }

    if (joint_names.size() != 7)
    {
      ROS_ERROR_STREAM("PandaPoseController: Wrong number of joint names, got " << joint_names.size() << " instead of 7 names!");
      return false;
    }

    //  Initialize hardware interface:
    franka_hw::FrankaModelInterface *model_interface = robot_hw->get<franka_hw::FrankaModelInterface>();
    if (model_interface == nullptr)
    {
      ROS_ERROR_STREAM("AICController: Error getting model interface from hardware");
      return false;
    }
    try
    {
      model_handle_.reset(
          new franka_hw::FrankaModelHandle(model_interface->getHandle(arm_id + "_model")));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "AICController: Exception getting model handle from interface: " << ex.what());
      return false;
    }

    franka_hw::FrankaStateInterface *state_interface =
        robot_hw->get<franka_hw::FrankaStateInterface>();
    if (state_interface == nullptr)
    {
      ROS_ERROR_STREAM("AICController: Error getting state interface from hardware");
      return false;
    }
    try
    {
      state_handle_.reset(
          new franka_hw::FrankaStateHandle(state_interface->getHandle(arm_id + "_robot")));
    }
    catch (hardware_interface::HardwareInterfaceException &ex)
    {
      ROS_ERROR_STREAM(
          "AICController: Exception getting state handle from interface: " << ex.what());
      return false;
    }

    hardware_interface::EffortJointInterface *effort_joint_interface =
        robot_hw->get<hardware_interface::EffortJointInterface>();
    if (effort_joint_interface == nullptr)
    {
      ROS_ERROR_STREAM("AICController: Error getting effort joint interface from hardware");
      return false;
    }
    for (size_t i = 0; i < 7; ++i)
    { // NOTE: HIRO uses position joint interface, named _position_joint_handles. This pushback call corresponds to their getHandle() call on lines 61 to 75 in panda_pose_controller.cpp
      try
      {
        joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
      }
      catch (const hardware_interface::HardwareInterfaceException &ex)
      {
        ROS_ERROR_STREAM("AICController: Exception getting joint handles: " << ex.what());
        return false;
      }
    }

    // hiro panda trac ik service
    _panda_ik_service = franka_aic::PandaTracIK();
    // Does the fact that PandaTracIK is part of the same namespace make a difference?
    //
    _is_executing_cmd = false;

    return true;
  }

  void AICController::starting(const ros::Time & /*time*/)
  {
    ROS_INFO("Running function: starting");

    // Initialise the controller's parameters
    //---------------------------------------------------------------
    franka::RobotState robot_state = state_handle_->getRobotState();

    mu_d << 1.0, 0.0, 0.0, -1.57, 0.0, 1.57, -1.57; // Center
    // mu_d << -1.57, 0.0, -0.0, -1.57, 0.0, 1.57, -1.57; // Right
    // mu_d << 1.57, 0.0, -0.0, -1.57, 0.0, 1.57, -1.57; //Left
    // mu_d << 0.75, 0.35, -0.4, -2.4, 0.25, 2.75, 0.90; // Picking something

    // Integration step
    h = 0.001; // gru_seen

    // Precision matrices (first set them to zero then populate the diagonal)
    SigmaP_yq0 = Eigen::Matrix<double, 7, 7>::Zero();     // gru_seen
    SigmaP_yq1 = Eigen::Matrix<double, 7, 7>::Zero();     // gru_seen
    SigmaP_mu = Eigen::Matrix<double, 7, 7>::Zero();      // gru_seen
    SigmaP_muprime = Eigen::Matrix<double, 7, 7>::Zero(); // gru_seen
    K_p = Eigen::Matrix<double, 7, 7>::Zero();            // gru_update
    K_d = Eigen::Matrix<double, 7, 7>::Zero();            // gru_update
    K_i = Eigen::Matrix<double, 7, 7>::Zero();            // gru_update

    // Initialize the vector for the low pass filter
    xPrev = Eigen::Matrix<double, 7, 1>::Zero(); // Inizializzazione di xPrev, previous input
    yPrev = Eigen::Matrix<double, 7, 1>::Zero(); // Inizializzazione di yPrev, previous output
    xn = Eigen::Matrix<double, 7, 1>::Zero();    // Inizializzazione di yn, output corrente
    yn = Eigen::Matrix<double, 7, 1>::Zero();    // Inizializzazione di yn, output corrente

    // Evaluated for a cutoff of 0.5Hz and Samplig of 997.2Hz
    alpha = 0.9969;
    beta = 0.001573;

    // Finished intiliazing the controller's parameters
    //---------------------------------------------------------------

    // Begin Tuning parameters of u-AIC
    //---------------------------------------------------------------
    // Variances associated with the beliefs and the sensory inputs
    var_mu = 10.0;      // gru_update
    var_muprime = 20.0; // gru_update
    var_q = 1;          // gru_seen
    var_qdot = 1;       // gru_seen

    k_p0 = 55;  // gru_update
    k_p1 = 35;  // gru_update
    k_p2 = 35;  // gru_update
    k_p3 = 15;  // gru_update
    k_p4 = 20;  // gru_update
    k_p5 = 12;  // gru_update
    k_p6 = 1.5; // gru_update
    k_d = 30;   // gru_update
    k_i = 0.01; // gru_update
    max_i = 200;
    I_gain << 0.02, 0.02, 0.02, 0.02, 0.02, 0.02, 0.02; // gru_update

    // Learning rates for the gradient descent
    k_a = 200;
    k_mu = 51.6; // gru_update

    // End tuning parameters
    //---------------------------------------------------------------

    // Initialize Angular Position Reference returned from IK:
    _joints_result.resize(7); // gru_seen

    // Populating matrices
    //---------------------------------------------------------------
    for (int i = 0; i < SigmaP_yq0.rows(); i = i + 1)
    {
      SigmaP_yq0(i, i) = 1 * 1 / var_q;       // gru_seen
      SigmaP_yq1(i, i) = 1 * 1 / var_qdot;    // gru_seen
      SigmaP_mu(i, i) = 1 / var_mu;           // gru_seen
      SigmaP_muprime(i, i) = 1 / var_muprime; // gru_seen

      K_d(i, i) = k_d; // gru_update
      K_i(i, i) = k_i; // gru_update

      // Internal belief starting from initial pose
      jointPos(i) = robot_state.q[i];      // gru_seen   robot_state.q --> msg->position in the uAIC
      jointVel(i) = robot_state.dq[i];     // gru_seen   robot_state.dq --> msg->velocity in the uAIC
      _joints_result(i) = jointPos(i);     // Initialize angular position reference | _position_joint_handles[i].getPosition();
      jointPosPrev(i) = _joints_result(i); // Initialize previous angular position reference
      _iters[i] = 0;                       /// Only used in TrapezoidVel trajectory method
    }

    // Re-tunin of last joints to have less jitter, another equivalent approach is to have a vector for k_a
    SigmaP_yq0(4, 4) = 0.1 * 1 / var_q;    // gru_seen
    SigmaP_yq1(4, 4) = 0.1 * 1 / var_qdot; // gru_seen

    SigmaP_yq0(5, 5) = 0.03 * 1 / var_q;    // gru_seen
    SigmaP_yq1(5, 5) = 0.03 * 1 / var_qdot; // gru_seen

    SigmaP_yq0(6, 6) = 0.01 * 1 / var_q;    // gru_seen
    SigmaP_yq1(6, 6) = 0.01 * 1 / var_qdot; // gru_seen

    // Single proportional    //gru_seen ALL of them
    K_p(0, 0) = k_p0;
    K_p(1, 1) = k_p1;
    K_p(2, 2) = k_p2;
    K_p(3, 3) = k_p3;
    K_p(4, 4) = k_p4;
    K_p(5, 5) = k_p5;
    K_p(6, 6) = k_p6;
    K_d(5, 5) = 10;
    K_i(3, 3) = 0.03;
    K_i(6, 6) = 0.03;
    //---------------------------------------------------------------

    // Initial belief
    mu = jointPos;   // gru_seen
    mu_p = jointVel; // gru_seen
    mu_past = mu;
    mu_p_past = mu_p;

    // Initial desired velocity
    mu_p_d << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // gru_seen

    // Initial control actions
    u << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; // gru_seen

    // Set initial time
    time_passed = 0.0;
  }

  void AICController::update(const ros::Time &time, const ros::Duration &period)
  {
    franka::RobotState robot_state = state_handle_->getRobotState();
    interval_length = period.toSec();
    time_passed += interval_length;

    // Save the robot state for algebric manipulation
    for (int i = 0; i < 7; i = i + 1)
    {
      // Set current sensory input
      jointPos(i) = robot_state.q[i];  // Measured joint position
      jointVel(i) = robot_state.dq[i]; // Measured joint velocity
    }

    //  Get most recent desired angular position from inverse kinematics
    for (int i = 0; i < 7; i++)
    {
      xn(i) = _joints_result(i);
    }

    // Compute the filtered signal
    for (int i = 0; i < 7; i++)
    {
      yn[i] = alpha * yPrev[i] + beta * xn[i] + beta * xPrev[i];

      xPrev[i] = xn[i];
      yPrev[i] = yn[i];
      mu_d(i) = yn[i];
    }

    /////////// unbiased AIC ///////////
    // Constant Position Reference State Estimator
    // Belief update
    mu_dot = -k_mu * (-SigmaP_yq0 * (jointPos - mu) + SigmaP_mu * (mu - (mu_past + h * mu_p_past))); // gru_update
    mu_dot_p = -k_mu * (-SigmaP_yq1 * (jointVel - mu_p) + SigmaP_muprime * (mu_p - mu_p_past));      // gru_update

    // Save current value of the belief to use in the next iteration as previous value
    mu_past = mu;     // gru_update
    mu_p_past = mu_p; // gru_update

    // Belifs update
    mu = mu + h * mu_dot;       // Belief about the position  //gru_seen
    mu_p = mu_p + h * mu_dot_p; // Belief about motion of mu  //gru_seen

    // Set curret values for next ieration
    I_gain = I_gain + mu_d - mu; // gru_update

    // Satruration of integral action
    for (int j = 0; j < 7; j++) // gru_update
    {
      if (I_gain(j) > max_i)
      {
        I_gain(j) = max_i;
      }
      if (I_gain(j) < -max_i)
      {
        I_gain(j) = -max_i;
      }
    }

    // Compute control actions
    // Unbiased uAIC
    u = K_p * (mu_d - mu) + K_d * (mu_p_d - mu_p) + K_i * (I_gain); // gru_update

    // Set the toques from u and publish
    for (size_t i = 0; i < 7; ++i)
    {
      joint_handles_[i].setCommand(u(i));
    }

    _limited_j_cmds.header.stamp = time;
    _limited_j_cmds.header.seq = pub_seq;
    _limited_j_cmds.position = std::vector<double>(_limited_joint_cmds.begin(), _limited_joint_cmds.end());

    _j_cmds.header.stamp = time;
    _j_cmds.header.seq = pub_seq;
    _j_cmds.position = std::vector<double>(_joint_cmds.begin(), _joint_cmds.end());

    _mu_d.header.stamp = time;
    _mu_d.header.seq = pub_seq;
    Eigen::VectorXd::Map(&_mu_d.position[0], mu_d.size()) = mu_d;

    _mu.header.stamp = time;
    _mu.header.seq = pub_seq;
    Eigen::VectorXd::Map(&_mu.position[0], mu.size()) = mu;
    Eigen::VectorXd::Map(&_mu.velocity[0], mu_p.size()) = mu_p;

    _mu_dot.header.stamp = time;
    _mu_dot.header.seq = pub_seq;
    Eigen::VectorXd::Map(&_mu_dot.position[0], mu.size()) = mu;
    Eigen::VectorXd::Map(&_mu_dot.velocity[0], mu_p.size()) = mu_p;

    _u.header.stamp = time;
    _u.header.seq = pub_seq;
    Eigen::VectorXd::Map(&_u.position[0], u.size()) = u;

    pub_limited_j_cmds.publish(_limited_j_cmds);
    pub_j_cmds.publish(_j_cmds);
    pub_mu.publish(_mu);
    pub_mu_dot.publish(_mu_dot);
    pub_mu_d.publish(_mu_d);
    pub_u.publish(_u);

    jointPosPrev = jointPos;
    ++pub_seq;
  }

  void AICController::stopping(const ros::Time &time)
  {
    // can't send immediate commands for 0 velocity to robot
  }

  void AICController::ref_callback(const std_msgs::Float32MultiArray::ConstPtr &ref)
  {
    for (int i = 0; i < 7; i++)
    {
      ref_p(i, 0) = ref->data[i];
      // ROS_INFO("%d: %s in callback",i,std::to_string(ref_p(i,0)).c_str());
    }
  }

  // void AICController::targetCartesianPoseCb(const geometry_msgs::PoseWithCovarianceStamped &target_pose)
  // {
  //   franka::RobotState robot_state = state_handle_->getRobotState();

  //   std::array<double, 7> currentPos;

  //   _target_pose = target_pose.pose.pose;

  //   double _x = _target_pose.position.x;
  //   double _y = _target_pose.position.y;
  //   double _z = _target_pose.position.z;

  //   std::array<double, 16> desired_pose_ht = {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, _x, _y, _z, 1};

  //   for (int i = 0; i < 7; i = i + 1)
  //   {
  //     // Set current sensory input
  //     _jointPos(i) = robot_state.q[i]; // Parsing values to Eigen::Matrix
  //     currentPos[i] = _jointPos(i);    // goint from Eigen::Matrix to std::array
  //   }

  //   _resultIK = franka_IK_EE_CC(desired_pose_ht, -1.57, currentPos);

  //   for (int i = 0; i < 7; i = i + 1)
  //   {
  //     // Set current sensory input
  //     _joints_result(i) = _resultIK[i];
  //   }
  // }

  void AICController::targetCartesianPoseCb(const geometry_msgs::PoseWithCovarianceStamped &target_pose)
  {
    _target_pose = target_pose.pose.pose;

    // Get the original orientation of 'commanded_pose'
    tf2::convert(_target_pose.orientation, q_orig);

    double r = 0, p = 3.14159, y = 0; // Rotate the previous pose by 180* about X
    q_rot.setRPY(r, p, y);

    q_new = q_rot * q_orig; // Calculate the new orientation
    q_new.normalize();

    // Stuff the new rotation back into the pose. This requires conversion into a msg type
    tf2::convert(q_new, _target_pose.orientation);
    _target_pose.orientation.w = q_new.getW();
    _target_pose.orientation.x = q_new.getX();
    _target_pose.orientation.y = q_new.getY();
    _target_pose.orientation.z = q_new.getZ();

    ik_result = _panda_ik_service.perform_ik(_target_pose);

    if (_panda_ik_service.is_valid)
    {
      _joints_result = ik_result;
    }
    else
    {
      _joints_result = _joints_result;
    }
    ROS_INFO("_joints_result: %d", _panda_ik_service.is_valid);
    if (_joints_result.rows() != 7)
    {
      ROS_ERROR("Panda Pose Controller: Wrong Amount of Rows Received From TRACIK");
      return;
    }
    _is_executing_cmd = true;
    for (int i = 0; i < 7; i++)
    {
      _iters[i] = 0;
    }
  }

  bool AICController::_isGoalReached()
  {
    // sees if goal is reached given a distance threshold epsilon
    // l2 norm is used for overall distance
    err = 0;
    for (int i = 0; i < 7; i++)
    {
      err += pow(joint_handles_[i].getPosition() - _joints_result(i), 2);
    }
    err = sqrt(err);
    //    ROS_INFO("_epsilon: %f, err: %f, Goal reached: %d",_epsilon,err,err <= _epsilon); // TODO: Delete when done debugging

    return err <= _epsilon;
  }

  void AICController::trapezoidVelCmd(const double &max_step, const double &v_change)
  {
    //    ROS_INFO("trapezoidVelCmd");
    // generates waypoints that increase velocity, then decrease
    step_time = max_step / v_change;
    // time to go from 0 to min_step for each time step
    change_dist = step_time * max_step * 0.5;

    for (int i = 0; i < 7; i++)
    {
      double diff = _joint_cmds[i] - joint_handles_[i].getPosition();

      if (_iters[i] == 0 && abs(diff) <= change_dist)
      {
        case_used_in_trapezoid_vel = 1; // TODO: Delete when done debugging
        // case where we have new trajectory, but it doesn't need the whole path
        _limited_joint_cmds[i] = _joint_cmds[i];
      }

      else if (abs(diff) <= change_dist)
      {
        case_used_in_trapezoid_vel = 2; // TODO: Delete when done debugging
        // when distance is small enough, decrease _iter value to 0 in decreased velocity
        _iters[i] -= v_change;
        _iters[i] = std::max(_iters[i], 0.);
        _limited_joint_cmds[i] = jointPosPrev(i) + _iters[i] * (diff > 0 ? 1.0 : -1.0);
      }
      else if (abs(diff) > change_dist)
      {
        case_used_in_trapezoid_vel = 3; // TODO: Delete when done debugging
        // distance is big, so increase velocity to max, if it's max,
        // send max command to controller from prev. pos.
        _iters[i] += v_change;
        _iters[i] = std::min(_iters[i], max_step);
        _limited_joint_cmds[i] = jointPosPrev(i) + _iters[i] * (diff > 0 ? 1.0 : -1.0);
      }
    }
  }
}

PLUGINLIB_EXPORT_CLASS(franka_aic::AICController,
                       controller_interface::ControllerBase)

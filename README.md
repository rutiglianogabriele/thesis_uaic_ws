# E-Grocery task with a robotic arm controller by an Active Inference Controller (AIC)

This repository contains all the ROS packages developed during the course of my Master Thesis in Automation and Robotics Engineering at Danmarks Tekniske Universitet (DTU).

## Table of contents
* [General Info](#general-info)
* [How to Install](#how-to-install)
* [How to Run It](#how-to-run-it)

# General Info

The available packages are:
<ol>
  <li> <strong>move_panda:</strong> Developed during the course of my master thesis. This package contains all the launch files and scripts to run the various e-grocery tasks.</li>
  <li><strong>object_detection:</strong> Developed during the course of my master thesis. This package contains the scripts needed to launch the camera and perform object detection and tracking. Morevoer, with the file <em>test_static_task</em> </li>
  <li><strong>franka_aic:</strong> This package has been originally developed by the authors in: https://github.com/cpezzato/active_inference. It has been modified to also implement the uAIC in:  https://github.com/cpezzato/unbiased_aic. This package contains the scripts and launch files to run the MRAC, AIC and uAIC controllers on the real Franka Emika Panda robot.</li>
  <li><strong>predictor:</strong> This packaged has been developed by student Christian Kampp Kruuse during the course of his master thesis. This package contains the scripts and launch files needed to run the Kalman Filter required to perform the prediction of the object pose during the dynamic e-grocery task.</li>
</ol>

The packages "move_panda", "object_detection" and "predictor" are project specific packages, this mean that they work well with the hardware set-up used during the course of this specific thesis project. For a different hardware set-up the code might need adaptation.

The results of the simuation of the e-grocery tasks can be seen at the following shared drive:
https://drive.google.com/drive/folders/1bWAupsXqaDjYjVLPrEB_1UMEMhO6jONW?usp=drive_link

# How to Install

Simply clone this repository inside your workspace and build it. Assuming your workspace is "my_ws":
```
cd my_ws/src
git clone git@github.com:rutiglianogabriele/thesis_uaic_ws.git
cd ..
catkin build
```

# How to Run It

First align the camera and the robot. Run the object_tracking_dynamic.py and:
<ol>
  <li>Align the blue strokes to the base frame of the robot (vertical blue line aligned with the front small triangle, horizonal line aligned with the lateral lights of the robot)</li>
  <li>Align the white horizontal line with the upper (from camera view, facing the robot base) side of the bolts.</li>
  <li>Align the conveyor belt with the two lower white horizontal lines on the bottom</li>
</ol>

At this point, run the conveyor belt at the desired speed and run this in a terminal on the computer:

```
cd Gabriele_thesis/panda_ws 
```

Open 2  more terminals and do:
```
source ./devel/setup.bash 
```
in all of them.

Then run each command in each different terminal tab (one command, one tab):
```
roscore
roslaunch franka_aic MRAC_controller.launch (for the MRAC controller)
roslaunch franka_aic AIC_controller.launch (for the uAIC controller)
```

In the last terminal you can launch the desired e-grocery task. Run either of the following:
```
roslaunch move_panda egrocery_static_task.launch (for the static e-grocery task, without conveyor belt)
roslaunch move_panda egrocery_dynamic_task.launch (for the dynamic e-grocery task, with conveyor belt)
```



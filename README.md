# E-Grocery task with a robotic arm controller by an Active Inference Controller (AIC)

## Table of contents
* [General Info](#general-info)

# General Info

This repository contains multiple ROS packages.
<ol>
  <li> <strong>move_panda:</strong> Developed during the course of my master thesis. This package contains all the launch files and scripts to run the various e-grocery tasks.</li>
  <li>**object_detection:** Developed during the course of my master thesis. This package contains the scripts needed to launch the camera and perform object detection and tracking. Morevoer, with the file <em>test_static_task</em> </li>
  <li>**franka_aic:** This package has been originally developed by the authors in: https://github.com/cpezzato/active_inference. It has been modified to also implement the uAIC in:  https://github.com/cpezzato/unbiased_aic. This package contains the scripts and launch files to run the MRAC, AIC and uAIC controllers on the real Franka Emika Panda robot.</li>
  <li>**predictor:** This packaged has been developed by student Christian Kampp Kruuse during the course of his master thesis. This package contains the scripts and launch files needed to run the Kalman Filter required to perform the prediction of the object pose during the dynamic e-grocery task.</li>
</ol>


The results of the simuation of the e-grocery tasks can be seen at the following shared drive:
https://drive.google.com/drive/folders/1bWAupsXqaDjYjVLPrEB_1UMEMhO6jONW?usp=drive_link

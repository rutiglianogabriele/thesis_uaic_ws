
import numpy as np
import math as m


def transform_camera_to_robot(x, y):
    # Minimum height for the gripper pose along the z axis. This will be hardcoded when sending the
    # Coordinates where the robot has to be sent.
    min_z = 0.27

    # Translation matrix
    # Translation of the origin of the camera frame along the x axis of the robot frame
    trans_x_1_0 = 0.359
    # Translation of the origin of the camera frame along the y axis of the robot frame
    trans_y_1_0 = 0

    # Rotations of the camera frame with respect to the robot frame
    phi = -90  # Rotation of the camera frame along the z axis

    # Roation angles:
    phi_rad = m.radians(phi)  # about z

    # Rotation about the z axis of phi degrees:
    R_z_phi = np.array([[m.cos(phi_rad), -m.sin(phi_rad), 0, 0],
                       [m.sin(phi_rad), m.cos(phi_rad), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    T_1_0 = R_z_phi
    T_1_0[0][3] = trans_x_1_0
    T_1_0[1][3] = trans_y_1_0

    # Vector P1, representing the pose of the object in camera frame:
    P1 = np.array([x, y, min_z, 1])
    P0 = np.dot(T_1_0, np.transpose(P1))

    print(P0)
    return P0


def find_pose(c, r):
    # Intrinsic parameters
    Cx = 640
    Cy = 360

    Fx = 955.529
    Fy = 955.126

    # Coordinates with respect to the camera frame
    z = 1.25

    # Pose with respect to camera Frame
    x_c = -((c-Cx)/Fx)*z
    y_c = ((r-Cy)/Fy)*z

    # Transform coordinates from camera frame to robot frame:
    x_w, y_w, _, _ = transform_camera_to_robot(x_c, y_c)

    pose = []
    pose.append([x_w, y_w])
    print("The object is placed at X: {:.2f} and Y: {:.2f}\nIn the robot frame".format(
        x_w, y_w))
    return pose


c = 500
r = 401

find_pose(c, r)

import sys
import cv2
import numpy as np
import math as m
import pyrealsense2 as rs


# The following class takes care of continuously receiving object coordinates in pixels and assigning
# Them with an ID. It is basically an obnject tracker. It returns a list of objects with their coordinates
### ----------------------------------------- BEGIN CLASS --------------------------------------###
class EuclideanDistTracker:
    def __init__(self):
        # Store the center positions of the objects
        self.center_points = {}
        # Keep the count of the IDs
        # each time a new object id detected, the count will increase by one
        self.id_count = 0

    def update(self, objects_rect):
        # Objects boxes and ids
        objects_bbs_ids = []

        # Get center point of new object
        for rect in objects_rect:
            x, y, w, h = rect
            cx = (x + x + w) // 2
            cy = (y + y + h) // 2

            # Find out if that object was detected already
            same_object_detected = False
            for id, pt in self.center_points.items():
                dist = m.hypot(cx - pt[0], cy - pt[1])

                if dist < 25:
                    self.center_points[id] = (cx, cy)
                    objects_bbs_ids.append([x, y, w, h, id, cx, cy])
                    same_object_detected = True
                    break

            # New object is detected we assign the ID to that object
            if same_object_detected is False:
                self.center_points[self.id_count] = (cx, cy)
                objects_bbs_ids.append([x, y, w, h, self.id_count, cx, cy])
                self.id_count += 1

        # Clean the dictionary by center points to remove IDS not used anymore
        new_center_points = {}
        for obj_bb_id in objects_bbs_ids:
            _, _, _, _, object_id, cx, cy = obj_bb_id
            center = self.center_points[object_id]
            new_center_points[object_id] = center

        # Update dictionary with IDs not used removed
        self.center_points = new_center_points.copy()
        return objects_bbs_ids

### ------------------------------------------ END CLASS ---------------------------------------###


# Create tracker object
tracker = EuclideanDistTracker()

# In the following class we are evaluating the pose of an object in the robot frame.
# The class should be able to detect pose of objects if the main variables that will be
# defined in the following lines are changed accordingly to the new set-up
### ------------------------------------- BEGIN POSE DETECTION ---------------------------------###

# Here we are not considering the translation along the z axis of the camera frame with respect
# to the robot frame. This is because for this simple application we can just assume that the final
# pose of the gripper will be equal to the minimal, which is min_z.


def transform_camera_to_robot(x, y, z):
    # Translation matrix
    # Translation of the origin of the camera frame along the x axis of the robot frame
    trans_x_1_0 = 0.355
    # Translation of the origin of the camera frame along the y axis of the robot frame
    trans_y_1_0 = 0
    # Translation of the origin of the camera frame along the z axis of the robot frame
    trans_z_1_0 = 1.32

    # Rotations about current x-axes
    theta = 180  # Rotation of the camera frame along the z axis

    # Roation angles:
    theta_rad = m.radians(theta)  # about x

    # Rotation about the x axis of phi degrees:
    R_x_theta = np.array([[1, 0, 0, 0],
                          [0, m.cos(theta_rad), -m.sin(theta_rad), 0],
                          [0, m.sin(theta_rad), m.cos(theta_rad), 0],
                          [0, 0, 0, 1]])

    # Rotations about current z-axes
    phi = -90  # Rotation of the camera frame along the z axis

    # Roation angles:
    phi_rad = m.radians(phi)  # about z

    # Rotation about the z axis of phi degrees:
    R_z_phi = np.array([[m.cos(phi_rad), -m.sin(phi_rad), 0, 0],
                       [m.sin(phi_rad), m.cos(phi_rad), 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    T_1_0 = np.dot(R_x_theta, R_z_phi)
    T_1_0[0][3] = trans_x_1_0
    T_1_0[1][3] = trans_y_1_0
    T_1_0[2][3] = trans_z_1_0

    print("The object is placed at X: {:.2f},  Y: {:.2f} and Z: {:.2f}\nIn the camera frame".format(
        x, y, z))
    # Vector P1, representing the pose of the object in camera frame:
    P1 = np.array([[x], [y], [z], [1]])
    P0 = T_1_0@P1

    return P0


def find_pose(r, c, z_c):
    # Intrinsic parameters
    Cx = 360
    Cy = 640

    Fx = 913.77
    Fy = 910.35

    # Pose with respect to camera Frame
    x_c = ((r-Cy)/Fy)*z_c
    y_c = ((c-Cx)/Fx)*z_c

    # Transform coordinates from camera frame to robot frame:
    x_w, y_w, z_w, _ = transform_camera_to_robot(x_c, y_c, z_c)

    pose = []
    pose.append([x_w, y_w, z_w])
    print(x_w, y_w, z_w)
    return pose

### -------------------------------------- END POSE DETECTION ----------------------------------###

# Function to draw two parallel lines on the image


def draw_conveyor_belt(image, height, conv_cy, line_color=(255, 255, 255), line_thickness=1):
    # Get image dimensions
    img_height, img_width, _ = image.shape

    # Calculate the y-coordinate for the top and bottom lines
    top_y = conv_cy - height // 2
    bottom_y = conv_cy + height // 2

    # Draw the top line
    cv2.line(image, (0, top_y), (img_width, top_y),
             color=line_color, thickness=line_thickness)

    # Draw the bottom line
    cv2.line(image, (0, bottom_y), (img_width, bottom_y),
             color=line_color, thickness=line_thickness)

# Draw the lines needed to place the camera perfectly with respect of the robot


def draw_reference_for_calibration(image):
    # Get image dimensions
    img_height, img_width, _ = image.shape

    top_y = 120

    # Draw a short cross at the top
    cross_arm_length = 100 // 2
    cv2.line(image, (img_width // 2, top_y - 150),
             (img_width // 2, top_y + 450),
             color=(255, 0, 0), thickness=1)
    cv2.line(image, (img_width // 2 - 200, top_y),
             (img_width // 2 + 200, top_y),
             color=(255, 0, 0), thickness=1)

    # Draw the top line
    cv2.line(image, (0, top_y+50), (img_width, top_y+50),
             color=(255, 255, 255), thickness=1)

# Function to crop a specific section of the color frame


def crop_region_of_interest(image, top_y, height, left_x, width):
    # Get the region of interest (ROI)
    roi = image[top_y:top_y + height, left_x:left_x + width]

    return roi


# Set the height of the conveyor belt lines
conveyor_belt_center_y = 425
conveyor_belt_height = 140
conveyor_belt_width = 230
cropping_y = 351
cropping_x = 875

# Configure RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
# _________________________________________________________________DEPTH_______________________________________________'
# Enable the depth stream
config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
# Create a colorizer object
colorizer = rs.colorizer()


# Declare filters
# Decimation - reduces depth frame density
dec_filter = rs.decimation_filter(2)
# Spatial edge-preserving spatial smoothing
spat_filter = rs.spatial_filter()
# Hole fillinf filter
hole_filling = rs.hole_filling_filter(1)
temp_filter = rs.temporal_filter()    # Temporal   - reduces temporal noise

# Aligning tool from realsense
align_to = rs.stream.color
align = rs.align(align_to)

# _________________________________________________________________COLOR_______________________________________________
# Enable the color stream
config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)


try:
    while True:
        # Wait for the next set of frames
        frames = pipeline.wait_for_frames()

        # Aligning depth frame to rgb
        aligned_frames = align.process(frames)
        # _________________________________________________________________DEPTH_______________________________________________
        # Get Depth frames
        depth_frame = aligned_frames.get_depth_frame()
        filtered = spat_filter.process(depth_frame)
        filtered = temp_filter.process(filtered)
        filtered = hole_filling.process(filtered)

        depth_frame = filtered.as_depth_frame()

        # _________________________________________________________________COLOR_______________________________________________
        # Get the color frame
        color_frame = aligned_frames.get_color_frame()

        # Convert the color frame to a numpy array
        color_image = np.asanyarray(color_frame.get_data())

        # Draw the conveyor belt lines on the image
        # draw_conveyor_belt(color_image, conveyor_belt_height,
        #                    conveyor_belt_center_y)

        # Get the center coordinates of the image
        center_x = color_image.shape[1] // 2
        center_y = color_image.shape[0] // 2

        # Draw a dot at the center of the image
        # cv2.circle(color_image, (center_x, center_y),
        #            radius=5, color=(0, 255, 0), thickness=-1)

        # draw_reference_for_calibration(color_image)

        # Crop the region of interest (ROI) ---- BOARD
        cropping_y = 200
        cropping_x = 350
        roi = color_image[cropping_y:600, cropping_x:950]

        roi_gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        _, threshold = cv2.threshold(roi_gray, 150, 255, cv2.THRESH_BINARY)

        # Detect the objects
        contours, _ = cv2.findContours(
            threshold, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        detections = []

        for cnt in contours:
            (x, y, w, h) = cv2.boundingRect(cnt)
            # Calculate area
            area = cv2.contourArea(cnt)
            # Filtering noise by area of the detecetd objct
            if area > 1000:
                # Evaluate the center of the figure
                M = cv2.moments(cnt)
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])

                # Draw the center of the object
                cv2.circle(roi_gray, (cX, cY), 7, (255, 255, 255), -1)
                cv2.putText(roi_gray, "center (x:" + str(cX+cropping_x) + ", y: " + str(cY+cropping_y) + ")", (cX - 20, cY - 20),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                # Append the data for the center in the detections:
                detections.append([x, y, w, h])

        # 2. Object Tracking
        boxes_ids = tracker.update(detections)

        for box_id in boxes_ids:
            x, y, w, h, id, cx, cy = box_id

            # Update the centers with the shift dictated by the mask we apply to the frame (shift = x1 and y1)
            cx = cx + cropping_x
            cy = cy + cropping_y

            cv2.putText(roi_gray, str(id), (x, y - 15),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.rectangle(roi_gray, (x, y), (x + w, y + h), (0, 255, 0), 3)

            cz = depth_frame.get_distance(cx, cy)  # Depth reading

            found_pose = find_pose(cx, cy, cz)

            cx_w = found_pose[0][0]
            cy_w = found_pose[0][1]
            cz_w = found_pose[0][2]

            # Display the cropped color frame
            cv2.imshow('Color Stream', color_image)

            # Convert the depth frame to a numpy array
            depth_image = np.asanyarray(
                colorizer.colorize(depth_frame).get_data())

            # Display the depth image
            cv2.imshow('Depth Image', depth_image)

            # Break the loop if the 'q' key is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break


finally:
    # Stop streaming and close all OpenCV windows
    pipeline.stop()
    cv2.destroyAllWindows()

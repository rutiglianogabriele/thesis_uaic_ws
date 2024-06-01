#!/usr/bin/env python

# Simple scripts that sends some coordinates to the robot.
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

STATION_A_X = STATION_A_Y = STATION_B_Y = STATION_D_X = STATION_E_X = 0.25
STATION_B_X = STATION_C_X = STATION_D_X = 0.55
STATION_C_Y = 0
STATION_D_Y = STATION_E_Y = -0.25
Z = 0.6
Z1 = 0.3


pub = rospy.Publisher(
    '/desktop/target_pose_cov_stmp_panda_link0', PoseWithCovarianceStamped, queue_size=10)

rospy.init_node('coordinate_publisher')


def main():
    # Define first position
    des_pose1 = PoseWithCovarianceStamped()

    des_pose1.pose.pose.position.x = STATION_A_X
    des_pose1.pose.pose.position.y = STATION_A_Y
    des_pose1.pose.pose.position.z = Z
    des_pose1.pose.pose.orientation.x = 0
    des_pose1.pose.pose.orientation.y = 0
    des_pose1.pose.pose.orientation.z = 1

    # Define picking position
    des_pose2 = PoseWithCovarianceStamped()

    des_pose2.pose.pose.position.x = STATION_C_X
    des_pose2.pose.pose.position.y = STATION_C_Y
    des_pose2.pose.pose.position.z = Z
    des_pose2.pose.pose.orientation.x = 0
    des_pose2.pose.pose.orientation.y = 0
    des_pose2.pose.pose.orientation.z = 1

    # Define hovering position
    des_pose3 = PoseWithCovarianceStamped()

    des_pose3.pose.pose.position.x = STATION_C_X
    des_pose3.pose.pose.position.y = STATION_C_Y
    des_pose3.pose.pose.position.z = Z1
    des_pose3.pose.pose.orientation.x = 0
    des_pose3.pose.pose.orientation.y = 0
    des_pose3.pose.pose.orientation.z = 1

    # Define final position
    des_pose5 = PoseWithCovarianceStamped()

    des_pose5.pose.pose.position.x = STATION_E_X
    des_pose5.pose.pose.position.y = STATION_E_Y
    des_pose5.pose.pose.position.z = Z
    des_pose5.pose.pose.orientation.x = 0
    des_pose5.pose.pose.orientation.y = 0
    des_pose5.pose.pose.orientation.z = 1

    start_time = rospy.Time.now()
    rospy.loginfo("node started at %s", start_time)

    rate = rospy.Rate(900)

    while not rospy.is_shutdown():
        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        if elapsed_time.to_sec() < 10:
            rospy.loginfo("1")
            pub.publish(des_pose1)
        elif 10 < elapsed_time.to_sec() < 20:
            rospy.loginfo("2")
            pub.publish(des_pose2)
        elif 20 < elapsed_time.to_sec() < 30:
            rospy.loginfo("3")
            pub.publish(des_pose3)
        elif 30 < elapsed_time.to_sec() < 40:
            rospy.loginfo("4")
            pub.publish(des_pose2)
        elif 40 < elapsed_time.to_sec() < 50:
            rospy.loginfo("5")
            pub.publish(des_pose5)
        elif 50 < elapsed_time.to_sec() < 60:
            rospy.loginfo("6")
            pub.publish(des_pose2)

        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

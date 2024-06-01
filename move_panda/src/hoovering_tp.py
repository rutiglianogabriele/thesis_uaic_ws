#!/usr/bin/env python

# Simple scripts that sends some coordinates to the robot.
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


pub = rospy.Publisher(
    '/desktop/target_pose_cov_stmp_panda_link0', PoseWithCovarianceStamped, queue_size=10)

rospy.init_node('coordinate_publisher')


def main():
    # Define first position

    des_pose1 = PoseWithCovarianceStamped()

    des_pose1.pose.pose.position.x = -0.1
    des_pose1.pose.pose.position.y = -0.6
    des_pose1.pose.pose.position.z = 0.55
    des_pose1.pose.pose.orientation.x = 0
    des_pose1.pose.pose.orientation.y = 0
    des_pose1.pose.pose.orientation.z = 1

    start_time = rospy.Time.now()
    rospy.loginfo("node started at %s", start_time)

    rate = rospy.Rate(1000)

    while not rospy.is_shutdown():

        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time

        pub.publish(des_pose1)

        rate.sleep()
    rospy.spin()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

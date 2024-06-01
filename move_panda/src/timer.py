#!/usr/bin/env python

import rospy
from std_msgs.msg import String

pub = rospy.Publisher('timer_command', String, queue_size=1)


def missionCallback(data):
    rospy.sleep(5)
    command_str = "Go"
    pub.publish(command_str)


def main():
    rospy.Subscriber("start_timer", String, missionCallback)
    rospy.init_node('timer', anonymous=True)

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():

        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

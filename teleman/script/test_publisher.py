#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from datetime import datetime


def teleman_talker():
    rospy.init_node('test_publisher', anonymous=True)
    pub = rospy.Publisher('/next_trial', String, queue_size=1)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        a = input("put num for next trial : ")
        pub.publish(a)
        rate.sleep()


if __name__ == '__main__':
    print("test publisher starts at")
    #print(datetime.utcnow().strftime('%Y-%m-%d %H:%M:%S.%f')[:-3])

    try:
        teleman_talker()
    except rospy.ROSInterruptException:
        pass

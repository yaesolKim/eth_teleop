#!/usr/bin/env python

import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped

haptic_pose = PoseStamped()

def haptic_talker():
    global haptic_pose
    rospy.init_node('test_publisher', anonymous=True)
    pub = rospy.Publisher('/cp_haptic', PoseStamped, queue_size=10)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        haptic_pose.pose.position.x += 0.04 #frontr/back
        haptic_pose.pose.position.y += 0.04 #left/right
        haptic_pose.pose.position.z += 0.04 #up/down

        #haptic_pose.pose.position.x = 5.0 #front/back
        #haptic_pose.pose.position.y = -0.1 #left/right
        #haptic_pose.pose.position.z = 6.0 #up/down

        haptic_pose.pose.orientation.x = 0.7071068
        haptic_pose.pose.orientation.y = 0.000
        haptic_pose.pose.orientation.z = 0.7071068
        haptic_pose.pose.orientation.w = 0.000

        pub.publish(haptic_pose)
        rate.sleep()

if __name__ == '__main__':
    print("test publisher starts")

    #get current robot position
    haptic_pose.header.frame_id = "s1"

    haptic_pose.pose.position.x = 5.0  # front/back
    haptic_pose.pose.position.y = 0.0  # left/right
    haptic_pose.pose.position.z = 6.0  # up/down

    haptic_pose.pose.orientation.x = 0.7071068
    haptic_pose.pose.orientation.y = 0.000
    haptic_pose.pose.orientation.z = 0.7071068
    haptic_pose.pose.orientation.w = 0.000

    try:
        haptic_talker()
    except rospy.ROSInterruptException:
        pass

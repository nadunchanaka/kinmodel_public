#!/usr/bin/env python

import rospy
from std_msgs.msg import *
from geometry_msgs.msg import Twist
import time

cmdData = Twist()
cmd_count = 0

def repub(data):
    global cmdData
    global cmd_count

    cmd_count += 1

    cmdData = data

def pubRepub(event):
    VAL = rospy.Publisher('/cmd_velue', Twist , queue_size=10)

    # print (cmdData)
    VAL.publish(cmdData)

def cmd_check(event):
    global cmd_count
    if cmd_count < 2:
        cmdData.linear.x = 0
        cmdData.angular.z = 0
    cmd_count = 0

    print(cmd_count)


def feedback():
    rospy.init_node('kin_model', anonymous=True)
    # rate = rospy.Rate(100) # 10hz
    while not rospy.is_shutdown():
        rospy.Subscriber('/cmd_vel', Twist, repub)
        timer = rospy.Timer(rospy.Duration(0.1), pubRepub)
        timer1 = rospy.Timer(rospy.Duration(3), cmd_check)
        rospy.spin()
        timer.shutdown()
        timer1.shutdown()

if __name__ == '__main__':
    try:
        # global data
        feedback()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit

import rospy, os, math, re
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def callback(msg):
    global position
    position  = msg.pose.pose.position

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:713]), 10),
    }
    take_action(regions)

def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0
    print position
    if (float(position.x) > 0 and float(position.x) < 2) and float(position.y) < -9:
        linear_x = 0
        angular_z = 0
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        pub.publish(msg)
        print "finish!"
        exit()
    else:
        if regions['front'] > 2 and regions['fleft'] > 2 and regions['fright'] > 2:
            state_description = 'case 1 - nothing'
            linear_x = 0.3
            angular_z = 0
        elif regions['front'] < 2 and regions['fleft'] > 2 and regions['fright'] > 2:
            state_description = 'case 2 - front'
            if regions['front'] < 0.5:
                linear_x = 0
                angular_z = 0.5
            else:
                linear_x = 0.3
                angular_z = 0.5
        elif regions['front'] < 2 and regions['fleft'] > 2 and regions['fright'] < 2:
            state_description = 'case 5 - front and fright'
            if regions['front'] < 0.5:
                linear_x = 0
                angular_z = -0.5
            else:
                linear_x = 0.3
                angular_z = -0.5
        elif regions['front'] < 2 and regions['fleft'] < 2 and regions['fright'] > 2:
            state_description = 'case 6 - front and fleft'
            if regions['front'] < 0.5:
                linear_x = 0
                angular_z = 0.5
            else:
                linear_x = 0.3
                angular_z = 0.5
        elif regions['front'] < 2 and regions['fleft'] < 2 and regions['fright'] < 2:
            state_description = 'case 7 - front and fleft and fright'
            if regions['front'] < 0.5 and regions['fleft'] < 0.5:
                linear_x = 0
                angular_z = 0.5
            elif regions['front'] < 0.5 and regions['fright'] < 0.5:
                linear_x = 0
                angular_z = -0.5
            else:
                linear_x = 0.3
                angular_z = 0.5
        else:
            state_description = 'unknown case'
            #rospy.loginfo(regions)
            linear_x = 0.3
            angular_z = 0

    #rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.angular.z = angular_z
    pub.publish(msg)
    exit()

def main():
    global pub

    rospy.init_node('reading_laser')    
    pub = rospy.Publisher('/m2wr/cmd_vel', Twist, queue_size=1)    
    sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    odom_sub = rospy.Subscriber('/m2wr/odom', Odometry, callback)    
    rospy.spin()

if __name__ == '__main__':
    main()

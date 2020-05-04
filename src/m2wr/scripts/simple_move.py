#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit

import rospy
from geometry_msgs.msg import Twist
import math,os

PI = math.pi    

class M2WR :
    def __init__(self, topic):
        self.topic = topic

    def move(self, lin_speed, ang_speed, time):
        velocity_publisher = rospy.Publisher(self.topic, Twist, queue_size=10)
        vel_msg = Twist()
        
        vel_msg.linear.x = abs(lin_speed)
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = abs(ang_speed)

        while not rospy.is_shutdown():

            t0 = rospy.Time.now().to_sec()

            while(rospy.Time.now().to_sec() - t0 < time):
                velocity_publisher.publish(vel_msg)

            vel_msg.linear.x = 0
            vel_msg.angular.z = 0
            velocity_publisher.publish(vel_msg)
                
            break

def handler(signal_received, frame):
    # Handle CTRL-C in Python2
    print("")
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

if __name__ == '__main__':
    signal(SIGINT, handler)
    rospy.init_node('m2wr_controller', anonymous=True)
    topic = raw_input("Topic to control : ")
    robot = M2WR(topic)
    try:
        while True:
            lin_speed = input("Input Linear Speed : ")
            ang_speed = input("Input Angular Speed : ")
            time = input("Input time : ")
            robot.move(lin_speed, ang_speed, time)
    except rospy.ROSInterruptException: 
        pass
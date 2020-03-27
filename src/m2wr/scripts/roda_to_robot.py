#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit

import rospy,os
import re
from geometry_msgs.msg import Twist
import math

class M2WR:
    def __init__(self, topic):
        self.topic = topic

    def roda_to_robot(self, omega_l, omega_r, time):
        xacro_file = open(os.getcwd()+'/src/m2wr/urdf/m2wr.xacro', 'r')
        radius_roda = 0
        jarak_antar_roda = 0
        for line in xacro_file:
            if "<wheelSeparation>" in line:
                jarak_antar_roda = line
            elif "<wheelDiameter>" in line:
                radius_roda = line
        
        radius_roda = float(re.findall(r"\d+\.\d+",radius_roda)[0])
        jarak_antar_roda = float(re.findall(r"\d+\.\d+",jarak_antar_roda)[0])

        # kecepatan linear robot
        Vx = (radius_roda * omega_l)/2 + (radius_roda * omega_r)/2
        # resultan terhadap Vy = 0
        Vy = 0
        # kecepatan sudut robot
        W = (radius_roda * omega_l)/jarak_antar_roda - (radius_roda * omega_r)/jarak_antar_roda

        return Vx, W, time

    def move(self, lin_speed, ang_speed, time):
        velocity_publisher = rospy.Publisher(self.topic, Twist, queue_size=10)
        vel_msg = Twist()
        
        vel_msg.linear.x = lin_speed
        vel_msg.linear.y = 0
        vel_msg.linear.z = 0
        vel_msg.angular.x = 0
        vel_msg.angular.y = 0
        vel_msg.angular.z = ang_speed
        
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
            l_wheel_ang_speed = input("Input Angular speed Left wheel : ")
            r_wheel_ang_speed = input("Input Angular Speed Right Wheel: ")
            time = input("Input time : ")
            lin_speed, ang_speed, time = robot.roda_to_robot(l_wheel_ang_speed, \
                                        r_wheel_ang_speed, time)
            robot.move(lin_speed, ang_speed, time)
    except rospy.ROSInterruptException: 
        pass
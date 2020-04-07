#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit

import rospy, os, math, re
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry

def get_dist_and_radius_wheel():
    xacro_file = open(os.getcwd()+'/src/m2wr/urdf/gazebo.xacro', 'r')
    radius_roda = 0
    jarak_antar_roda = 0
    for line in xacro_file:
        if "<wheelSeparation>" in line:
            jarak_antar_roda = line
        elif "<wheelDiameter>" in line:
            radius_roda = line

    radius_roda = float(re.findall(r"\d+\.\d+",radius_roda)[0])
    jarak_antar_roda = float(re.findall(r"\d+\.\d+",jarak_antar_roda)[0])

    return radius_roda/2, jarak_antar_roda

PI = math.pi
radius_roda, jarak_antar_roda = get_dist_and_radius_wheel()
position = None

def clbk_position(msg):
    global position
    position  = msg.pose.pose.position

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 20),
        'fright': min(min(msg.ranges[144:287]), 20),
        'front':  min(min(msg.ranges[288:431]), 20),
        'fleft':  min(min(msg.ranges[432:575]), 20),
        'left':   min(min(msg.ranges[576:719]), 20),
    }
    take_action(regions)

def take_action(regions):
    msg = Twist()

    omega_l, omega_r = 0, 0
    global position
    if (float(position.x) > 0 and float(position.x) < 2) and float(position.y) < -9:
        omega_l, omega_r = 0, 0
    else:
        if regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            omega_l, omega_r = 5, 5
        elif regions['front'] > 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            omega_l, omega_r = 3, 5
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            omega_l, omega_r = 5, 3
        elif regions['front'] > 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            if regions['left'] > regions['right']:  
                omega_l, omega_r = 4, 5
            elif regions['left'] < regions['right']:
                omega_l, omega_r = 5, 4
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] > 1:
            omega_l, omega_r = 5, 2.5
        elif regions['front'] < 1 and regions['fleft'] > 1 and regions['fright'] < 1:
            omega_l, omega_r = 2.5, 5
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] > 1:
            omega_l, omega_r = 5, 2.5
        elif regions['front'] < 1 and regions['fleft'] < 1 and regions['fright'] < 1:
            if regions['left'] > regions['right']:  
                omega_l, omega_r = -2.5, 2.5
            elif regions['left'] < regions['right']:
                omega_l, omega_r = 2.5, -2.5
        else:
            state_description = 'unknown case'
            omega_l, omega_r = 5, 5

    linear_speed, angular_speed = roda_to_robot(omega_l, omega_r)
    msg.linear.x = linear_speed
    msg.angular.z = angular_speed
    pub.publish(msg)
    exit()

def roda_to_robot(omega_l, omega_r):
    Vx = (radius_roda * omega_l)/2 + (radius_roda * omega_r)/2
    #Vy selalu 0
    W = (radius_roda * omega_l)/jarak_antar_roda - (radius_roda * omega_r)/jarak_antar_roda
    return Vx, W

def main():
    global pub

    rospy.init_node('reading_laser')    
    pub = rospy.Publisher('/m2wr/cmd_vel', Twist, queue_size=1)    
    laser_sub = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    odom_sub = rospy.Subscriber('/m2wr/odom', Odometry, clbk_position)    
    rospy.spin()

if __name__ == '__main__':
    main()

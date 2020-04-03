#!/usr/bin/env python

from signal import signal, SIGINT
from sys import exit
from nav_msgs.msg import Odometry

import rospy, os, math, re
from geometry_msgs.msg import Twist


def get_dist_and_radius_wheel():
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

    return radius_roda/2, jarak_antar_roda


# default 0.4 (jarak roda) dan 0.2 diameter roda
PI = math.pi
radius_roda, jarak_antar_roda = get_dist_and_radius_wheel()
print(radius_roda, jarak_antar_roda)

class M2WR :
    def __init__(self, topic):
        self.topic = topic

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

def roda_to_robot(omega_l, omega_r):
    Vx = (radius_roda * omega_l)/2 + (radius_roda * omega_r)/2
    #Vy selalu 0
    W = (radius_roda * omega_l)/jarak_antar_roda - (radius_roda * omega_r)/jarak_antar_roda
    return Vx, W

def roda_to_pose(omega_l, omega_r, time, theta_rad, x, y):
    Vx, W = roda_to_robot(omega_l, omega_r)
    theta_t_rad = W * time

    xt, yt = x, y
    if (W == 0):
        xt += Vx * math.cos(theta_rad) * time
        yt += Vx * -math.sin(theta_rad) * time
    else:
        xt += ((Vx*math.sin(theta_t_rad+theta_rad)/W) - (Vx*math.sin(0+theta_rad)/W))
        
        #menyesuaikan dengan koordinat gazebo maka yt akan dikali minus
        yt += -((Vx*math.cos(0+theta_rad)/W) - (Vx*math.cos(theta_t_rad+theta_rad)/W))

    theta_t_rad += theta_rad

    return Vx, W, theta_t_rad, xt, yt



if __name__ == '__main__':
    signal(SIGINT, handler)
    rospy.init_node('m2wr_controller', anonymous=True)
    topic = raw_input("Topic to control : ")
    robot = M2WR(topic)
    x = 0
    y = 0
    theta_rad = 0
    try:
        while True:
            omega_l = input("Input kecepatan sudut roda kiri : ")
            omega_r = input("Input kecepatan sudut roda kanan : ")
            time = input("Input t : ")
            linear_speed, angular_speed, theta_t_rad, xt, yt = roda_to_pose(omega_l, omega_r, time, theta_rad, x, y)
            # print kecepatan linear dan angular             
            print("v = ", linear_speed)
            print("w = ", angular_speed)

            # print vektor pose ###
            x = xt
            y = yt
            theta_rad = theta_t_rad
            print("theta = ", theta_rad * (180/PI))
            print("x = ", x)
            print("y = ", y)
            robot.move(linear_speed, angular_speed, time)
    except rospy.ROSInterruptException: 
        pass

#! /usr/bin/env python

import rospy

from signal import signal, SIGINT
from sys import exit

from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

def callback_laser(msg) :
  laser_region = {
    'right' : max(msg.ranges[0:143]),
    'fright' : min(msg.ranges[144:287]),
    'front' : min(msg.ranges[288:431]),
    'fleft' : min(msg.ranges[432:575]),
    'left' : max(msg.ranges[576:719])
  }
  move_action(laser_region)

def move_action(region) :
  msg = Twist()
  lin_spd = 0.0
  ang_spd = 0.0

  state = ""

  if (region["front"] > 2) :
    state = "Forward"
    lin_spd = 2
    ang_spd = 0

  elif (region["front"] > 1.5):
    state = "Obstacle Ahead"
    lin_spd = 1.5
    ang_spd = 0

  elif (region["front"] <= 1.5) :
    lin_spd = 1
    # if (region["left"] > 2):
    #   state = "Case B : Turn Left"
    #   ang_spd = -1.8
    # elif (region["right"] > 2) :
    #   state = "Case C : Turn Right"
    #   ang_spd = 1.8
    if (region["left"] > region['right']):
      state = "Turn Left"
      ang_spd = -1.8
    elif (region["left"] < region['right']) :
      state = "Turn Right"
      ang_spd = 1.8
    else :
      state = "Slow Down"
      ang_spd = 0
  else :
    state = "Unknown Case"
    rospy.loginfo(region)
  
  rospy.loginfo(state)
  msg.linear.x = lin_spd
  msg.angular.z = ang_spd
  pub.publish(msg)

def main():
  global pub

  rospy.init_node("obstacle_avoider")
  
  sub = rospy.Subscriber("/m2wr/laser/scan",LaserScan, callback_laser)
  pub = rospy.Publisher('/m2wr/cmd_vel', Twist, queue_size=1)
  rospy.spin()

def handler(signal_received, frame):
    # Handle CTRL-C in Python2
    print("")
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

if __name__ == "__main__" :
  signal(SIGINT, handler)
  main()

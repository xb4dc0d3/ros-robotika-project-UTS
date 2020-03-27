#! /usr/bin/env python

from signal import signal, SIGINT
from sys import exit

# import ros stuff
import rospy
from geometry_msgs.msg import TwistStamped, Twist, Pose, Vector3, Point, Quaternion
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker

class TrajectoryInteractiveMarkers:
    def __init__(self, topic):
        self.count = 0 
        # rospy.Subscriber("/cmd_vel",TwistStamped, self.event_in_cb)
        rospy.Subscriber(topic,Twist, self.event_in_cb)
        rospy.sleep(0.5)

    def event_in_cb(self,msg):
        self.waypoints = msg
        self.a = list()
        # self.a.append(self.waypoints.twist.linear.x)
        # self.a.append(self.waypoints.twist.linear.y)
        # self.a.append(self.waypoints.twist.linear.z)

        self.a.append(self.waypoints.linear.x)
        self.a.append(self.waypoints.linear.y)
        self.a.append(self.waypoints.linear.z)
        self.show_text_in_rviz()

    def show_text_in_rviz(self):
        self.marker = Marker()
        self.marker_publisher = rospy.Publisher('visualization_marker', Marker, queue_size=5)
        self.marker = Marker(
                    type=Marker.SPHERE,
                    id=0,
                    lifetime=rospy.Duration(1000),
                    pose=Pose(Point(self.a[0]/10**5,self.a[1]/10**5,self.a[2]/10**5), Quaternion(0, 0, 0, 1)),
                    scale=Vector3(0.05, 0.05, 0.05),
                    header=Header(frame_id='link_chassis'),
                    color=ColorRGBA(0.0, 2.0, 0.0, 0.8))
        self.count+=1
        self.marker.id = self.count
        self.marker_publisher.publish(self.marker)

def handler(signal_received, frame):
    # Handle CTRL-C in Python2
    print("")
    print('SIGINT or CTRL-C detected. Exiting gracefully')
    exit(0)

if __name__ == '__main__':
    signal(SIGINT, handler)
    rospy.init_node("trajectory_interactive_markers_node", anonymous=True)
    topic = raw_input("Topic to subscribe : ")
    trajectory_interactive_markers = TrajectoryInteractiveMarkers(topic)
    rospy.sleep(0.5)
    rospy.spin()
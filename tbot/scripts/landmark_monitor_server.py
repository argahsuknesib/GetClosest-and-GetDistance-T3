#!/usr/bin/env python3
import rospy
import math

from tbot_msgs.srv import GetClosest, GetClosestResponse, GetDistance, GetDistanceResponse
from nav_msgs.msg import Odometry


class LandmarkMonitor(object):
    def __init__(self):
        self._landmarks = {
         "Cylinder_1": (-1.1, -1.1),
         "Cylinder_2": (-1.1, 0),
         "Cylinder_3": (-1.1, 1.1),
         "Cylinder_4": (0, -1.1),
         "Cylinder_5": (0,0),
         "Cylinder_6": (0, 1.1),
         "Cylinder_7": (1.1, -1.1),
         "Cylinder_8": (1.1, 0),
         "Cylinder_9":(1.1, 1.1),
         "GreenBox_1":(3.5, 0),
         "GreenBox_2":(1.8, 2.7),
         "GreenBox_3":(1.8, -2.7),
         "GreenBox_4": (-1.8, 2.7),
         "GreenBox_5":(-1.8, -2.7),
         "HOME":(0,0)
        }
        self._x = 0
        self._y = 0

    def get_closest(self,req):
        rospy.loginfo('GetClosest is called')
        best_landmark = ''
        best_distance = -1
        for name, (x,y) in self._landmarks.items():
            dx = x - self._x
            dy = y - self._y
            square_distance = dx*dx + dy*dy
            if best_distance == -1 or square_distance < best_distance:
                best_distance = square_distance
                best_landmark = name
        response = GetClosestResponse()
        response.name = best_landmark
        return response

    def get_distance(self,req):
        rospy.loginfo('GetDistance called with {}'.format(req.name))
        if req.name not in self._landmarks:
            rospy.logerr('Unknown landmark "{}"'.format(req.name))
            return None
        x,y = self._landmarks[req.name]
        dx = x - self._x
        dy = x - self._y
        response = GetDistanceResponse()
        response.distance = math.sqrt(dx*dx + dy*dy)
        return response

    def odom_callback(self,msg):
        self._x = msg.pose.pose.position.x
        self._y = msg.pose.pose.position.y
        rospy.loginfo('x:{:.2f}, y:{:.2f}'.format(self._x,self._y))

def main():
    rospy.init_node('landmark_server')
    monitor = LandmarkMonitor()
    get_closest = rospy.Service('get_closest', GetClosest, monitor.get_closest)
    get_distance = rospy.Service('get_distance', GetDistance, monitor.get_distance)
    sub = rospy.Subscriber('/odom', Odometry, monitor.odom_callback)

    rospy.spin()

if __name__ == '__main__':
    main()
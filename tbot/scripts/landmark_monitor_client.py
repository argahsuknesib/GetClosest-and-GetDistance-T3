#!/usr/bin/env python3
from __future__ import print_function
import rospy
import tbot_msgs.srv


def main():
    rospy.init_node('landmark_client')

    get_closest = rospy.ServiceProxy('get_closest', tbot_msgs.srv.GetClosest)
    get_distance = rospy.ServiceProxy('get_distance',tbot_msgs.srv.GetDistance)
    req =tbot_msgs.srv.GetClosestRequest()
    resp = get_closest(req)
    print('Closest: {}'.format(resp.name))

    landmarks = ['Cylinder_1', 'Cylinder_2', 'Cylinder_3', 'Cylinder_4',
                 'Cylinder_5', 'Cylinder_6', 'Cylinder_7', 'Cylinder_8',
                 'Cylinder_9', 'GreenBox_1', 'GreenBox_2', 'GreenBox_3',
                 'GreenBox_4', 'GreenBox_5', 'HOME']
    for landmark in landmarks:
        req = tbot_msgs.srv.GetDistanceRequest()
        req.name = landmark
        resp = get_distance(req)
        print('{}: {}'.format(landmark, resp.distance))

if __name__ == '__main__':
    main()

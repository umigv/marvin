#!/usr/bin/env python3

import rospy

from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2

def callback(data: PointCloud2):
    for point in point_cloud2.read_points(data, ["intensity"]):
        if (point[0] != 1):
            rospy.loginfo(point)

if __name__=="__main__":
    rospy.init_node('point_cloud_intensities')
    rospy.Subscriber("/points2", PointCloud2, callback)
    rospy.spin()
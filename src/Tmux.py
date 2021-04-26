#!/usr/bin/env python
import rospy
from sensor_msgs.msg import PointCloud, PointCloud2
import numpy as np
import ros_numpy


def get_pointcloud2():
    rospy.init_node('lidar_xy', anonymous=True)
    rospy.Subscriber('/PointCloud2', PointCloud2, get_point_cartesian)
    rospy.spin()

def get_point_cartesian(data):
    pc = ros_numpy.numpify(data)
    points = np.zeros((pc.shape [0], 3))

    points[:,0] = pc['x']
    points[:,1] = pc['y']
    points[:,2] = pc['z']

    print(points)

if __name__ == '__main__':
    try:
        get_pointcloud2()
    except rospy.ROSInterruptException:
        pass
    
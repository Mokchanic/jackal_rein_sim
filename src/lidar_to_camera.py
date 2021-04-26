#!/usr/bin/env python
import rospy
import numpy as np
import ros_numpy
import sys
import camera_utils

import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes

import sensor_msgs.msg
from sensor_msgs.msg import PointCloud, PointCloud2, CameraInfo, Image

class BoxLidar:
    def __init__(self):
        self.img_pub = rospy.Publisher('laser_img', Image, queue_size=1000)
        
        self.bridge = CvBridge()
        self.cam_sub = rospy.Subscriber('/camera/color/camera_info', CameraInfo,self.get_camera)
        self.img_sub = rospy.Subscriber('/camera/color/image_raw', Image)
        self.yolo_sub = rospy.Subscriber('darknet_ros/bounding_boxes', BoundingBoxes,)
        self.point_sub = rospy.Subscriber('/PointCloud2', PointCloud2,)

    def get_camera(self, data):
        # get focal_length
        self.pd = data.K
        lst = [self.pd]
        array1d = np.array(lst)
        array2d = array1d.reshape(3,3)
        self.pd_matrix = array2d[0,0]

    def get_point_cartesian(self, point):
        self.pc = ros_numpy.numpify(point)
        points = np.zeros((self.pc.shape [0], 3))

        points[0:,0] = self.pc['x']
        points[0:,1] = self.pc['y']
        points[0:,2] = self.pc['z']
        # points[0:,3] = [1]

        #calculate image axis
        i = 0
        row = int(format(len(points))) #row data

        for i in range(0, row):
            self.a = points[i,:]
            self.b = np.array(self.a, ndmin=2).T
            
            self.lidar_x = self.b[0,0]
            self.lidar_y = self.b[1,0]
            self.lidar_z = self.b[2,0]

            self.image_point_x = self.pd_matrix * self.lidar_y / self.lidar_x
            self.image_point_y = self.pd_matrix * self.lidar_z / self.lidar_x
            print(360-self.image_point_x)
            print(240-self.image_point_y)



if __name__ == '__main__':
    rospy.init_node('lidar_to_camera_node', anonymous=True)
    
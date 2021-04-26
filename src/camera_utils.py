#! /usr/bin/env python

import rospy
import numpy as np
import math
# camera info
from sensor_msgs.msg import CameraInfo
from collections import defaultdict

# transform 
import tf2_ros
from geometry_msgs.msg import TransformStamped 

class CameraProjection():
    # camera coordinate transformation

    def __init__(self):
        # get data from camera info 
        self.image_width_ = 640
        self.image_height_ = 480
        self.distortion_param = [0.0, 0.0, 0.0, 0.0, 0.0]
        #simulation
        self.intrinsic_camera_matrix = [462.1379497504639, 0.0, 320.5, 0.0, 462.1379497504639, 240.5, 0.0, 0.0, 1.0]
        self.projection_camera_matrix = [462.1379497504639, 0.0, 320.5, 0.0, 0.0, 462.1379497504639, 240.5, 0.0, 0.0, 0.0, 1.0, 0.0]

        

        #realsense
        #self.intrinsic_camera_matrix = [606.9410400390625, 0.0, 320.7190246582031, 0.0, 605.557861328125, 231.009033203125, 0.0, 0.0, 1.0]
        #self.projection_camera_matrix = [606.9410400390625, 0.0, 320.7190246582031, 0.0, 0.0, 605.557861328125, 231.009033203125, 0.0, 0.0, 0.0, 1.0, 0.0]
    
    # fail
    # def camera_info_(self):
    #     self.camera_info_dict = {}
    #     self.camera_info_dict['image_width'] = self.image_width_
    #     self.camera_info_dict['image_height'] = self.image_height_
    #     self.camera_info_dict['distortion_param'] = self.distortion_param
    #     self.camera_info_dict['intrinsic_camera_matrix'] = self.intrinsic_camera_matrix
    #     self.camera_info_dict['projection_camera_matrix'] = self.projection_camera_matrix

    #     return self.camera_info_dict

    def get_camera_matrix(self):
        pass

    def pixel_to_image(self):
        pass
    
    def image_to_pixel(self,data):
        pass


    def camera_to_image(self, X,Y,Z):
        # data = [X Y Z 1] is it right?
        camera_matrix = np.reshape(self.projection_camera_matrix,(3,4))
        camera_matrix[0][2] = 0
        camera_matrix[1][2] = 0
        #self.ratio = ((self.projection_camera_matrix[0]) * (Y/ X)) / self.image_width_ 
        self.ratio = float(1/ X) 
        posi_ = np.array([X ,Y ,Z ,1]) 
        
        image_posi = np.dot(camera_matrix, np.transpose(posi_)) * self.ratio # [x,y,1]
        
        return image_posi



    def image_to_camera(self, data): # data = [x, y, z] of image
        for i in range(len(data)):
            self.x_camera = float(data[i][0] * data[i][2] / self.projection_camera_matrix[0])
            self.y_camera = float(data[i][1] * data[i][2] / self.projection_camera_matrix[0]) # focal length
        
            if 0 < data[i][2] < 3.0:
                self.z_camera = data[i][2]
            elif data[i][2] <= 0:
                self.z_camera = 0.0
            else:
                self.z_camera = 10.0

        return self.x_camera, self.y_camera, self.z_camera

    def camera_to_world(self, data):
        pass

    
    def lidar_to_camera(self, data):
        pass

        
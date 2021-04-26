#!/usr/bin/env python

import rospy
import numpy as np
import darknet_ros_msgs.msg
from darknet_ros_msgs.msg import BoundingBoxes

import tf
from tf import TransformBroadcaster

import tf2_ros
import tf_conversions

import sensor_msgs.msg
from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import Pose, Vector3


class BoxPose:
    def __init__(self):
        self.height_ = 0
        self.width_ = 0
        self.focal_matrix = 0
        self.final_matrix = 0
        self.real_x = 0.0
        self.real_y = 0.0
        self.select_depth_data = 5 #give_depth

    def get_matrix(self, pixel):
        self.height_ = pixel.height
        self.width_ = pixel.width
        self.focal_matrix = pixel.K

        #focal_matrix
        lst = [self.focal_matrix]
        array1d = np.array(lst)
        array2d = array1d.reshape(3,3)
        self.final_matrix = array2d[0,0]
        return self.final_matrix

 
    def get_center_pose(self, data):
        br = TransformBroadcaster()
        rate = rospy.Rate(10)
        
        for index_, box in enumerate(data.bounding_boxes):

            self.camera_focal_length = self.final_matrix   
            #find image plane
            self.x_img_pos = (self.width_ /2)  - (box.xmin + box.xmax) / 2
            self.y_img_pos = (self.height_ / 2) - (box.ymin + box.ymax) / 2
            #calculate ratio
            print((box.xmin + box.xmax) / 2)
            
            try:
                self.real_x = self.x_img_pos*float(self.select_depth_data)/float(self.camera_focal_length)
                self.real_y = self.y_img_pos*float(self.select_depth_data)/float(self.camera_focal_length)
            except ZeroDivisionError:
                print("ZeroDivision")


            translation = [self.select_depth_data, self.real_x, self.real_y]
            rotation = (0.0, 0.0, 0.0, 1.0)
                
                
            tf_id = str(index_)            
            
            br.sendTransform(translation, rotation, rospy.Time.now(), tf_id , '/camera_link')
            br = TransformBroadcaster()
            rate.sleep()

    def pub_get_center_vector():
        pub = rospy.Publisher('vector', Vector3, queue_size=10)

        vec_ = Vector3()

        vec_.position.x = vector[0]
        vec_.position.x = vector[1]
        vec_.position.x = vector[2]
        pub.publish(pose_)
            


    def publisher():
        pub = rospy.Publisher('axis', Pose, queue_size=10)
                        
        pose_= Pose()
                
        pose_.position.x = translation[0]
        pose_.position.y = translation[1]
        pose_.position.z = translation[2]
        pub.publish(pose_)

if __name__ == '__main__':
    rospy.init_node('Box_posi_node', anonymous=True)

    boxpose = BoxPose()
    rospy.Subscriber('/camera/color/camera_info', sensor_msgs.msg.CameraInfo, boxpose.get_matrix)
    rospy.Subscriber('/darknet_ros/bounding_boxes', darknet_ros_msgs.msg.BoundingBoxes, boxpose.get_center_pose)
    boxpose.publisher
    boxpose.pub_get_center_vector

    rospy.spin()
#! /usr/bin/env python
import sys
import rospy
import numpy as np
import ros_numpy # convert msg to np.array : https://github.com/eric-wieser/ros_numpy
import camera_utils


import cv2
from cv_bridge import CvBridge, CvBridgeError
from darknet_ros_msgs.msg import BoundingBoxes
from sensor_msgs.msg import PointCloud2

from sensor_msgs.msg import Image
from sensor_msgs.msg import LaserScan




# lidar --> camera coordinate

# camera --> image coordinate 

# image show point data 

# if you want to run this code you need to get 3 kind of msg
# 1. Image
# 2. BoundingBoxes (darknet_ros)
# 3. PointCloud2 
class image_converter:
    
    def __init__(self):
        self.img_pub = rospy.Publisher('laser_img',Image, queue_size=1000)

        self.bridge = CvBridge()
        self.img_sub = rospy.Subscriber('/camera/color/image_raw',Image,self.callback_camera)
        self.yolo_sub = rospy.Subscriber('/darknet_ros/bounding_boxes',BoundingBoxes,self.callback_yolo)
        self.laser_sub = rospy.Subscriber('/PointCloud2',PointCloud2, self.callback_lidar) #woon bong's --> rosrun cloud trans

        self.camera_utils_ = camera_utils.CameraProjection()


        self.image_width_ = 640
        self.image_height_ = 480
        self.focal_length = 462.1379497504639


    def callback_yolo(self, data):
        try:
            self.boxes = data.bounding_boxes
        except rospy.ROSInterruptException:
            pass

    def callback_lidar(self, data):
        try:
            self.pc2 = ros_numpy.numpify(data)
        except rospy.ROSInterruptException:
            pass
        
        self.points = np.zeros((self.pc2.shape[0],3))
        self.points[:,0] = self.pc2['x'] 
        self.points[:,1] = self.pc2['y']
        self.points[:,2] = self.pc2['z']

    def callback_camera(self, data):
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        (self.rows, self.cols, channels) = cv_img.shape
        
        for box in self.boxes:
            cv2.rectangle(cv_img, (box.xmin,box.ymin), (box.xmax, box.ymax), (0,255,0), 1)
            self.x_center = int(((self.cols /2)  - (box.xmin + box.xmax) / 2) + (self.cols/2)) # image to pixel (u,v)
            self.y_center = int(((self.rows / 2) - (box.ymin + box.ymax) / 2) + (self.rows/2))
            
            cv2.line(cv_img, (self.x_center,self.y_center),(self.x_center,self.y_center), (255,0,0), 5)
        
        for point in self.points:
            # 1. lidar to camera : need transformation from lidar to camera coordinate

            # 2. camera to image
            self.img_posi = self.camera_utils_.camera_to_image(point[0], point[1], point[2]) #* self.camera_utils_.ratio
            
            #print(self.img_posi)
            self.pixel_posi = (self.img_posi + [0, (self.cols /2 ), 0]) #* self.camera_utils_.ratio# ratio# [depth , pixel_v, pixel_u]
            #print(self.camera_utils_.ratio)
            #print(self.pixel_posi)
            cv2.line(cv_img, (int(self.pixel_posi[1]),int(self.pixel_posi[2]+240)), (int(self.pixel_posi[1]), int(self.pixel_posi[2]+240)), (255,0,0), 10) # 240 is for v pixel so you have to change!!!! 
            
        
        cv2.imshow("Image window", cv_img)
        cv2.waitKey(3)

        try:
            self.img_pub.publish(self.bridge.cv2_to_imgmsg(cv_img, "bgr8"))
        except CvBridgeError as e:
            print(e)




def main(args):
    ic = image_converter()
    rospy.init_node('image_converter', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutdown")
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main(sys.argv)
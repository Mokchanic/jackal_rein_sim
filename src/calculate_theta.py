#!/usr/bin/env python
import rospy
import numpy as np
import math

from geometry_msgs.msg import Pose

from std_msgs.msg import Int16 

# def __init__(self):
#     self.fov = rospy.get_param('fov')

class GetAngle:

    def theta(axis):
        pi = math.pi
        fov = 70

        pose_x = axis.position.x
        pose_y = axis.position.y
        
        #get_angle
        angle = np.arctan2(pose_y, pose_x)
        rospy.loginfo(angle)

        # #fov_range
        # angle_max = (fov/2)*pi/180
        # angle_min = (-fov/2)*pi/180

        # if angle_min <= angle <= angle_max:
        #     rospy.loginfo(angle)
        # else:
        #     pass
    def ang_publisher():
        pub = rospy.Publisher('theta', Int16, queue_size=10)
        fi_theta_ = Int16()

        fi_theta_int = angle
        pub.publish(fi_theta_)



    if __name__ == '__main__':
        rospy.init_node('cal_theta', anonymous=True)
        getangle = GetAngle()
        
        rospy.Subscriber("axis", Pose, theta)
        rospy.spin()
        
        getangle.pose_subscriber
        getangle.findtheta
        getangle.ang_publisher
#! /usr/bin/env python 
import tf
import numpy as np
import math


def euler_to_quaternion(roll,pitch,yaw):
    # get [x,y,z,w] return list 
    return tf.transformations.quaternion_from_euler(roll,pitch,yaw)


def quaternion_to_euler(quaternion):
     # list [x,y,z,w] return [roll,pitch,yaw]
    return tf.transformations.euler_from_quaternion(quaternion)
    


def cartesian_to_angle(x,y,z):
    PI = math.pi
    roll = np.arctan2(-y, z)
    pitch = np.arctan2(x,z)
    yaw = np.arctan2(y,x)

    return [roll,pitch,yaw]

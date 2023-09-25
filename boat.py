import rclpy
import rospy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import math
import torch
import time
import ultralytics.models.yolo
import cv2
from mavros_msgs.msg import OverrideRCIn

flag = True
img_centre_x = 416 / 2
throttle_channel = 3  
steering_channel = 1 
rudder_channel = 2

def area_comp (coord) :
    area = abs(coord[0] - coord[2]) * abs(coord[1] * coord[3])

    return area

class Node:
    def __init__(self):
        self.boundingboxes = rospy.Subsciber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback)
        self.thrust_control = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)

        self.rc_msg = OverrideRCIn()
    
    def bounding_callback(self, boxes):
        bb_tuples = []
        for box in boxes:
            x_min = box.xmin
            y_min = box.ymin
            x_max = box.xmax
            y_max = box.ymax
            bb_tuples.append((x_min, y_min, x_max, y_max))
        
        sorted_bb_tuples = sorted(bb_tuples, key=area_comp, reverse=True)
        bb_centres = [((x1 + x2) / 2, (y1 + y2) / 2) for (x1, y1, x2, y2) in sorted_bb_tuples]

        if len(bb_centres == 0):
            if (flag):
                flag = False
                self.rc_msg.channels[rudder_channel - 1] = 0
                self.rc_msg.channels[throttle_channel - 1] = 1230
                time.sleep(1.2)
            else:
                self.rc_msg.channels[rudder_channel - 1] = 0
                self.rc_msg.channels[throttle_channel - 1] = 1110
            return

        req_obj = bb_centres[0]


        if (abs(img_centre_x - req_obj[0]) > 80.0):
            pwm_speed = 1500 + (img_centre_x - req_obj[0])
            self.rc_msg.channels[steering_channel - 1] = pwm_speed
            self.rc_msg.channels[rudder_channel - 1] = (img_centre_x - req_obj[0]) / 10
        else:
            self.rc_msg.channels[rudder_channel - 1] = 0
            self.rc_msg.channels[throttle_channel - 1] = 1250
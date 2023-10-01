import rospy
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import torch
import time
import ultralytics.models.yolo
from mavros_msgs.msg import OverrideRCIn

flag = True
img_centre_x = 416 / 2     #------------image_centre_x is in image_callback also-----------
throttle_channel = 3  
steering_channel = 1 
rudder_channel = 2

def area_comp (coord) :
    area = abs(coord[0] - coord[2]) * abs(coord[1] * coord[3])

    return area

class Node:
    def __init__(self):
        rospy.init_node('boat_control')
        # self.boundingboxes = rospy.Subsciber("/darknet_ros/bounding_boxes", BoundingBoxes, self.bounding_callback)

        #Check Subsciber topic name (self.image_sub)
        self.thrust_control = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        self.cv_bridge = CvBridge()

        self.model = torch.hub.load('/home/jetson/yolov5', 'custom', path='/home/jetson/yolov5/best.pt', source='local')
        
        self.rc_msg = OverrideRCIn()
        self.current_image = None
    
    def image_callback(self, image_data) :
    	self.current_image = image_data
    
    def move(self):
    	while True : 
			global flag
			print("hello")
			img_data = self.current_image
			cv_image = self.cv_bridge.imgmsg_to_cv2(img_data)
			result_image = self.model(cv_image)
			predict_image = result_image.xyxy[0]
			img_size = result_image.ims[0].shape[:]
			# img_centre_y = img_size[0] / 2
			img_centre_x = img_size[1] / 2   #------------image_centre_x is in defined globally also-----------
			#Finds coordinates of the bounding box
			bb_tuples = []
			for img in predict_image:
				x1, y1, x2, y2, _, _ = img.tolist()
				bb_tuples.append((x1, y1, x2, y2))

			sorted_bb_tuples = sorted(bb_tuples, key=area_comp, reverse=True)
			bb_centres = [((x1 + x2) / 2, (y1 + y2) / 2) for (x1, y1, x2, y2) in sorted_bb_tuples]

			#If there are no bounding boxes...
			if (len(bb_centres) == 0):
				if (flag):
					flag = False
					self.rc_msg.channels[rudder_channel - 1] = 0
					self.rc_msg.channels[throttle_channel - 1] = 1230
					time.sleep(1.2)
				else:
					self.rc_msg.channels[rudder_channel - 1] = 0
					self.rc_msg.channels[throttle_channel - 1] = 1110
				return

			#Selects the centre of the box with largest area...
			req_obj = bb_centres[0]

			#Aligns the boat with the detected object...Threshold is 80 pixels...
			if (abs(img_centre_x - req_obj[0]) > 80.0):
				pwm_speed = 1500 + (img_centre_x - req_obj[0])
				self.rc_msg.channels[steering_channel - 1] = pwm_speed
				self.rc_msg.channels[rudder_channel - 1] = (img_centre_x - req_obj[0]) / 10
				self.thrust_control.publish(self.rc_msg)
			else:
				self.rc_msg.channels[rudder_channel - 1] = 0
				self.rc_msg.channels[steering_channel - 1] = 0 #Recently added...
				self.rc_msg.channels[throttle_channel - 1] = 1250
				self.thrust_control.publish(self.rc_msg)
            
new_node = Node()
time.sleep(10)
new_node.move()
rospy.spin()

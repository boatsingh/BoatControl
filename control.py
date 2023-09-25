import rclpy
import rospy
from rclpy.node import Node
from mavros_msgs.msg import OverrideRCIn
from sensor_msgs.msg import Image, Imu
from cv_bridge import CvBridge
import math
import torch
import time
import ultralytics.models.yolo
import cv2
# from tf_transformations import euler_from_quaternion 

#Minimum thrust
zero_msg = Float64()
zero_msg.data = 0.0

#Maximum thrust
msg = Float64()
msg.data = 75.0

# This function converts from quaternion to euler
# def quaternion_to_euler_angle(w, x, y, z):
#     ysqr = y * y

#     t0 = +2.0 * (w * x + y * z)
#     t1 = +1.0 - 2.0 * (x * x + ysqr)
#     X = math.degrees(math.atan2(t0, t1))

#     t2 = +2.0 * (w * y - z * x)
#     t2 = +1.0 if t2 > +1.0 else t2
#     t2 = -1.0 if t2 < -1.0 else t2
#     Y = math.degrees(math.asin(t2))

#     t3 = +2.0 * (w * z + x * y)
#     t4 = +1.0 - 2.0 * (ysqr + z * z)
#     Z = math.degrees(math.atan2(t3, t4))

#     return [X, Y, Z]

#To find area of bounding box
def area_comp (coord) :
    area = abs(coord[0] - coord[2]) * abs(coord[1] * coord[3])
    
    return area


class MinimalPublisher(Node):

    def __init__(self):
        rospy.init_node('boat_control')
        # self.publisherL = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        # self.publisherR = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.thruster_control = rospy.Publisher("/mavros/rc/override", OverrideRCIn, queue_size=10)
        # self.imu_sub = self.create_subscription(Imu, '/wamv/sensors/imu/imu/data', self.imu_callback, 10)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.image_callback)
        # self.initial_yaw = None
        # self.current_yaw = None
        # self.set_yaw_flag = True
        # self.go_straight_flag = True
        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
        self.cv_bridge = CvBridge()
        self.model = torch.hub.load('/home/mohammad/boat_thrust/src/boat/boat/yolov5', 'custom', path='/home/mohammad/boat_thrust/src/boat/boat/best.pt', source='local')

    # def timer_callback(self):
    #     msg = Float64()
    #     msg.data = 10000.0
    #     self.publisherL.publish(msg)
    #     self.publisherR.publish(msg)
    #     # self.get_logger().info('Publishing: "%d"' % msg.data)
    #     self.i += 1

    # def imu_callback (self, imu_data):
    #     euler_orientation = quaternion_to_euler_angle(imu_data.orientation.w, imu_data.orientation.x, imu_data.orientation.y, imu_data.orientation.z)
    #     if (self.set_yaw_flag):
    #         self.initial_yaw = euler_orientation[2]
    #         self.current_yaw = euler_orientation[2]
    #         self.set_yaw_flag = False
    #     else:
    #         self.current_yaw = euler_orientation[2]

    def image_callback(self, img_data):  

        
        cv_image = self.cv_bridge.imgmsg_to_cv2(img_data)
        result_image = self.model(cv_image)
        predict_image = result_image.xyxy[0]
        img_size = result_image.ims[0].shape[:]
        img_centre_y = img_size[0] / 2
        img_centre_x = img_size[1] / 2

        #Finds coordinates of the bounding box
        bb_tuples = []
        for img in predict_image:
            x1, y1, x2, y2, _, _ = img.tolist()
            bb_tuples.append((x1, y1, x2, y2))
        
        # if (len(bb_tuples)>1):
        sorted_bb_tuples = sorted(bb_tuples, key=area_comp, reverse=True)
        # else:
        #     sorted_bb_tuples = bb_tuples
        bb_centres = [((x1 + x2) / 2, (y1 + y2) / 2) for (x1, y1, x2, y2) in sorted_bb_tuples]
        
        result_image.show()
        cv2.waitKey(0)

        #If there are no objects to detect
        if (len(bb_centres) == 0):
            
            if (abs(self.current_yaw - self.initial_yaw) < 0.1):
                self.go_straight_flag = True

            if (self.go_straight_flag):
                self.go_straight_flag = False
                self.publisherR.publish(msg)
                self.publisherL.publish(msg)
                time.sleep(3)
            else:
                zero_obj_turn_msg = Float64()
                zero_obj_turn_msg.data = msg.data / 4.0
                self.publisherR.publish(msg)
                self.publisherL.publish(zero_obj_turn_msg)
            return
        
        req_obj = bb_centres[0]
        
        #If the objects are detected.
        if (abs(img_centre_x - req_obj[0]) > 80.0):
            tmp_msg = Float64()
            tmp_msg.data = msg.data * (abs(img_centre_x - req_obj[0]) / 100.0 )
            turn_msg = Float64()
            turn_msg.data = tmp_msg.data / 4.0

            if ((img_centre_x - req_obj[0]) > 0 ):
                self.publisherR.publish(tmp_msg)
                self.publisherL.publish(turn_msg)
            else:
                self.publisherL.publish(tmp_msg)
                self.publisherR.publish(turn_msg)
        else:
            self.publisherR.publish(msg)
            self.publisherL.publish(msg)
        
        # print(img_size)
        # print(bb_centres)
        


def main(args=None):
    rclpy.init(args=args)
    minimal_publisher = MinimalPublisher()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    # minimal_publisher.destroy_node()
    # rclpy.shutdown()


if __name__ == '__main__':
    main()
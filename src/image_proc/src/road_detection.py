#!/usr/bin/env python
import cv2
import numpy as np
import rospy, cv_bridge
from sensor_msgs.msg import Image
from std_msgs.msg import Float32


class ImageProcessing:
    
    def __init__(self):
        self.frameWidth = 640
        self.frameHeight = 480

        self.num_red_pub = rospy.Publisher('/road', Float32, queue_size = 1)

    def process_image(self, image):
        # Make a copy of the image
        image_copy = np.copy(image)

        #Resize
        image_copy = cv2.resize(image_copy, (self.frameWidth, self.frameHeight))

        image_copy = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)

        height_i = int(self.frameHeight*0.75)

        sub_image = image_copy[height_i:self.frameHeight, 0:self.frameWidth]

        sum_columnas = np.sum(sub_image == 255, axis = 0)

        road = np.argmax(sum_columnas)

        return road




class CameraControl:

    def __init__(self):
        self.bridge = cv_bridge.CvBridge()

        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)

        self.image = None
        self.rate = rospy.Rate(30)
        self.im_proc = ImageProcessing()


    def image_callback(self, msg):
        self.image = self.bridge.imgmsg_to_cv2(msg, desired_encoding = 'bgr8')

    def run(self):
        if self.image is None:
            return

        road = self.im_proc.process_image(self.image)
        self.im_proc.num_red_pub.publish(road)


if __name__ == "__main__":
    rospy.init_node("road_detection")
    cameraControl = CameraControl()

    while not rospy.is_shutdown():
        cameraControl.run()
        cameraControl.rate.sleep()

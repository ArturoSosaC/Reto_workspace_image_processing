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

        self.num_red_pub = rospy.Publisher('/num_red', Float32, queue_size = 1)
    
    def process_image(self, image):

        # Make a copy of the image
        image_copy = np.copy(image)

        #Resize
        image_copy = cv2.resize(image_copy, (self.frameWidth, self.frameHeight))

        # Change color to RGB (from BGR)
        image_copy = cv2.cvtColor(image_copy, cv2.COLOR_BGR2RGB)

        #Define color selection boundaries in RGB values
        lower_red = np.array([180,0,0]) 
        upper_red = np.array([255,70,70])

        # Define the masked area
        mask = cv2.inRange(image_copy, lower_red, upper_red)

        # Contar pixeles blancos
        num_red = np.sum(mask == 255)

        # Porcentaje
        total_pixeles = self.frameHeight*self.frameWidth
        num_red = num_red*100/total_pixeles

        return num_red

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

        num_red = self.im_proc.process_image(self.image)
        self.im_proc.num_red_pub.publish(num_red)


if __name__ == "__main__":
    rospy.init_node("color_detection")
    cameraControl = CameraControl()

    while not rospy.is_shutdown():
        cameraControl.run()
        cameraControl.rate.sleep()


#!/usr/bin/env python
import numpy as np
import rospy, cv_bridge
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist


class SpeedControl:

    def __init__(self):
        
        self.kp = 0.004
        self.kd = 10

        self.deltaT = 0.0
        self.eprev = 0.0
        self.currT = 0.0
        self.prevT = 0.0

        self.red_min = 20.0
        self.white_middle = 320

        self.speed_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)



    def process_color(self, red_per, white_offset):

        speed = Twist()

        e = self.white_middle - white_offset

        self.currT = rospy.get_time()
        deltaT = (self.currT-self.prevT)/(pow(10,6))
        self.prevT = self.currT
        edt = (e - self.eprev)/deltaT 
        self.eprev = e

        controller = self.kp*e + self.kd*edt
    
        if red_per > self.red_min:
            speed.linear.x = 0.0
            speed.angular.z = 0.0
        else:
            speed.linear.x = 0.3
            speed.angular.z = controller

        return speed


class ColorControl:

    def __init__(self):

        self.red_sub = rospy.Subscriber('/num_red', Float32, self.red_callback)
        self.white_sub = rospy.Subscriber('/road', Float32, self.white_callback)

        self.red_msg = None
        self.white_msg = None

        self.rate = rospy.Rate(30)
        self.sp_control = SpeedControl()

    def red_callback(self, msg):
        self.red_msg = msg.data

    def white_callback(self, msg):
        self.white_msg = msg.data

    def run(self):
        if self.red_msg is None or self.white_msg is None:
            return

        speed = self.sp_control.process_color(self.red_msg, self.white_msg)
        self.sp_control.speed_pub.publish(speed)


if __name__ == "__main__":
    rospy.init_node("control")
    colorControl = ColorControl()

    while not rospy.is_shutdown():
        colorControl.run()
        colorControl.rate.sleep()








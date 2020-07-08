#!/usr/bin/env python
#Written By Davis Landry | 7/2020
import rospy
from controller.msg import Depth
from ackermann_msgs.msg import AckermannDriveStamped as drive_cmd


#Basic Proportional Derivative Controller
class drive:
    def __init__(self):
        self.command_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        self.depth_topic = '/depth_data'
        self.cmd_pub = drive_cmd()
        self.angle_diff_prev = 0
        #Proportional Gain
        self.kp = 0.18
        #Derivative Gain
        self.kd = 0.1
        #Scaling Factyor
        self.scale = 1

        #create publisher passing it the command_topic
        self.pub = rospy.Publisher(self.command_topic, drive_cmd, queue_size = 1)

        #create subscriber to the depth topic
        self.depth_sub = rospy.Subscriber(self.depth_topic, Depth, self.scan_callback)


    def scan_callback(self, data):
        #Data has left, right and center

        #Positive means car must turn left, negative car must turn right
        self.angle_diff = data.left - data.right

        #Calculate Proportional
        self.prop = self.kp * self.angle_diff
        #Calculate Derivative
        self.deriv = self.kd * ((self.angle_diff - self.angle_diff_prev)*30)
        self.angle_diff_prev = self.angle_diff
        #Combine Proportional and Derivative Parts
        self.steer_angle = (self.prop + self.deriv)*self.scale
        #Set limit on how hard car can turn
        if self.steer_angle < -0.15:
            self.steer_angle  = -0.15
        elif self.steer_angle > 0.15:
            self.steer_angle = 0.15
        self.cmd_pub.drive.steering_angle =  self.steer_angle
        rospy.loginfo(self.cmd_pub.drive.steering_angle)

        #Speed of 2.0 works, but this control scheme breaks down as the car goes faster
        #change this value to 5.0 and the car will no longer register the turn.
        self.cmd_pub.drive.speed = 2.0

        self.pub.publish(self.cmd_pub)


if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive()
	rospy.spin()

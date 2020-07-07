#!/usr/bin/env python
import rospy
from controller.msg import Depth
from ackermann_msgs.msg import AckermannDriveStamped as drive_cmd



class drive:
    def __init__(self):
        self.command_topic = '/vesc/high_level/ackermann_cmd_mux/input/nav_0'
        self.depth_topic = '/depth_data'
        self.cmd_pub = drive_cmd()
        self.angle_diff_prev = 0
        self.kp = 0.18 #.18
        self.kd = 0
        self.scale = 1

        #create publisher passing it the vel_topic_name and msg_type
        self.pub = rospy.Publisher(self.command_topic, drive_cmd, queue_size = 1)

        #create subscriber
        self.depth_sub = rospy.Subscriber(self.depth_topic, Depth, self.scan_callback)


    def scan_callback(self, data):


        self.left = data.left
        self.right = data.right
        self.center = data.center

        self.angle_diff = self.left - self.right
        #rospy.loginfo(self.angle_diff)

        self.prop = self.kp * self.angle_diff
        self.deriv = self.kd * (self.angle_diff_prev - self.angle_diff)
        self.angle_diff_prev = self.angle_diff
        self.steer_angle = (self.prop + self.deriv)*self.scale
        if self.steer_angle < -0.15:
            self.steer_angle  = -0.15
        elif self.steer_angle > 0.15:
            self.steer_angle = 0.15
        self.cmd_pub.drive.steering_angle =  self.steer_angle
        rospy.loginfo(self.cmd_pub.drive.steering_angle)
        self.cmd_pub.drive.speed = 2.0

        self.pub.publish(self.cmd_pub)


if __name__ == "__main__":
	rospy.init_node('drive_node')
	obj = drive()
	rospy.spin()

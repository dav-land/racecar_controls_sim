#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from controller.msg import Depth
from cv_bridge import CvBridge, CvBridgeError
import sys
import os

class ImageListener:
    def __init__(self, topic):
        self.topic = topic
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(topic, Image, self.imageDepthCallback)
        self.pub = rospy.Publisher('/depth_data', Depth, queue_size=1)
        # self.img_pub = rospy.Publisher('/depth_image', Image, queue_size=1)
        self.pub_msg = Depth()

    def imageDepthCallback(self, data):

        try:
            clip = 7.0
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
            cv_image[cv_image == 1.0] = clip
            center_pixel = (data.width/2, data.height/2)
            vertical_pixel = center_pixel[1]

            self.right = [cv_image[vertical_pixel, x] for x in range(550,600) ]
            self.right = sum(self.right)/len(self.right)

            self.left = [  cv_image[vertical_pixel , x] for x in range(50, 100) ]
            self.left = sum(self.left)/len(self.left)
            self.center = [ cv_image[vertical_pixel, x] for x in range(300,340)]

            # rospy.loginfo(self.center)
            self.center = sum(self.center)/len(self.center)

            self.pub_msg.left = self.left
            self.pub_msg.right = self.right
            self.pub_msg.center = self.center
            #rospy.loginfo(self.pub_msg)
            self.pub.publish(self.pub_msg)
            #rospy.sleep(0.0333)

        except CvBridgeError as err:
            print(err)
            return


def main():
    topic = '/camera/depth/image_raw'
    listener = ImageListener(topic)
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()

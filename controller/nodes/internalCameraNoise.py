#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys
import os
import numpy as np

class ImageListener:
    def __init__(self):
        self.topic = ''
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/depth/image_raw/internal', Image, self.imageDepthCallback)
        self.pub = rospy.Publisher('/camera/depth/image_raw', Image, queue_size=1)
        self.pub_msg = Image()

    def imageDepthCallback(self, data):

        try:
            clip = 7.0
            cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)

            # for y in range(data.width):
            #     dist = cv_image[240][y]
            #     z_error_std_dev = dist * dist * 0.003766478    #This is from stereo camera error function E_z = z^2/(bf) * E_d
            #
            #     error = np.random.normal(0,z_error_std_dev,(1,data.width))
            #     cv_image[:][y] = cv_image[:][y] + error


            self.pub_msg = self.bridge.cv2_to_imgmsg(cv_image)

            self.pub.publish(self.pub_msg)

        except CvBridgeError as err:
            print(err)
            return


def main():
    listener = ImageListener()
    rospy.spin()

if __name__ == '__main__':
    node_name = os.path.basename(sys.argv[0]).split('.')[0]
    rospy.init_node(node_name)
    main()

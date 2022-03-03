#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tf2_msgs.msg import TFMessage
rospy.init_node('test')

#座標を持ってくる
class tf():
    def __init__(self):
        rospy.Subscriber('/tf', TFMessage, self.tfCB)

    def tfCB(self, msg):
        pose_x = msg.transforms
        #pose_y = msg.transforms.translation.y
        #pose_z = msg.transforms.rotation.z
        #pose_w = msg.transforms.rotation.w
        print pose_x

if __name__ == '__main__':
    rospy.init_node('test')
    tf = tf()
    rospy.spin()

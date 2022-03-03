#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from tf import TransformListener

rospy.init_node('test')

#座標を持ってくる
tf = TransformListener()
position, rotation = tf.lookupTransform("/map", "/base_link", rospy.Time(0))
pose_x = position[0]
pose_y = position[1]
pose_z = rotation[2]
pose_w = rotation[3]

print pose_x, pose_y, pose_z, pose_w

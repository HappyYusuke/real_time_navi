#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------
# Title: リアルタイムで座標を設定し、Navigationもするサービスサーバー
# Author: Yusuke Kanazawa
# Date: 2022/03/03
# Memo:
#-------------------------------------

import rospy
import actionlib
from std_msgs.msg import Float64
from std_srvs.srv import Empty
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from tf import TransformListener
from real_time_navi.srv import RealTimeNavi, RealTimeNaviResponse

class RealTimeNaviServer():
    def __init__(self):
        rospy.Service('/realtime_navi_server', RealTimeNavi, self.checkState)
        rospy.loginfo("Ready to set_location_server")
        # Action
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Service
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # Value
        self.tf = TransformListener()
        self.location_list = []

    def checkState(self, srv_req):
        if srv_req.state == 'set':
            rospy.loginfo("Set location")
            return RealTimeNaviResponse(result = self.set_location())
        elif srv_req.state == 'navigation':
            rospy.loginfo("Executing Navigation")
            return RealTimeNaviResponse(result = self.navigation())
        else:
            rospy.logger("<" + srv_req.state + "> state doesn't exist.")
            return RealTimeNaviResponse(result = False)

    def set_location(self):
        position, rotation = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
        self.location_list = [position[0], position[1], rotation[2], rotation[3]]
        print self.location_list
        return RealTimeNaviResponse(result = True)

    def navigation(self, location_list):
        # set goal_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = location_list[0]
        goal.target_pose.pose.position.y = location_list[1]
        goal.target_pose.pose.orientation.z = location_list[2]
        goal.target_pose.pose.orientation.w = location_list[3]
        # clearing costmap
        rospy.loginfo("Clearing costmap...")
        rospy.wait_for_service('move_base/clear_costmaps')
        self.clear_costmap()
        rospy.sleep(0.5)
        # start navigation
        self.head_pub.publish(0)
        self.ac.wait_for_server()
        self.ac.send_goal(goal)
        navi_state = self.ac.get_state()
        while not rospy.is_shutdown():
            navi_state = self.ac.get_state()
            if navi_state == 3:
                rospy.loginfo('Navigation success!!')
                return RealTimeNaviResponse(result = True)
            elif navi_state == 4:
                rospy.loginfo('Navigation failed')
                return RealTimeNaviResponse(result = False)
            else:
                pass

if __name__ == '__main__':
    rospy.init_node('realtime_navi_node')
    try:
        rtns = RealTimeNaviServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

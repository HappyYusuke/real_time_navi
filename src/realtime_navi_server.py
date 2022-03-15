#!/usr/bin/env python
# -*- coding: utf-8 -*-
#-------------------------------------
# Title: リアルタイムでロケーションをセットし、Navigationもするサービスサーバー
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
        rospy.loginfo("Ready to realtime_navi_server")
        # Action
        self.ac = actionlib.SimpleActionClient('/move_base', MoveBaseAction)
        # Publisher
        self.head_pub = rospy.Publisher('/servo/head', Float64, queue_size = 1)
        # Service
        self.clear_costmap = rospy.ServiceProxy('/move_base/clear_costmaps', Empty)
        # Value
        self.tf = TransformListener()
        self.location_dict = {}
        self.realtime_location = []
        rospy.set_param('/real_time_navi/location_dict', self.location_dict)
        self.location_pose_x = 0.00
        self.location_pose_y = 0.00
        self.location_pose_z = 0.00
        self.location_pose_w = 0.00

    def setPosition(self, name):
        # get position
        position, rotation = self.tf.lookupTransform("/odom", "/base_link", rospy.Time(0))
        self.location_pose_x = position[0]
        self.location_pose_y = position[1]
        self.location_pose_z = rotation[2]
        self.location_pose_w = rotation[3]
        rospy.sleep(0.3)
        # set dictionary
        self.location_dict[name] = []
        self.location_dict[name].append(self.location_pose_x)
        self.location_dict[name].append(self.location_pose_y)
        self.location_dict[name].append(self.location_pose_z)
        self.location_dict[name].append(self.location_pose_w)

    def checkState(self, srv_req):
        if not srv_req.location_name == 'NULL':
            if srv_req.state == 'add':
                rospy.loginfo("Add location")
                return RealTimeNaviResponse(result = self.addLocation(srv_req.location_name))
            elif srv_req.state == 'update':
                rospy.loginfo("Update: '" + srv_req.location_name + "'")
                return RealTimeNaviResponse(result = self.updateLocation(srv_req.location_name))
            elif srv_req.state == 'navigation':
                rospy.loginfo("Start navigation")
                return RealTimeNaviResponse(result = self.sendGoal(srv_req.location_name))
            else:
                rospy.logerr("'" + srv_req.state + "' state doesn't exist.")
                return RealTimeNaviResponse(result = False)
        else:
            rospy.logerr("[location_name] is empty")
            return RealTimeNaviResponse(result = False)

    def addLocation(self, name):
        if name in self.location_dict:
            rospy.logerr("'" + name + "' has been registerd. Please enter a different name")
            return False
        else:
            self.getPosition()
            self.setDictionary(name)
            rospy.set_param('real_time_navi/location_dict', self.location_dict)
            print self.location_dict
            rospy.loginfo("Registerd '" + name + "'")
            return True

    def updateLocation(self, name):
        if name in self.location_dict:
            self.getPosition()
            self.setDictionary(name)
            rospy.set_param('/real_time_navi/location_dict', self.location_dict)
            print self.location_dict
            rospy.loginfo("Update completed '" + name + "'")
            return True
        else:
            rospy.logerr("'" + name + "' is not in [/real_time_navi/location_dict]")
            return False

    def sendGoal(self, name):
        # get param
        self.realtime_location = rospy.get_param('/real_time_navi/location_dict/' + name)
        # set goal_pose
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'odom'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose.position.x = self.realtime_location[0]
        goal.target_pose.pose.position.y = self.realtime_location[1]
        goal.target_pose.pose.orientation.z = self.realtime_location[2]
        goal.target_pose.pose.orientation.w = self.realtime_location[3]
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
        rospy.loginfo("During Navigation...")
        while not rospy.is_shutdown():
            navi_state = self.ac.get_state()
            if navi_state == 3:
                rospy.loginfo('Navigation success!!')
                return True
            elif navi_state == 4:
                rospy.loginfo('Navigation failed')
                return False
            else:
                pass


if __name__ == '__main__':
    rospy.init_node('realtime_navi_node')
    try:
        rtns = RealTimeNaviServer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

#!/usr/bin/env python
import rospy
from tf import TransformListener
from real_time_navi.srv import RealTimeNavi, RealTimeNaviResponse

class test2():
    def __init__(self):
        rospy.Service('/test2_server', RealTimeNavi, self.now_location)
        rospy.loginfo("Ready to test2_server")
        self.tf = TransformListener()

    def now_location(self, data):
        try:
            position, rotation = self.tf.lookupTransform("/map", "/base_link", rospy.Time(0))
            pose_x = position[0]
            pose_y = position[1]
            pose_z = rotation[2]
            pose_w = rotation[3]
            print data
            print position, rotation
            return RealTimeNaviResponse(result = True)
        except rospy.ROSInternalException:
            rospy.logger("!!Interrupt!!")
            return RealTimeNaviResponse(result = False)

if __name__ == '__main__':
    rospy.init_node('test2')
    t = test2()
    rospy.spin()

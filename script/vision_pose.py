#!/usr/bin/env python
#  Authors: Tarek Taha
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful, but
# WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
# or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License
# for more details.
#
# You should have received a copy of the GNU General Public License along
# with this program; if not, write to the Free Software Foundation, Inc.,
# 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA
# Updated: Tarek Taha : tarek.taha@kustar.ac.ae

import rospy
import thread
import threading
import time

from geometry_msgs.msg import PoseStamped, Quaternion, TransformStamped
from math import *
#from mavros.srv import CommandBool
from mavros.utils import *
from std_msgs.msg import Header
from std_msgs.msg import String
from tf.transformations import quaternion_from_euler
from visualization_msgs.msg import Marker


class Vision2FCU:

    def __init__(self):
        rospy.init_node('vision_pose2_fcu', anonymous=True)
        self.vision_pub      = rospy.Publisher('/mavros/vision_pose/pose', PoseStamped, queue_size=10)    
        self.vision_pose_sub = rospy.Subscriber('/visualization_marker', Marker, self.getData)
        self.currentPose  = PoseStamped()
        rate = rospy.Rate(10) 
        while not rospy.is_shutdown():
            rate.sleep()

    def getData(self,topic):
        self.currentPose.pose   = topic.pose
        self.currentPose.header = Header() 
        self.currentPose.header.frame_id = "uav_frame"
        self.currentPose.header.stamp = rospy.Time.now()
        self.vision_pub.publish(self.currentPose)
        #print self.currentPose.pose

if __name__ == '__main__':
    try:
        Vision2FCU()
    except rospy.ROSInterruptException:
        pass

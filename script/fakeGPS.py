#!/usr/bin/env python
# vim:set ts=4 sw=4 et:
#
# Copyright 2015 UAVenture AG.
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
#    - Changed topic names after re-factoring : https://github.com/mavlink/mavros/issues/233

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

class Setpoint:

    def __init__(self, pub, rospy):
        self.pub = pub
        self.rospy = rospy
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.currentx = 0.0
        self.currenty = 0.0
        self.currentz = 0.0
        self.currentxo = 0.0 
        self.currentyo = 0.0 
        self.currentzo = 0.0 
        self.currentwo = 1.0 
        try:
            thread.start_new_thread( self.navigate, () )
        except:
            print "Error: Unable to start thread"

        self.done = False
        self.done_evt = threading.Event()
        sub = rospy.Subscriber('/visualization_marker', Marker, self.getData)

    def navigate(self):
        rate = self.rospy.Rate(50) # 10hz
        
        msg = TransformStamped()
        msg.header = Header() 
        msg.header.frame_id = "usb_cam"
        msg.header.stamp = rospy.Time.now()

        while 1:

            msg.transform.translation.x = self.currentx
            msg.transform.translation.y = self.currenty
            msg.transform.translation.z = self.currentz
            msg.transform.rotation.x = self.currentxo
            msg.transform.rotation.y = self.currentyo
            msg.transform.rotation.z = self.currentzo
            msg.transform.rotation.w = self.currentwo
            '''
            msg.pose.position.x = self.currentx
            msg.pose.position.y = self.currenty
            msg.pose.position.z = self.currentz
            msg.pose.orientation.x = self.currentxo
            msg.pose.orientation.y = self.currentyo
            msg.pose.orientation.z = self.currentzo
            msg.pose.orientation.w = self.currentwo
            '''
            # For demo purposes we will lock yaw/heading to north.
            #yaw_degrees = 0  # North
            #yaw = radians(yaw_degrees)
            #quaternion = quaternion_from_euler(0, 0, yaw)
            #msg.pose.orientation = Quaternion(*quaternion)

            self.pub.publish(msg)

            rate.sleep()
    '''
    def set(self, x, y, z, delay=0, wait=True):
        self.done = False
        self.x = x
        self.y = y
        self.z = z

        if wait:
            rate = rospy.Rate(5)
            while not self.done:
                rate.sleep()
        
        time.sleep(delay)
    '''
    def getData(self, topic):
        self.currentx = topic.pose.position.x
        self.currenty = topic.pose.position.y
        self.currentz = topic.pose.position.z
        self.currentxo = topic.pose.orientation.x
        self.currentyo = topic.pose.orientation.y
        self.currentzo = topic.pose.orientation.z
        self.currentwo = topic.pose.orientation.w
    '''
    def reached(self, topic):
            
            #print topic.pose.position.z, self.z, abs(topic.pose.position.z - self.z)
            if abs(topic.pose.position.x - self.x) < 0.2 and abs(topic.pose.position.y - self.y) < 0.2 and abs(topic.pose.position.z - self.z) < 0.2:
                self.done = True
            print "Current Pose:",topic.pose.position.x,topic.pose.position.y,topic.pose.position.z
            print "Set Pose:",self.x,self.y,self.z
            self.done_evt.set()
    '''
def setpoint_demo():
    pub = rospy.Publisher('/mavros/mocap/tf', TransformStamped, queue_size=10)
    
    rospy.init_node('gpsData', anonymous=True)
    rate = rospy.Rate(10) 

    setpoint = Setpoint(pub, rospy)

    #print "move in x axis 1 meter "
    #setpoint.set(0.0, 1.0, 1.2, 0)
    
    
    while not rospy.is_shutdown():
      print "NOT MANUAL" 


if __name__ == '__main__':
    try:
        setpoint_demo()
    except rospy.ROSInterruptException:
        pass

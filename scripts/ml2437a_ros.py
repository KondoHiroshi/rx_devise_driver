#! /usr/bin/env python3

import sys
import time
sys.path.append("/home/amigos/ros/src/NASCORX_System-master/NASCORX_System")
from .device import ML2437A

import rospy
import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32


class ml2437a_controller(object):
    def __init__(self):
        self.pm = ML2437A.ml2437a()

        self.pub_power = rospy.Publisher("topic_pub_power", Float64, queue_size = 1)
        self.sub_power = rospy.Subscriber("topic_sub_power", Float64, self.power)
        self.pub.ave_onoff = rospy.Publisher("topic_pub_ave_onoff", Int32, queue_size = 1)
        self.sub_ave_onoff = rospy.Subscriber("topic_sub_ave_onoff", Int32, self.ave_onoff)
        self.pub.ave_count = rospy.Publisher("topic_pub_ave_count", Int32, queue_size = 1)
        self.sub_ave_count = rospy.Subscriber("topic_sub_ave_count", Int32, self.ave_onoff)

    def power(self):
        while not rospy.is_shutdown():
            ret = self.pm.measure()
            msg.data = float(ret)
            self.pub_power.publish(msg)
            continue

    def ave_onoff(self,q):
        self.pm.set_average_onoff(q.data)

        ret = self.quary_average_onoff()
        msg.data = int(ret)
        self.pub_ave_onoff.publish(msg)

    def ave_count(self):
        self.pm.set_average_count(q.data)
        ret = self.quary_average_count()
        msg.data = int(ret)
        self.pub_ave_count.publish(msg)


if __name__ == "__main__" :
    rospy.init_node("ml2437a")
    ctrl = ml2437a_controller()
    rospy.spin()

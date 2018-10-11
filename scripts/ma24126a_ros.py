#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
import serial
import time
import sys,os

sys.path.append("/home/amigos/ros/src")
import threading
import pymeasure


class ma24126a_controller(object):
    def __init__(self):

        self.pm = pymeasure.usbpm.open("ma24126a")
        self.rate = rospy.get_param("~rate")

        self.zero_set_flag = 0
        self.close_flag = 0
        self.power_flag = 0
        self.capt_flag = 0
        self.avemode_flag = 0

        self.pub_zeroset = rospy.Publisher("topic_pub_zeroset", String, queue_size = 1)
        self.sub_zeroset = rospy.Subscriber("topic_sub_zeroset", Float64, self.zero_set_switch)
        self.pub_close = rospy.Publisher("topic_pub_close", String, queue_size = 1)
        self.sub_close = rospy.Subscriber("topic_sub_close", Float64, self.close_switch)
        self.pub_power = rospy.Publisher("topic_pub_power", Float64, queue_size = 1)
        self.sub_power = rospy.Subscriber("topic_sub_power", Float64, self.power_switch)
        self.sub_change_avemode = rospy.Subscriber("topic_sub_change_avemode", Float64, self.avemode_switch)
        self.pub_change_avemode = rospy.Publisher("topic_pub_change_avemode", Float64, queue_size = 1)
        self.sub_change_capt = rospy.Subscriber("topic_sub_change_capt", Float64, self.capt_switch)
        self.pub_change_capt = rospy.Publisher("topic_pub_change_capt", Float64, queue_size = 1)

#flag

    def zero_set_switch(self,q):
        self.zero_set_flag = q.data
        return

    def close_switch(self,q):
        self.close_flag = q.data
        return

    def power_switch(self,q):
        self.power_flag = q.data
        return

    def capt_switch(self,q):
        self.capt = q.data
        self.capt_flag = 1
        return

    def avemode_switch(self,q):
        self.avemode = q.data
        self.avemode_flag = 1
        return

#main methond
    def zero_set(self):
        while not rospy.is_shutdown():
            if self.zero_set_flag == 0:
                time.sleep(self.rate)
                continue

            self.pm.zero_set()

            msg = String()
            msg.data = "OK"
            self.pub_zeroset.publish(msg)

            self.zero_set_flag = 0
            continue

    def power(self):
        while not rospy.is_shutdown():
            if self.power_flag == 0:
                continue

            if self.power_flag == 1:
                msg = Float64()
                msg.data = float(self.pm.quary(b"PWR?\n"))
                self.pub_power.publish(msg)
                continue
            continue

    def close(self):
        while not rospy.is_shutdown():
            if self.close_flag == 0:
                continue

            self.pm.close()

            msg = String()
            msg.data = "CLOSED"
            self.pub_close.publish(msg)

            self.close_flag = 0
            continue


    def change_capt(self):
        while not rospy.is_shutdown():
            if self.capt_flag == 0:
                time.sleep(self.rate)
                continue

            self.pm.change_capt(self.capt)

            msg = Float64()
            msg.data = float(self.pm.check_capt())
            self.pub_change_capt.publish(msg)

            self.capt_flag = 0
            continue

    def change_avemode(self):
        while not rospy.is_shutdown():
            if self.avemode_flag == 0:
                time.sleep(self.rate)
                continue

            self.pm.change_avemode(self.avemode)

            msg = Float64()
            msg.data = float(self.pm.check_avemode())
            self.pub_change_avemode.publish(msg)

            self.avemode_flag = 0
            continue

    def start_thread(self):
        th1 = threading.Thread(target=self.power)
        th1.setDaemon(True)
        th1.start()
        th2 = threading.Thread(target=self.change_capt)
        th2.setDaemon(True)
        th2.start()
        th3 = threading.Thread(target=self.change_avemode)
        th3.setDaemon(True)
        th3.start()
        th4 = threading.Thread(target=self.zero_set)
        th4.setDaemon(True)
        th4.start()
        th5 = threading.Thread(target=self.close)
        th5.setDaemon(True)
        th5.start()

if __name__ == "__main__" :
    rospy.init_node("ma24126a")
    ctrl = ma24126a_controller()
    ctrl.start_thread()
    rospy.spin()

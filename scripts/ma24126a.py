#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from std_msgs.msg import Int32
from std_msgs.msg import String
import serial
import time
import sys,os

import threading
import pymeasure


class ma24126a_controller(object):
    def __init__(self):

        self.pm = pymeasure.usbpm.open("ma24126a")
        self.rate = rospy.get_param("~rate")
#flag
        self.zero_set_flag = 0
        self.close_flag = 0
        self.power_flag = 0
        self.avemode_flag = 0
        self.avetyp_flag = 0
        self.capt_flag = 0
#pubsub
        self.pub_zeroset = rospy.Publisher("topic_pub_zeroset", String, queue_size = 1)
        self.sub_zeroset = rospy.Subscriber("topic_sub_zeroset", Float64, self.zero_set_switch)
        self.pub_close = rospy.Publisher("topic_pub_close", String, queue_size = 1)
        self.sub_close = rospy.Subscriber("topic_sub_close", Float64, self.close_switch)
        self.pub_power = rospy.Publisher("topic_pub_power", Float64, queue_size = 1)
        self.sub_power = rospy.Subscriber("topic_sub_power", Float64, self.power_switch)
        self.pub_power_error = rospy.Publisher("topic_pub_power_error", String, queue_size = 1)


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

#main methond

    def zero_set(self):

        print("zero setting now")
        self.pm.zero_set()
        print("zero set has done")

        continue

    def power(self):
        while not rospy.is_shutdown():

            msg = Float64()
            ret = self.pm.quary(b"PWR?\n")
            time.sleep(0.1)
            try:
                msg.data = float(ret)
                self.pub_power.publish(msg)
            except:
                msg = String()
                self.pub_power_error.publish(ret)
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

    def change_avetyp(self):
        while not rospy.is_shutdown():
            if self.avetyp_flag == 0:
                time.sleep(self.rate)
                continue

            self.pm.change_avetyp(self.avetyp)

            msg = Float64()
            msg.data = float(self.pm.check_avetyp())
            self.pub_change_avetyp.publish(msg)

            self.avetyp_flag = 0
            continue

#thread

    def start_thread(self):
        th1 = threading.Thread(target=self.power)
        th1.setDaemon(True)
        th1.start()

        th4 = threading.Thread(target=self.zero_set)
        th4.setDaemon(True)
        th4.start()
        th5 = threading.Thread(target=self.close)
        th5.setDaemon(True)
        th5.start()


if __name__ == "__main__" :
    rospy.init_node("ma24126a")
    ctrl = ma24126a_controller()
    ctrl.zero_set()
    ctrl.start_thread()
    rospy.spin()

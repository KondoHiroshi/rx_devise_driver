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
        capt = rospy.get_param("~capt")

#flag

        self.power_flag = 0


#pubsub

        rospy.Subscriber("ma24126a_power_cmd", Float64, self.power_switch)

        self.pub_power = rospy.Publisher("/dev/ma24126a/power", Float64, queue_size = 1000)

        print("Doing zero setting now")
        self.pm.zero_set()
        print("Finish zero setting !!")
        print("Doing pm setting now")
        self.pm.change_capt(capt)

#flag


    def power_switch(self,q):
        self.power_flag = q.data

#main methond

    def power(self):
        msg = Float64()
        while not rospy.is_shutdown():
            if self.power_flag == 0:
                continue

            while self.power_flag == 1:
                ret = self.pm.power()
                msg.data = float(ret)
                self.pub_power.publish(msg)

            continue


#thread

    def start_thread(self):
        th1 = threading.Thread(target=self.power)
        th1.setDaemon(True)
        th1.start()


if __name__ == "__main__" :
    rospy.init_node("ma24126a")
    ctrl = ma24126a_controller()
    ctrl.start_thread()
    rospy.spin()

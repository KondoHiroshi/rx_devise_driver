#! /usr/bin/env python3

import sys
import time
#sys.path.append("/home/amigos/ros/src/NASCORX_System-master/NASCORX_System")
#from .device import ML2437A

sys.path.append("/home/amigos/ros/src/NASCORX_System-master")
import pymeasure

import rospy
import std_msgs
from std_msgs.msg import Float64
from std_msgs.msg import String
from std_msgs.msg import Int32


class ml2437a_controller(object):
    def __init__(self):
#self.pm = ML2437A.ml2437a()

#self.sub_power = rospy.Subscriber("topic_sub_power", Float64, self.power)
        self.pub_ave_onoff = rospy.Publisher("topic_pub_ave_onoff", Int32, queue_size = 1)
        self.sub_ave_onoff = rospy.Subscriber("topic_sub_ave_onoff", Int32, self.ave_onoff)
        self.pub_ave_count = rospy.Publisher("topic_pub_ave_count", Int32, queue_size = 1)
        self.sub_ave_count = rospy.Subscriber("topic_sub_ave_count", Int32, self.ave_onoff)


    def ave_onoff(self,q):
        self.pm.set_average_onoff(q.data)

        ret = self.quary_average_onoff()
        msg.data = int(ret)
        self.pub_ave_onoff.publish(msg)

    def ave_count(self,q):
        self.pm.set_average_count(q.data)
        ret = self.quary_average_count()
        msg.data = int(ret)
        self.pub_ave_count.publish(msg)

class ml2437a_deriver(object):
    '''
    DESCRIPTION
    ================
    This class cntrols the ML2437A.
    ARGUMENTS
    ================
    1. dev: device number
        Type: int
        Default: 1
    '''

    def __init__(self, IP='192.168.100.44', GPIB=13):
        self.IP = IP
        self.GPIB = GPIB
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)

    def measure(self, ch=1, resolution=3):
        '''
        DESCRIPTION
        ================
        This function queries the input power level.

        ARGUMENTS
        ================
        1. ch: the sensor channel number.
            Number: 1-2
            Type: int
            Default: 1
        2. resolution: the sensor order of the resolution.
            Number: 1-3
            Type: int
            Default: 3

        RETURNS
        ================
        1. power: the power value [dBm]
            Type: float
        '''
        self.com.open()
        self.com.send('CHUNIT %d, DBM' %(ch))
        self.com.send('CHRES %d, %d' %(ch, resolution))
        self.com.send('o %d' %(ch))
        time.sleep(0.1)
        ret = self.com.readline()
        self.com.close()
        power = float(ret)
        return power

    def set_average_onoff(self, onoff, sensor='A'):
        '''
        DESCRIPTION
        ================
        This function switches the averaging mode.

        ARGUMENTS
        ================
        1. onoff: averaging mode
            Number: 0 or 1
            Type: int
            Default: Nothing.

        2. sensor: averaging sensor.
            Number: 'A' or 'B'
            Type: string
            Default: 'A'

        RETURNS
        ================
        Nothing.
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        if onoff == 1:
            self.com.send('AVG %s, RPT, 60' %(sensor))
        else:
            self.com.send('AVG %s, OFF, 60' %(sensor))
        self.com.close()
        return

    def query_average_onoff(self):
        '''
        DESCRIPTION
        ================
        This function queries the averaging mode.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. onoff: averaging mode
            Number: 0 or 1
            Type: int
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('STATUS')
        ret = self.com.readline()
        if ret[17] == '0':
            ret = 0
        else:
            ret = 1
        self.com.close()
        return ret

    def set_average_count(self, count, sensor='A'):
        '''
        DESCRIPTION
        ================
        This function sets the averaging counts.

        ARGUMENTS
        ================
        1. count: averaging counts
            Type: int
            Default: Nothing.

        2. sensor: averaging sensor.
            Number: 'A' or 'B'
            Type: string
            Default: 'A'

        RETURNS
        ================
        Nothing.
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('AVG %s, RPT, %d' %(sensor, count))
        self.com.close()
        return

    def query_average_count(self):
        '''
        DESCRIPTION
        ================
        This function queries the averaging counts.

        ARGUMENTS
        ================
        Nothing.

        RETURNS
        ================
        1. count: averaging counts
            Type: int
        '''
        self.com = pymeasure.gpib_prologix(self.IP, self.GPIB)
        self.com.open()
        self.com.send('STATUS')
        ret = self.com.readline()
        count = int(ret[19:23])
        self.com.close()
        return count


if __name__ == "__main__" :
    rospy.init_node("ml2437a")
    ctrl = ml2437a_controller()
    print("aaa")
    pm = ml2437a_deriver()
    pub_power = rospy.Publisher("topic_pub_power", Float64, queue_size = 1)
    rospy.spin()

while not rospy.is_shutdown():
    ret = pm.measure()
    msg.data = float(ret)
    pub_power.publish(msg)
    continue

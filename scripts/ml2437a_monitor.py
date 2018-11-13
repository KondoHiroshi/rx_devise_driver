#! /usr/bin/env python3


import sys
import time
import numpy
import pyinterface

import rospy
from std_msgs.msg import Float64


if __name__ == '__main__':

    rospy.init_node('cpz3177')
    rate = rospy.get_param('~rate')
    rsw_id = rospy.get_param('~rsw_id')
    node_name = 'cpz3177'
    mode = 'diff'
    ch = 10

    pub = rospy.Publisher('{0}_rsw{1}_{2}{3}'.format(node_name, rsw_id, mode, ch), Float64, queue_size=1)

    try:
        ad = pyinterface.open(3177, rsw_id)
    except OSError as e:
        rospy.logerr("{e.strerror}. node={node_name}, rsw={rsw_id}".format(**locals()))
        sys.exit()

while not rospy.is_shutdown():
    msg = Float64()
    ret = ad.input_voltage(ch, mode)
    msg.data = ret
    pub.publish(msg)
    time.sleep(rate)
    continue

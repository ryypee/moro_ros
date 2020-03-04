#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from marker_msgs.msg import MarkerDetection
from filtering_utils.pf import PF

pf = PF(1000,10,10)

def odom_callback(msg):
    # rospy.loginfo("odometry message")
    pf.predict(msg)
    #pass


def marker_callback(msg):
    info = msg.markers
    if len(info) == 0:# or ekf.initialized == False:
        return
    pf.update(info)


def pf_loc():
    rospy.init_node('pf_localization', anonymous=True)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("base_marker_detection", MarkerDetection, marker_callback)
    rospy.spin()


if __name__ == '__main__':
    pf_loc()

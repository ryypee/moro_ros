#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from nav_msgs.msg import Odometry
from marker_msgs.msg import MarkerDetection
from filtering_utils.ekf import EKF
import pickle
from tf.transformations import euler_from_quaternion
import signal

ekf = EKF(3, 2, 2)

odometry_history = []
state_history = []
gt_history = []

def odom_callback(msg):
    #rospy.loginfo("odometry message")
    if ekf.initialized:
        ekf.predict(msg)
    else:
        return


def marker_callback(msg):
    info = msg.markers
    if len(info) == 0 or ekf.initialized == False:
        print("No beacons seen!!!!")
        return
    for i in range(len(info)):
        #pass
        ekf.update(info[i])

def collect_data(msg):
    ekf.save_data_for_analysis(msg)


def ekf_loc():
    rospy.init_node('ekf_localization', anonymous=True)
    rospy.Subscriber("odom", Odometry, odom_callback)
    rospy.Subscriber("base_marker_detection", MarkerDetection, marker_callback)
    rospy.Subscriber("base_pose_ground_truth", Odometry, collect_data)
    rospy.spin()





if __name__ == '__main__':
    ekf_loc()

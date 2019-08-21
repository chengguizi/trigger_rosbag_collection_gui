#!/usr/bin/env python

import std_srvs.srv
import rospy
import message_filters

import sensor_msgs.msg
import rs2_ros.msg # requires package rs2_ros, clone from GitHub
import std_msgs.msg
import nav_msgs.msg
import geometry_msgs.msg

from threading import Thread, Lock

import data2rosbag
from os.path import expanduser

import time

mutex = Lock()
msg_cache = [] # store the latest synchronised messages
rosbag = data2rosbag.Data2rosbag(expanduser("~") + "/bags")

def take_now_handler(req): # req constain is_dark information
    global msg_cache, mutex

    if (msg_cache == []):
        return std_srvs.srv.SetBoolResponse(False,"Message buffer is empty!")

    if req.data:
        rospy.loginfo("Taking Dark Image")
    else:
        rospy.loginfo("Taking Bright Image")
    # stabilise frame for 0.5s second
    time.sleep(0.5)
    mutex.acquire()
    
    # artificially synchronise all data's timestamp
    timestamp = msg_cache[0].header.stamp
    for msg in msg_cache:
        msg.header.stamp = timestamp

    odom = msg_cache[5]

    pose = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose.header = odom.header
    pose.pose = odom.pose

    rosbag.update(msg_cache[0],msg_cache[1],msg_cache[2],msg_cache[3],msg_cache[4], pose, req)
    mutex.release()
    return std_srvs.srv.SetBoolResponse(True,"")

def filterCallback(sub_left, sub_right, sub_left_info, sub_right_info, sub_camstats, sub_odom):
    global msg_cache, mutex
    mutex.acquire()
    msg_cache = [sub_left, sub_right, sub_left_info, sub_right_info, sub_camstats, sub_odom]
    # print msg_cache[0].header.seq
    mutex.release()

def main():
    # global msg_filter 
    rospy.init_node('trigger_rosbag_collection')
    serv = rospy.Service('take_now',std_srvs.srv.SetBool(), take_now_handler)

    # message synchronisers
    sub_left = message_filters.Subscriber("/rs2_ros/stereo/left/image_rect_raw", sensor_msgs.msg.Image)
    sub_right = message_filters.Subscriber("/rs2_ros/stereo/right/image_rect_raw", sensor_msgs.msg.Image)
    sub_left_info = message_filters.Subscriber("/rs2_ros/stereo/left/camera_info", sensor_msgs.msg.CameraInfo)
    sub_right_info = message_filters.Subscriber("/rs2_ros/stereo/right/camera_info", sensor_msgs.msg.CameraInfo)
    sub_camstats = message_filters.Subscriber("/rs2_ros/stereo/camera_stats", rs2_ros.msg.CameraStats)
    sub_odom = message_filters.Subscriber("/aft_mapped_to_init", nav_msgs.msg.Odometry)

    msg_filter = message_filters.ApproximateTimeSynchronizer([sub_left, sub_right, sub_left_info, sub_right_info, sub_camstats, sub_odom],queue_size=2, slop=0.2)
    msg_filter.registerCallback(filterCallback)

    # msg_cache = message_filters.Cache(msg_filter)

    rospy.loginfo("ready to take snapshots")
    rospy.spin()
    rospy.loginfo("snapshot.py shutting down!")

if __name__ == "__main__":
    main()
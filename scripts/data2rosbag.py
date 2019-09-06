#!/usr/bin/env python

import rosbag
import rospy

import sensor_msgs.msg
import rs2_ros.msg

import geometry_msgs.msg

from datetime import datetime

# for calculating delta pose
import deltapose

class Data2rosbag:
    _path = ""
    _filename = ""
    _bag = ""

    _pub_dark_left = ""
    _pub_dark_right = ""
    _pub_bright_left = ""
    _pub_bright_right = ""
    _pub_pose = ""
    _pub_pose_delta = ""
    _last_pose = ""

    def __init__(self, path):
        self._path = path
        self._pub_dark_left = rospy.Publisher('/snapshot_dark_left', sensor_msgs.msg.Image, queue_size=1)
        self._pub_dark_right = rospy.Publisher('/snapshot_dark_right', sensor_msgs.msg.Image, queue_size=1)
        self._pub_bright_left = rospy.Publisher('/snapshot_bright_left', sensor_msgs.msg.Image, queue_size=1)
        self._pub_bright_right = rospy.Publisher('/snapshot_bright_right', sensor_msgs.msg.Image, queue_size=1)
        self._pub_pose = rospy.Publisher('/snapshot_pose', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)
        self._pub_pose_delta = rospy.Publisher('/snapshot_pose_delta', geometry_msgs.msg.PoseWithCovarianceStamped, queue_size=1)

    def start(self):
        datetimestr = datetime.now().strftime('%Y-%m-%d-%H-%M-%S')
        self._filename = "trigger_stereo_pose_" + datetimestr + ".bag"
        rospy.loginfo("recording to " + self._filename)
        self._bag = rosbag.Bag(self._path + "/" + self._filename,'w')

    def update(self, image_left, image_right, image_left_info, image_right_info, image_stats, pose, lidar_points, is_dark):
        if self._bag == "":
            self.start()
        
        self._bag.write("image_left",image_left)
        self._bag.write("image_right",image_right)
        self._bag.write("image_left_info",image_left_info)
        self._bag.write("image_right_info",image_right_info)
        self._bag.write("image_stats",image_stats)
        self._bag.write("is_dark",is_dark)
        self._bag.write("pose",pose)
        self._bag.write("lidar",lidar_points)

        if (is_dark.data == True):
            self._pub_dark_left.publish(image_left)
            self._pub_dark_left.publish(image_left)
            self._pub_dark_right.publish(image_right)
            self._pub_dark_right.publish(image_right)
        else:
            self._pub_bright_left.publish(image_left)
            self._pub_bright_left.publish(image_left)
            self._pub_bright_right.publish(image_right)
            self._pub_bright_right.publish(image_right)

            # only publish delta when it is bright
            if self._last_pose != "":
                rospy.loginfo(self._last_pose)
                rospy.loginfo(pose)
                delta_pose = deltapose.delta_pose(self._last_pose, pose)
                self._pub_pose_delta.publish(delta_pose)
                self._bag.write("pose_delta", delta_pose)

            self._last_pose = pose

        self._pub_pose.publish(pose)
        # self._pub_pose.publish(pose)
    def close(self):
        if self._bag != "":
            self._bag.close()
        self._bag = ""

    def __del__(self):
        self.close()
    
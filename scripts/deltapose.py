#!/usr/bin/env python

import rospy
import geometry_msgs.msg

import tf

import copy

def pose2matrix(pose):
    trans_vec = [ pose.pose.pose.position.x,  pose.pose.pose.position.y,  pose.pose.pose.position.z]
    trans_matrix = tf.transformations.translation_matrix(trans_vec)
    
    quat_matrix = tf.transformations.quaternion_matrix([pose.pose.pose.orientation.x, pose.pose.pose.orientation.y, pose.pose.pose.orientation.z, pose.pose.pose.orientation.w])
    # quat_matrix = tf.transformations.quaternion_matrix(tf.transformations.random_quaternion())

    return tf.transformations.concatenate_matrices(trans_matrix, quat_matrix) # Rotation first, translation later

def delta_pose(pose_pre, pose_curr):
    
    tf_pre = pose2matrix(pose_pre)
    tf_curr = pose2matrix(pose_curr)

    tf_pre_inverse = tf.transformations.inverse_matrix(tf_pre)

    tf_delta = tf.transformations.concatenate_matrices(tf_curr, tf_pre_inverse)

    ret = copy.deepcopy(pose_curr) #geometry_msgs.msg.PoseWithCovarianceStamped()

    (ret.pose.pose.orientation.x, ret.pose.pose.orientation.y , ret.pose.pose.orientation.z, ret.pose.pose.orientation.w ) = tf.transformations.quaternion_from_matrix(tf_delta)
    (ret.pose.pose.position.x, ret.pose.pose.position.y, ret.pose.pose.position.z) = tf.transformations.translation_from_matrix(tf_delta)
    
    return ret

if __name__ == "__main__":
    pose_pre = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose_pre.pose.pose.position.x = 1.5

    pose_curr = geometry_msgs.msg.PoseWithCovarianceStamped()
    pose_curr.pose.pose.position.x = 2

    delta_pose(pose_pre, pose_curr)

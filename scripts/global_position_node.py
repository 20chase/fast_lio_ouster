#!/usr/bin/python2.7

import time
import rospy
import tf

import numpy as np
import math

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
from std_srvs.srv import Empty


class GlobalPositionNode:
    def __init__(self):
        rospy.init_node("global_position_node")

        self._pose_pub = rospy.Publisher("/fast_lio/global_info", Pose, queue_size=1)
        
        self._tf_broadcaster = tf.TransformBroadcaster()
        self._tf_listener = tf.TransformListener()
        self._tf_ros = tf.TransformerROS()

        self.imu_euler = np.zeros(3)
        self.imu_buffer = list()
        self.imu_timestamp = None
        self.imu_timestamp_buffer = list()
        rospy.Subscriber("/imu/data", Imu, self._imu_cb)

        self.imu_info_received = False

        self._plane_reset_client = rospy.Service('/reset_srv', Empty, self._reinitialize_pose, buff_size=10)
        self._boardcast_timer = rospy.Timer(rospy.Duration(0.02), self._global_info_boardcast_cb)

        # world to odom pose
        self._pose_w2o = Pose()
        self._pose_w2o.position.x = 0.0
        self._pose_w2o.position.y = 0.0
        self._pose_w2o.position.z = 0.0

        self._pose_w2o.orientation.x = 0.0
        self._pose_w2o.orientation.y = 0.0
        self._pose_w2o.orientation.z = 0.0
        self._pose_w2o.orientation.w = 1.0

        time.sleep(0.5)
        rospy.spin()
        
    def _reinitialize_pose(self, srv):
        if not self.imu_euler[0]:
            rospy.logwarn("Miss External IMU Msg!")
            return []

        try:
            t_o2b, quat_o2b = self._tf_listener.lookupTransform(
                "odom", 
                "base_link", 
                rospy.Time(0))
        except tf.LookupException() as e:
            rospy.logwarn("{}".format(e))
            return []

        base_euler = tf.transformations.euler_from_quaternion(
            quat_o2b) # roll pitch yaw angle definition 

        # roll pitch from external imu and yaw from base link
        quat_w2b = tf.transformations.quaternion_from_euler(
            self.imu_euler[0], 
            self.imu_euler[1], 
            base_euler[2])
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        T_o2b = self._tf_ros.fromTranslationRotation(t_o2b, quat_o2b)
        T_b2o = np.linalg.inv(T_o2b)

        # world to odom
        T_w2b = self._tf_ros.fromTranslationRotation(t_o2b, quat_w2b)
        
        T_w2o = np.dot(T_w2b, T_b2o)
        _, _, angles, t_w2o, _ = tf.transformations.decompose_matrix(T_w2o)

        # dist_1 = math.sqrt(t_o2b[0]**2 + t_o2b[1]**2 + t_o2b[2]**2)
        # dist_2 = math.sqrt(t_w2o[0]**2 + t_w2o[1]**2 + t_w2o[2]**2)
        # print (np.arccos((2*dist_1*dist_1 - dist_2*dist_2)/(2*dist_1*dist_1))*180/np.pi)

        quat_w2o = tf.transformations.quaternion_from_euler(
            angles[0], angles[1], angles[2]
        )

        self._pose_w2o.position.x = t_w2o[0]
        self._pose_w2o.position.y = t_w2o[1]
        self._pose_w2o.position.z = t_w2o[2]

        self._pose_w2o.orientation.x = quat_w2o[0]
        self._pose_w2o.orientation.y = quat_w2o[1]
        self._pose_w2o.orientation.z = quat_w2o[2]
        self._pose_w2o.orientation.w = quat_w2o[3]

        print(self._pose_w2o)

        # TODO: send transform here, not other places
        # self._pose_pub.publish(self._pose_w2o)
        rospy.loginfo("reinitialize pose")
        return []
    
    def _global_info_boardcast_cb(self, event):
        if not self.imu_info_received:
            return 

        # world to odom transform
        t_w2o = (self._pose_w2o.position.x,
                self._pose_w2o.position.y,
                self._pose_w2o.position.z)
        quat_w2o = (self._pose_w2o.orientation.x,
                    self._pose_w2o.orientation.y,
                    self._pose_w2o.orientation.z,
                    self._pose_w2o.orientation.w)
        self._tf_broadcaster.sendTransform(t_w2o, quat_w2o, rospy.Time.from_sec(self.imu_timestamp), 'odom', 'world')

        # world to base-link 
        try:
            t_w2b, quat_w2b = self._tf_listener.lookupTransform(
                "world",
                "base_link", 
                rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException) as e:
            rospy.logwarn("{}".format(e))
            return

        euler_w2b = tf.transformations.euler_from_quaternion(quat_w2b)
        quat_w2bp = tf.transformations.quaternion_from_euler(0.0, 0.0, euler_w2b[2])

        T_w2bp = self._tf_ros.fromTranslationRotation(t_w2b, quat_w2bp)
        T_w2o = self._tf_ros.fromTranslationRotation(t_w2o, quat_w2o)
        T_o2w = np.linalg.inv(T_w2o)
        T_o2bp = np.dot(T_o2w, T_w2bp)

        _, _, euler_o2bp, t_o2bp, _ = tf.transformations.decompose_matrix(T_o2bp)
        quat_o2bp = tf.transformations.quaternion_from_euler(euler_o2bp[0], euler_o2bp[1], euler_o2bp[2])

        self._tf_broadcaster.sendTransform(t_o2bp, quat_o2bp, rospy.Time.from_sec(self.imu_timestamp), 'base_link_plane', 'odom')

    def _imu_cb(self, msg):
        imu_euler = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, 
             msg.orientation.y, 
             msg.orientation.z, 
             msg.orientation.w])

        self.imu_buffer.append(np.array(imu_euler))
        self.imu_timestamp_buffer.append(np.array(msg.header.stamp.to_sec()))
        if len(self.imu_buffer) > 20:
            self.imu_buffer.pop(0)
            self.imu_timestamp_buffer.pop(0)

        self.imu_euler = np.mean(self.imu_buffer, axis=0)
        self.imu_timestamp = np.mean(self.imu_timestamp_buffer, axis=0)
        # self.imu_euler = self.imu_buffer[-1]
        # self.imu_timestamp = self.imu_timestamp_buffer[-1]

        self.imu_info_received = True

        # print('Average time: {}'.format(self.imu_timestamp))
        # print('Average euler: {}'.format(self.imu_euler))

    def _gnss_cb(self, msg):
        pass

if __name__ == "__main__":
    node = GlobalPositionNode()
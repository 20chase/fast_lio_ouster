import time
import rospy
import tf

import numpy as np

from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu
# from std_srvs.srv import Empty

class GlobalPositionNode:
    def __init__(self):
        rospy.init_node("global_position_node")

        self._pose_pub = rospy.Publisher("/fast_lio/global_info", Pose, queue_size=1)
        
        self._tf_listener = tf.TransformListener()
        self._tf_ros = tf.TransformerROS()

        self.imu_euler = np.zeros(3)
        self.imu_buffer = []
        rospy.Subscriber("/imu/data", Imu, self._imu_cb)

        time.sleep(1)

    def run(self):
        loop = rospy.Rate(0.1)
        while not rospy.is_shutdown():
            if self._reinitialize_pose():
                rospy.loginfo("reinitialize pose")
            loop.sleep()

    def _reinitialize_pose(self):
        if not self.imu_euler[0]:
            rospy.logwarn("Miss External IMU Msg!")
            return False

        try:
            translation, orientation = self._tf_listener.lookupTransform(
                "odom", 
                "base_link", 
                rospy.Time(0))
        except tf.LookupException() as e:
            rospy.logwarn("{}".format(e))
            return False

        base_euler = tf.transformations.euler_from_quaternion(
            orientation)

        quat_w2b = tf.transformations.quaternion_from_euler(
            self.imu_euler[0], 
            self.imu_euler[1], 
            base_euler[2])

        T_o2b = self._tf_ros.fromTranslationRotation(translation, orientation)
        T_b2o = np.linalg.inv(T_o2b)

        # t_w2b = tf.transformations.translation_matrix(
        #     translation
        # )
        # R_w2b = tf.transformations.quaternion_matrix(
        #     quat_w2b
        # )
        T_w2b = self._tf_ros.fromTranslationRotation(translation, quat_w2b)
        
        T_w2o = np.dot(T_w2b, T_b2o)
        _, _, angles, t_w2o, _ = tf.transformations.decompose_matrix(T_w2o)

        quat_w2o = tf.transformations.quaternion_from_euler(
            angles[0], angles[1], angles[2]
        )

        pose_msg = Pose()
        pose_msg.position.x = t_w2o[0]
        pose_msg.position.y = t_w2o[1]
        pose_msg.position.z = t_w2o[2]

        pose_msg.orientation.x = quat_w2o[0]
        pose_msg.orientation.y = quat_w2o[1]
        pose_msg.orientation.z = quat_w2o[2]
        pose_msg.orientation.w = quat_w2o[3]

        print(pose_msg)

        self._pose_pub.publish(pose_msg)
        return True

    def _imu_cb(self, msg):
        imu_euler = tf.transformations.euler_from_quaternion(
            [msg.orientation.x, 
             msg.orientation.y, 
             msg.orientation.z, 
             msg.orientation.w])

        self.imu_buffer.append(np.array(imu_euler))
        if len(self.imu_buffer) > 20:
            self.imu_buffer.pop(0)
        
        self.imu_euler = np.mean(self.imu_buffer, axis=0)

if __name__ == "__main__":
    node = GlobalPositionNode()
    node.run()
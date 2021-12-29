#!/usr/bin/python2.7

import rospy
import time

from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

class GlobalManagerNode:
    """
    global manager node for the high-level control of ground plane prediction reset.
    """
    def __init__(self):
        # create node
        rospy.init_node("global_manager_node")

        # reset publisher 
        self._plane_reset_pub = rospy.Publisher('/reset_vel', Twist, queue_size=10)
        self._plane_reset_twist = Twist()
        self._plane_reset_twist.angular.x = 0.0
        self._plane_reset_twist.angular.y = 0.0
        self._plane_reset_twist.angular.z = 0.0
        self._plane_reset_twist.linear.x = 0.0
        self._plane_reset_twist.linear.y = 0.0
        self._plane_reset_twist.linear.z = 0.0

        # reset service 
        self._plane_reset_srv = rospy.ServiceProxy('/reset_srv', Empty)

        # get parameters
        reset_period = rospy.get_param('reset_period', default=10.0)
        rospy.loginfo("Received reset period is {} s".format(reset_period))

        # timer 
        self._reset_hz = float(1.0/reset_period)

        time.sleep(1)
    
    def run(self):
        loop = rospy.Rate(self._reset_hz)
        while not rospy.is_shutdown():
            # publish reset message
            self._plane_reset_pub.publish(self._plane_reset_twist)
            # send reset service 
            try:
                self._plane_reset_srv()
            except rospy.ServiceException as exc:
                print("{}".format(exc))
            loop.sleep()


if __name__ == '__main__':
    manager = GlobalManagerNode()
    manager.run()


#!/usr/bin/env python

# DEBUG SCRIPT THAT EMULATES INPUT ON JOINT_STATES AND ACKRM/DESIRED_STEER TOPICS

import random
import rospy
from ackrm_robot.msg import HeaderFloat64
from sensor_msgs.msg import JointState


def talker():
    pub_js = rospy.Publisher('ackrm/joint_states', JointState, queue_size=1)
    pub_th = rospy.Publisher('ackrm/desired_steer', HeaderFloat64, queue_size=1)
    rospy.init_node('dummy', anonymous=True)
    rate = rospy.Rate(1)  # 30hz
    print(rate)
    joint_state = JointState()
    theta = HeaderFloat64()
    while not rospy.is_shutdown():
        theta.header.stamp = rospy.Time.now()
        theta.header.frame_id = 'theta'
        theta.data = random.uniform(-0.5, 0.5)
        joint_state.header.stamp = rospy.Time.now()
        joint_state.header.frame_id = 'joint_state'
        joint_state.name = ['giunto1', 'giunto2', 'giunto3']
        joint_state.position = [random.uniform(
            0, 10), random.randint(0, 10), random.randint(0, 10)]
        rospy.loginfo(theta)
        pub_th.publish(theta)
        rospy.loginfo(joint_state)
        pub_js.publish(joint_state)
        rate.sleep()


if __name__ == '__main__':
    talker()

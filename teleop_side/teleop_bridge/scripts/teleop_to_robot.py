#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import JointState

class TeleopToRobot:
    def __init__(self):
        rospy.init_node('teleop_to_robot_node', anonymous=True)
        
        # 订阅 "joint_states_single" 话题
        rospy.Subscriber("joint_states_single", JointState, self.joint_state_callback, queue_size=1)
        
        # 发布到 "/teleop/joint_states_left" 话题
        self.pub = rospy.Publisher("/teleop/joint_states_left", JointState, queue_size=1)
        
        rospy.loginfo("Teleop to Robot bridge node started")
        rospy.loginfo("Subscribing to: joint_states_single")
        rospy.loginfo("Publishing to: /teleop/joint_states_left")
    
    def joint_state_callback(self, msg):
        """接收 joint_states_single 消息并转发到 /teleop/joint_states_left"""
        # 直接转发消息
        self.pub.publish(msg)

def main():
    try:
        node = TeleopToRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


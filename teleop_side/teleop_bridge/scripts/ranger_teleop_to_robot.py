#!/usr/bin/env python3

import rospy
from ranger_msgs.msg import RsStatus
from geometry_msgs.msg import Twist

class RangerTeleopToRobot:
    def __init__(self):
        rospy.init_node('ranger_teleop_to_robot_node', anonymous=True)
        
        self.max_vel = rospy.get_param('~max_vel', 1.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 4.8)
        
        rospy.Subscriber("/ranger_base_node/rs_state", RsStatus, self.rs_state_callback, queue_size=1)
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.loginfo("Ranger Teleop to Robot node started")
        rospy.loginfo("Subscribing to: /ranger_base_node/rs_state")
        rospy.loginfo("Publishing to: /cmd_vel")
        rospy.loginfo("Parameters: max_vel=%.3f, max_angular_vel=%.3f", self.max_vel, self.max_angular_vel)
    
    def map_stick_to_velocity(self, stick_value, input_min=-100, input_max=99):
        stick_value = max(input_min, min(input_max, stick_value))
        
        if stick_value == 0:
            return 0.0
        
        if stick_value > 0:
            velocity = (stick_value / input_max) * self.max_vel
        else:
            velocity = (stick_value / abs(input_min)) * self.max_vel
        
        return velocity
    
    def map_var_a_to_angular_velocity(self, var_a_value):
        if var_a_value >= 128:
            var_a_int8 = var_a_value - 256
        else:
            var_a_int8 = var_a_value
        
        input_min = -100
        input_max = 99
        
        var_a_int8 = max(input_min, min(input_max, var_a_int8))
        
        if var_a_int8 == 0:
            return 0.0
        
        if var_a_int8 > 0:
            angular_velocity = (var_a_int8 / input_max) * self.max_angular_vel
        else:
            angular_velocity = (var_a_int8 / abs(input_min)) * self.max_angular_vel
        
        return angular_velocity
    
    def rs_state_callback(self, msg):
        cmd_vel = Twist()
        
        cmd_vel.linear.x = self.map_stick_to_velocity(msg.stick_left_v)
        
        cmd_vel.linear.y = self.map_stick_to_velocity(msg.stick_right_h)
        
        cmd_vel.angular.z = self.map_var_a_to_angular_velocity(msg.var_a)
        
        rospy.loginfo("Publishing cmd_vel: linear.x=%.3f, linear.y=%.3f, angular.z=%.3f", 
                     cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z)
        
        self.cmd_vel_pub.publish(cmd_vel)

def main():
    try:
        node = RangerTeleopToRobot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

if __name__ == '__main__':
    main()


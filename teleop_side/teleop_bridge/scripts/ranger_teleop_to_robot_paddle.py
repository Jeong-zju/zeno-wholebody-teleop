#!/usr/bin/env python3

import rospy
from ranger_msgs.msg import RsStatus
from geometry_msgs.msg import Twist

class RangerTeleopToRobot:
    def __init__(self):
        rospy.init_node('ranger_teleop_to_robot_node', anonymous=True)
        
        self.max_vel = rospy.get_param('~max_vel', 1.5)
        self.max_angular_vel = rospy.get_param('~max_angular_vel', 4.8)
        self.input_linear_min = rospy.get_param('~input_linear_min', -1000)
        self.input_linear_max = rospy.get_param('~input_linear_max', 999)
        self.input_angular_min = rospy.get_param('~input_angular_min', -1000)
        self.input_angular_max = rospy.get_param('~input_angular_max', 999)
        self.linear_deadzone = rospy.get_param('~linear_deadzone', 0.2)
        self.angular_deadzone = rospy.get_param('~angular_deadzone', 0.2)
        
        rospy.Subscriber("/teleop/cmd_vel", Twist, self.cmd_vel_callback, queue_size=1)
        
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        rospy.loginfo("Ranger Teleop to Robot node started")
        # rospy.loginfo("Subscribing to: /ranger_base_node/rs_state")
        rospy.loginfo("Subscribing to: /teleop/cmd_vel")
        rospy.loginfo("Publishing to: /cmd_vel")
        rospy.loginfo("Parameters: max_vel=%.3f, max_angular_vel=%.3f, input_linear_min=%.3f, input_linear_max=%.3f, input_angular_min=%.3f, input_angular_max=%.3f, linear_deadzone=%.3f, angular_deadzone=%.3f", 
                     self.max_vel, self.max_angular_vel, self.input_linear_min, self.input_linear_max, self.input_angular_min, self.input_angular_max, self.linear_deadzone, self.angular_deadzone)
    
    def map_linear_velocity(self, stick_value):
        # Apply deadzone: if absolute value is less than deadzone, set to 0
        if abs(stick_value) < self.linear_deadzone:
            stick_value = 0.0
        
        stick_value = max(self.input_linear_min, min(self.input_linear_max, stick_value))
        
        if stick_value == 0:
            return 0.0
        
        if stick_value > 0:
            velocity = (stick_value / self.input_linear_max) * self.max_vel
        else:
            velocity = (stick_value / abs(self.input_linear_min)) * self.max_vel
        
        return velocity
    
    def map_angular_velocity(self, angular_velocity):
        # Apply deadzone: if absolute value is less than deadzone, set to 0
        if abs(angular_velocity) < self.angular_deadzone:
            angular_velocity = 0.0
        
        angular_velocity = max(self.input_angular_min, min(self.input_angular_max, angular_velocity))
        
        if angular_velocity == 0:
            return 0.0
        
        if angular_velocity > 0:
            angular_velocity = (angular_velocity / self.input_angular_max) * self.max_angular_vel
        else:
            angular_velocity = (angular_velocity / abs(self.input_angular_min)) * self.max_angular_vel
        
        return angular_velocity
    
    def cmd_vel_callback(self, msg):
        cmd_vel = Twist()
        
        cmd_vel.linear.x = self.map_linear_velocity(msg.linear.x)
        
        cmd_vel.linear.y = self.map_linear_velocity(msg.linear.y)
        
        cmd_vel.angular.z = self.map_angular_velocity(msg.angular.z)
        
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


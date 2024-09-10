#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist

class BumperHandler:
    def __init__(self):
        self.bumper_triggered = False
        self.bumper_sub = rospy.Subscriber('/bumper_state', Bool, self.bumper_callback)

    def bumper_callback(self, bool_msg):
        if bool_msg.data:
            self.bumper_triggered = True
            rospy.loginfo("[State Machine] Bumper triggered! Transitioning to recovery behavior.")

class RobotStateMachine:
    def __init__(self):
        self.state = 'CONTROLLING'
        self.bumper_handler = BumperHandler()

    def state_machine(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            if self.state == 'CONTROLLING':
                if self.bumper_handler.bumper_triggered:
                    self.transition_to('RECOVERING')
            elif self.state == 'RECOVERING':
                self.execute_recovery()
                self.transition_to('CONTROLLING')
            rate.sleep()

    def transition_to(self, new_state):
        rospy.loginfo(f"[State Machine] Transitioning from {self.state} to {new_state}")
        self.state = new_state

    def execute_recovery(self):
        twist = Twist()
        twist.linear.x = -0.25  # Example: Move backward
        self.publish_command(twist, duration=5.0)
        self.bumper_handler.bumper_triggered = False

    def publish_command(self, twist, duration):
        cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rate = rospy.Rate(10)
        start_time = rospy.Time.now()

        while rospy.Time.now() - start_time < rospy.Duration(duration):
            cmd_pub.publish(twist)
            rate.sleep()

        twist.linear.x = 0.0
        cmd_pub.publish(twist)  # Stop the robot

if __name__ == '__main__':
    rospy.init_node('robot_state_machine')
    state_machine = RobotStateMachine()
    state_machine.state_machine()

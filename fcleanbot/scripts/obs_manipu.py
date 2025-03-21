#!/usr/bin/env python3

import sys
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import moveit_commander
import geometry_msgs.msg

class ManipulationTrigger:
    def __init__(self):
        rospy.init_node('manipulation_trigger', anonymous=True)
        rospy.loginfo("Manipulation Trigger node started.")
        
        # Subscribers
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)
        self.cmd_vel_sub = rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # Timer to periodically check conditions
        self.timer = rospy.Timer(rospy.Duration(0.5), self.timer_callback)
        
        # State variables
        self.obstacle_detected = False
        self.current_cmd_vel = Twist()  # defaults to zero values
        self.manipulation_performed = False

    def laser_callback(self, msg):
        """
        Check if any laser reading within ±15° (≈0.26 rad) of the front
        is less than 0.4 m. If so, set obstacle_detected flag.
        """
        obstacle = False
        for i, r in enumerate(msg.ranges):
            if math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -0.26 <= angle <= 0.26:
                if r < 0.29:
                    obstacle = True
                    break
        self.obstacle_detected = obstacle

    def cmd_vel_callback(self, msg):
        """
        Save the latest velocity command.
        """
        self.current_cmd_vel = msg

    def timer_callback(self, event):
        """
        Periodically check if both conditions are met:
        - An obstacle is detected within 0.4m in front.
        - The robot's commanded velocity is (approximately) zero.
        If yes, trigger the manipulation routine.
        """
        if self.manipulation_performed:
            return

        # Check if the robot is stationary (using a small threshold)
        is_stationary = (abs(self.current_cmd_vel.linear.x) < 0.01 and 
                         abs(self.current_cmd_vel.angular.z) < 0.01)
        
        if self.obstacle_detected and is_stationary:
            rospy.loginfo("Conditions met: obstacle detected and robot is stationary. Triggering manipulation.")
            self.perform_manipulation()
            self.manipulation_performed = True

    def perform_manipulation(self):
        """
        Execute the provided manipulation code using MoveIt.
        """
        # Initialize MoveIt Commander and allow connections
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.loginfo("Initializing MoveIt Commander for manipulation...")
        arm_group = moveit_commander.MoveGroupCommander("arm")
        gripper_group = moveit_commander.MoveGroupCommander("gripper")
        rospy.sleep(3)

        # Move the arm to the first target pose
        target_pose1 = geometry_msgs.msg.Pose()
        target_pose1.position.x = 0.241
        target_pose1.position.y = 0.0
        target_pose1.position.z = 0.116
        target_pose1.orientation.w = 1.0

        rospy.loginfo("Planning motion to first target pose...")
        arm_group.set_pose_target(target_pose1)
        arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        rospy.sleep(10.0)

        # Close the gripper robustly
        rospy.loginfo("Closing gripper...")
        current_gripper_joint_values = gripper_group.get_current_joint_values()
        closed_value = -0.007  # Adjust according to your robot's closed state
        current_gripper_joint_values[0] = closed_value
        gripper_group.set_joint_value_target(current_gripper_joint_values)
        success = gripper_group.go(wait=True)
        gripper_group.stop()
        if not success:
            rospy.logwarn("Gripper failed to close!")
        rospy.sleep(1.0)

        # Move the arm to the second pose
        target_pose2 = geometry_msgs.msg.Pose()
        target_pose2.position.x = 0.046
        target_pose2.position.y = 0.0
        target_pose2.position.z = 0.345
        target_pose2.orientation.w = 1.0

        rospy.loginfo("Moving to second pose...")
        arm_group.set_pose_target(target_pose2)
        arm_group.go(wait=True)
        arm_group.stop()
        arm_group.clear_pose_targets()
        rospy.sleep(1.0)

        rospy.loginfo("Task completed successfully!")

if __name__ == '__main__':
    try:
        ManipulationTrigger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

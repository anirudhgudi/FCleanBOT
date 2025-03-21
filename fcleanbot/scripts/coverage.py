#!/usr/bin/env python3

from math import sqrt, atan2, sin, cos
import math
import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist, Pose2D
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt  # For plotting

class Controller:
    def __init__(self, P=0.0, D=0.0, set_point=0):
        self.Kp = P
        self.Kd = D
        self.set_point = set_point
        self.previous_error = 0

    def update(self, current_value):
        error = self.set_point - current_value
        P_term = self.Kp * error
        D_term = self.Kd * (error - self.previous_error)
        self.previous_error = error
        return P_term + D_term

    def setPoint(self, set_point):
        self.set_point = set_point
        self.previous_error = 0

    def setPD(self, P=0.0, D=0.0):
        self.Kp = P
        self.Kd = D

class TurtlebotCoverage:
    def __init__(self):
        rospy.init_node("turtlebot_coverage")
        rospy.loginfo("Press Ctrl + C to terminate")
        
        self.vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # Reset odometry
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for _ in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()

        # Subscribers for odometry, map, and laser
        self.pose = Pose2D()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)
        self.map_sub = rospy.Subscriber("/map", OccupancyGrid, self.map_callback)
        self.laser_sub = rospy.Subscriber("/scan", LaserScan, self.laser_callback)

        # Parameters
        self.robot_radius = 0.3  # meters
        self.border_margin = self.robot_radius
        
        self.coverage_path = []
        self.map_received = False

        # Flag to indicate if an obstacle is detected within 0.3m in front.
        self.obstacle_detected = False

        # List to store the robot's positions for later plotting.
        self.positions = []

        # Register shutdown hook to plot the trajectory when the node stops.
        rospy.on_shutdown(self.shutdown_hook)

        try:
            rospy.spin()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")

    def map_callback(self, msg):
        rospy.loginfo("Map received. Starting coverage planning in region (0,0)-(4,4).")
        self.map_received = True
        # For this fixed-region planning, we ignore the occupancy grid data.
        self.plan_area_coverage()

    def laser_callback(self, msg):
        # Check laser data in front within ±15° (≈0.26 rad) of the robot's heading.
        obstacle = False
        for i, r in enumerate(msg.ranges):
            if math.isnan(r):
                continue
            angle = msg.angle_min + i * msg.angle_increment
            if -0.26 <= angle <= 0.26:
                if r < 0.3:
                    obstacle = True
                    break
        self.obstacle_detected = obstacle

    def odom_callback(self, msg):
        quaternion = [
            msg.pose.pose.orientation.x,
            msg.pose.pose.orientation.y,
            msg.pose.pose.orientation.z,
            msg.pose.pose.orientation.w
        ]
        _, _, yaw = tf.transformations.euler_from_quaternion(quaternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # Save the current position for plotting.
        self.positions.append((self.pose.x, self.pose.y))

    def ensure_safe_start(self):
        """Move the robot to a safe starting position inside (0,0)-(4,4)."""
        desired_start_x = 0 + self.border_margin
        desired_start_y = 0 + self.border_margin
        rospy.loginfo(f"Moving to safe start position: ({desired_start_x:.2f}, {desired_start_y:.2f})")
        self.move_to_goal(desired_start_x, desired_start_y, Controller(0.2, 0.1))

    def plan_area_coverage(self):
        """Generate a lawnmower (zigzag) pattern of waypoints within the region (0,0)-(4,4)."""
        rospy.loginfo("Planning coverage path within (0,0)-(4,4).")
        region_min_x = 0 + self.border_margin
        region_min_y = 0 + self.border_margin
        region_max_x = 4 - self.border_margin
        region_max_y = 4 - self.border_margin
        
        self.ensure_safe_start()
        
        step_x = 0.5  # horizontal spacing in meters
        step_y = 0.5  # vertical spacing in meters
        
        waypoints = []
        x = region_min_x
        col = 0
        while x <= region_max_x:
            if col % 2 == 0:
                y = region_min_y
                while y <= region_max_y:
                    waypoints.append((x, y))
                    y += step_y
            else:
                y = region_max_y
                while y >= region_min_y:
                    waypoints.append((x, y))
                    y -= step_y
            x += step_x
            col += 1

        self.coverage_path = waypoints
        rospy.loginfo(f"Generated {len(waypoints)} waypoints for coverage.")
        self.follow_path()

    def follow_path(self):
        """Follow each generated waypoint sequentially using PD control."""
        theta_PD = Controller(0.2, 0.1)
        for point in self.coverage_path:
            goal_x, goal_y = point
            rospy.loginfo(f"Navigating to waypoint ({goal_x:.2f}, {goal_y:.2f})")
            self.move_to_goal(goal_x, goal_y, theta_PD)

    def move_to_goal(self, goal_x, goal_y, theta_PD):
        vel = Twist()
        # Clear any previous hold position data when starting a new goal.
        if hasattr(self, 'hold_position'):
            del self.hold_position
        if hasattr(self, 'last_hold_error'):
            del self.last_hold_error

        while not rospy.is_shutdown():
            # If an obstacle is detected within 0.3m, engage position hold using PD control.
            if self.obstacle_detected:
                rospy.loginfo("Obstacle detected. Holding position using PD control.")
                # On first obstacle detection, store the current pose as the hold position.
                if not hasattr(self, 'hold_position'):
                    self.hold_position = (self.pose.x, self.pose.y, self.pose.theta)
                    self.last_hold_error = (0.0, 0.0)  # (last forward error, last lateral error)
                
                # Compute error between hold position and current position.
                dx = self.hold_position[0] - self.pose.x
                dy = self.hold_position[1] - self.pose.y
                # Transform the error to the robot's frame.
                x_error = cos(self.pose.theta) * dx + sin(self.pose.theta) * dy
                y_error = -sin(self.pose.theta) * dx + cos(self.pose.theta) * dy

                # PD parameters for hold control.
                Kp_lin = 0.5
                Kd_lin = 0.1
                Kp_ang = 0.5
                Kd_ang = 0.1

                d_error_lin = x_error - self.last_hold_error[0]
                d_error_ang = y_error - self.last_hold_error[1]
                self.last_hold_error = (x_error, y_error)

                # Compute corrective commands.
                v_cmd = Kp_lin * x_error + Kd_lin * d_error_lin
                w_cmd = Kp_ang * y_error + Kd_ang * d_error_ang

                vel.linear.x = v_cmd
                vel.angular.z = w_cmd
                self.vel_pub.publish(vel)
                self.rate.sleep()
                continue
            else:
                # When no obstacle, clear any hold position state.
                if hasattr(self, 'hold_position'):
                    del self.hold_position
                if hasattr(self, 'last_hold_error'):
                    del self.last_hold_error

            # Compute errors toward the goal.
            error_x = goal_x - self.pose.x
            error_y = goal_y - self.pose.y
            distance = sqrt(error_x**2 + error_y**2)
            if distance < 0.2:
                break

            desired_theta = atan2(error_y, error_x)
            theta_PD.setPoint(desired_theta)
            theta_correction = theta_PD.update(self.pose.theta)
            vel.linear.x = 0.2 * distance
            vel.angular.z = theta_correction

            self.vel_pub.publish(vel)
            self.rate.sleep()

        # Stop the robot after reaching the goal.
        vel.linear.x = 0
        vel.angular.z = 0
        self.vel_pub.publish(vel)

    def shutdown_hook(self):
        """Plot the recorded positions when the node shuts down."""
        rospy.loginfo("Shutting down. Plotting trajectory...")
        if not self.positions:
            rospy.loginfo("No positions recorded.")
            return

        # Unzip the list of positions into x and y components.
        xs, ys = zip(*self.positions)
        plt.figure()
        plt.plot(xs, ys, marker='o', linestyle='-')
        plt.title("Robot Trajectory")
        plt.xlabel("X position (m)")
        plt.ylabel("Y position (m)")
        plt.grid(True)
        plt.axis("equal")
        plt.show()

if __name__ == '__main__':
    TurtlebotCoverage()

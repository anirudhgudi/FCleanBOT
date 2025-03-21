#!/usr/bin/env python
import rospy
import math
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from std_msgs.msg import Float64

# --------------------------------------------------------------------------
# 1) Define approximate link lengths (meters) for OpenManipulator-X.
#    These are based on Robotis documentation and drawings.
# --------------------------------------------------------------------------
d1 = 0.1286   # vertical offset from the base to the shoulder (joint2)
L2 = 0.1466   # length of the upper arm link
L3 = 0.1290   # length of the forearm link
L4 = 0.0240   # small offset from the wrist to the gripper tip (optional)

# --------------------------------------------------------------------------
# 2) IK function: compute [theta1, theta2, theta3, theta4] for a desired (x,y,z).
# --------------------------------------------------------------------------
def compute_open_manipulator_x_ik(x, y, z, elbow='down'):
    """
    Returns joint angles [theta1, theta2, theta3, theta4] in radians
    for a target end-effector position (x, y, z).

    If the position is unreachable, returns None.

    elbow='down' (default) or 'up' controls which elbow solution is chosen.
    """

    # Base rotation around z-axis
    theta1 = math.atan2(y, x)

    # Horizontal distance from the arm's vertical axis
    r = math.sqrt(x**2 + y**2)
    # Vertical distance from shoulder joint
    z_prime = z - d1

    # Distance from shoulder to target
    R = math.sqrt(r**2 + z_prime**2)

    # Check reach
    # The arm can reach between |L2 - L3| and (L2 + L3), ignoring L4 for now.
    if R < abs(L2 - L3) or R > (L2 + L3 + 1e-6):
        rospy.logwarn("Target (%.3f, %.3f, %.3f) is out of reach." % (x, y, z))
        return None

    # Law of cosines for the elbow angle
    # R^2 = L2^2 + L3^2 + 2*L2*L3*cos(delta)
    cos_delta = (R**2 - L2**2 - L3**2) / (2.0 * L2 * L3)
    # Numerical clamp to [-1, 1]
    cos_delta = max(min(cos_delta, 1.0), -1.0)
    delta = math.acos(cos_delta)  # 0 <= delta <= pi

    # "Elbow up" or "elbow down" sets the sign of theta3
    if elbow.lower() == 'down':
        theta3 = -delta
    else:
        theta3 = +delta

    # Shoulder angle
    phi = math.atan2(z_prime, r)
    sin_part = L3 * math.sin(delta)
    cos_part = L2 + L3 * math.cos(delta)
    gamma = math.atan2(sin_part, cos_part)

    if elbow.lower() == 'down':
        theta2 = phi - gamma
    else:
        theta2 = phi + gamma

    # Wrist angle: to keep the gripper "flat", we do:
    theta4 = -(theta2 + theta3)

    return [theta1, theta2, theta3, theta4]

# --------------------------------------------------------------------------
# 3) Helper function to publish joint angles to the arm
# --------------------------------------------------------------------------
def publish_joint_angles(angles, pub, duration=2.0):
    """
    Publish a JointTrajectory message with the given joint angles (in radians)
    to the open_manipulator_x arm.

    :param angles: list [theta1, theta2, theta3, theta4]
    :param pub: the ROS publisher (to /open_manipulator_x/goal_joint_space_path)
    :param duration: how many seconds to complete the motion
    """
    traj = JointTrajectory()
    # The joint names must match those used by the OpenManipulator-X
    traj.joint_names = ["joint1", "joint2", "joint3", "joint4"]

    point = JointTrajectoryPoint()
    point.positions = angles
    point.time_from_start = rospy.Duration(duration)
    traj.points.append(point)

    pub.publish(traj)

# --------------------------------------------------------------------------
# 4) Main: Example usage
# --------------------------------------------------------------------------
def main():
    rospy.init_node("open_manipulator_x_custom_ik_node", anonymous=True)

    # Publisher for joint angles
    joint_pub = rospy.Publisher("/arm_controller/command",
                                JointTrajectory, queue_size=10)
    # Publisher for the gripper
    gripper_pub = rospy.Publisher("/open_manipulator_x/goal_tool_control/command",
                                  Float64, queue_size=10)

    # Give publishers a moment to connect
    rospy.sleep(2.0)

    # ---------------------------
    # First target: pick position
    # ---------------------------
    pick_x, pick_y, pick_z = 0.262, 0.049, 0.1
    pick_angles = compute_open_manipulator_x_ik(pick_x, pick_y, pick_z, elbow='down')
    if pick_angles is None:
        rospy.logerr("Cannot reach pick position.")
        return
    rospy.loginfo("Moving to pick position: (%.3f, %.3f, %.3f)", pick_x, pick_y, pick_z)
    publish_joint_angles(pick_angles, joint_pub, duration=2.5)
    rospy.sleep(3.0)  # wait for motion to complete

    # ---------------------------
    # Close gripper to pick
    # ---------------------------
    gripper_msg = Float64()
    gripper_msg.data = 1.0  # '1.0' might represent closed. Adjust for your setup.
    rospy.loginfo("Closing gripper...")
    gripper_pub.publish(gripper_msg)
    rospy.sleep(2.0)

    # ---------------------------
    # Second target: place position
    # ---------------------------
    place_x, place_y, place_z = 0.04, 0.0, 0.22
    place_angles = compute_open_manipulator_x_ik(place_x, place_y, place_z, elbow='down')
    if place_angles is None:
        rospy.logerr("Cannot reach place position.")
        return
    rospy.loginfo("Moving to place position: (%.3f, %.3f, %.3f)", place_x, place_y, place_z)
    publish_joint_angles(place_angles, joint_pub, duration=3.0)
    rospy.sleep(4.0)

    # (Optionally open gripper here if you want to "release" the object)
    # gripper_msg.data = 0.0  # open
    # gripper_pub.publish(gripper_msg)

    rospy.loginfo("Done.")

if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass

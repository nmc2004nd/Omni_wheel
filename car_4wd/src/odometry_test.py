#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from collections import deque

class OmniOdometry:
    def __init__(self):
        rospy.init_node("omni_odometry_4wheel", anonymous=True)

        # Parameters
        self.r = rospy.get_param("~wheel_radius", 0.0375)  # meters
        self.L = rospy.get_param("~wheel_base", 0.155)     # meters
        self.slip_compensation = rospy.get_param("~slip_compensation", 0.95)  # 0-1
        
        # Odometry matrix initialization
        self._init_odometry_matrix()

        # State variables
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = rospy.Time.now()
        
        # Smoothing filter for velocities
        self.velocity_buffer = deque(maxlen=5)
        self.angular_buffer = deque(maxlen=10)
        
        # ROS interfaces
        self.joint_state_sub = rospy.Subscriber("/omni_car/joint_states", JointState, self.joint_state_callback)
        self.imu_sub = rospy.Subscriber("/omni_car/imu/data", Imu, self.imu_callback)
        self.odom_pub = rospy.Publisher("/odom", Odometry, queue_size=10)
        self.br = tf.TransformBroadcaster()

    def _init_odometry_matrix(self):
        """Initialize the odometry matrix for 4-wheel omnidirectional robot"""
        angles = [math.pi/4, 3*math.pi/4, 5*math.pi/4, 7*math.pi/4]
        
        # Construct the kinematic matrix
        H = np.array([
            [-math.sin(angle), math.cos(angle), self.L] for angle in angles
        ])
        
        # Compute the pseudo-inverse with regularization
        self.mOd_full = np.linalg.pinv(H) * self.r
        self.mOd_full[np.abs(self.mOd_full) < 1e-10] = 0  # Remove near-zero values
        
        # Use only linear velocity components (ignore angular z from wheels)
        self.mOd = self.mOd_full[:2, :]

    def joint_state_callback(self, msg):
        """Process wheel velocities and compute odometry"""
        try:
            # Extract wheel velocities in correct order
            wheel_velocities = [0.0] * 4
            for i, name in enumerate(msg.name):
                if "front_left" in name:
                    wheel_velocities[0] = msg.velocity[i]
                elif "rear_left" in name:
                    wheel_velocities[1] = msg.velocity[i]
                elif "rear_right" in name:
                    wheel_velocities[2] = msg.velocity[i]
                elif "front_right" in name:
                    wheel_velocities[3] = msg.velocity[i]

            # Calculate body-frame velocities
            Vxy = np.dot(self.mOd, wheel_velocities)
            Vx = Vxy[0] * self.slip_compensation
            Vy = Vxy[1] * self.slip_compensation

            # Time delta calculation
            current_time = rospy.Time.now()
            dt = (current_time - self.last_time).to_sec()
            self.last_time = current_time

            # Update position (transform to world frame)
            self.x += (Vx * math.cos(self.yaw) - Vy * math.sin(self.yaw)) * dt
            self.y += (Vx * math.sin(self.yaw) + Vy * math.cos(self.yaw)) * dt

            # Apply smoothing filter
            self.velocity_buffer.append((Vx, Vy))
            avg_Vx = sum(v[0] for v in self.velocity_buffer) / len(self.velocity_buffer)
            avg_Vy = sum(v[1] for v in self.velocity_buffer) / len(self.velocity_buffer)

            self.publish_odometry(avg_Vx, avg_Vy, current_time)

        except Exception as e:
            rospy.logerr(f"Error in joint state callback: {str(e)}")

    def imu_callback(self, msg):
        """Process IMU data for angular velocity and orientation"""
        try:
            # Smooth angular velocity
            self.angular_buffer.append(msg.angular_velocity.z)
            self.w = sum(self.angular_buffer) / len(self.angular_buffer)
            
            # Get orientation from IMU
            orientation = msg.orientation
            q = [orientation.x, orientation.y, orientation.z, orientation.w]
            _, _, yaw = euler_from_quaternion(q)
            
            # Simple low-pass filter for yaw
            self.yaw = 0.9 * self.yaw + 0.1 * yaw

        except Exception as e:
            rospy.logwarn(f"IMU processing error: {str(e)}")

    def publish_odometry(self, Vx, Vy, timestamp):
        """Publish odometry message and TF transform"""
        odom = Odometry()
        odom.header.stamp = timestamp
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # Orientation
        q = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Velocity
        odom.twist.twist.linear.x = Vx
        odom.twist.twist.linear.y = Vy
        odom.twist.twist.angular.z = self.w

        # Publish odometry
        self.odom_pub.publish(odom)

        # Broadcast TF
        self.br.sendTransform(
            (self.x, self.y, 0),
            q,
            timestamp,
            "base_footprint",
            "odom"
        )

    def run(self):
        rospy.loginfo("4-Wheel Omni-directional Odometry Node started")
        rospy.spin()

if __name__ == "__main__":
    try:
        node = OmniOdometry()
        node.run()
    except rospy.ROSInterruptException:
        pass
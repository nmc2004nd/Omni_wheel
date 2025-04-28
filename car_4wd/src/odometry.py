#!/usr/bin/env python3

import rospy
import math
import numpy as np
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
import tf
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class OmniOdometry:
    def __init__(self):
        # Khởi tạo node
        rospy.init_node('omni_odometry', anonymous=True)

        # Robot parameters
        self.wheel_radius =  0.0375 # Bán kính bánh xe (m)  0.0375
        self.robot_radius =  0.1534  # Khoảng cách từ tâm robot đến bánh xe (m)  0.1534

        # Robot pose
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.w = 0.0  # Vận tốc góc từ IMU

        # Ma trận động học nghịch cho 4 bánh
        wheel_angles_deg = [45, 135, 225, 315]  # Góc bánh xe: front_left, front_right, rear_left, rear_right
        drive_matrix = np.empty((0, 3))
        for angle_deg in wheel_angles_deg:
            angle = math.radians(angle_deg)
            row = [-math.sin(angle) / self.wheel_radius,
                   math.cos(angle) / self.wheel_radius,
                   self.robot_radius / self.wheel_radius]
            drive_matrix = np.append(drive_matrix, [row], axis=0)
        self.odometry_matrix = np.linalg.pinv(drive_matrix)

        # Subscribers
        self.joint_state_sub = rospy.Subscriber('/omni_car/joint_states', JointState, self.joint_state_callback)
        self.imu_sub = rospy.Subscriber('/omni_car/imu/data', Imu, self.imu_callback)

        # Publisher
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)

        # TF broadcaster
        self.br = tf.TransformBroadcaster()

        # Time tracking
        self.last_time = rospy.Time.now()

    def joint_state_callback(self, msg):
        # Kiểm tra xem message có đủ dữ liệu không
        if len(msg.velocity) < 4:
            rospy.logwarn("Received joint state message with less than 4 velocities!")
            return

        # Lấy vận tốc góc của từng bánh xe
        wheel_velocities = [0.0, 0.0, 0.0, 0.0]  # [front_left, front_right, rear_left, rear_right]
        for i in range(len(msg.name)):
            if msg.name[i] == "front_left_rim_joint":
                wheel_velocities[0] = msg.velocity[i]
            elif msg.name[i] == "front_right_rim_joint":
                wheel_velocities[1] = msg.velocity[i]
            elif msg.name[i] == "rear_left_rim_joint":
                wheel_velocities[2] = msg.velocity[i]
            elif msg.name[i] == "rear_right_rim_joint":
                wheel_velocities[3] = msg.velocity[i]

        # Chuyển đổi vận tốc góc thành vận tốc tuyến tính (m/s)
        wheel_speeds = [v * self.wheel_radius for v in wheel_velocities]

        # Tính vận tốc robot (Vx, Vy, omega) bằng ma trận động học nghịch
        velocity_vec = np.dot(self.odometry_matrix, np.array(wheel_speeds))
        Vx = velocity_vec[0]
        Vy = velocity_vec[1]
        omega = velocity_vec[2]

        # Tính thời gian
        current_time = rospy.Time.now()
        dt = (current_time - self.last_time).to_sec()  # Sửa từ toSec() thành to_sec()
        self.last_time = current_time

        # Cập nhật vị trí dùng IMU yaw
        # self.x += Vx * math.cos(self.yaw) * dt - Vy * math.sin(self.yaw) * dt
        # self.y += Vx * math.sin(self.yaw) * dt + Vy * math.cos(self.yaw) * dt
        # self.yaw += self.w * dt  # Dùng vận tốc góc từ IMU để cập nhật yaw
        
        # Cập nhật vị trí dùng IMU yaw
        self.x += Vx * math.cos(self.yaw) * dt - Vy * math.sin(self.yaw) * dt
        self.y += Vx * math.sin(self.yaw) * dt + Vy * math.cos(self.yaw) * dt

        # Chuẩn hóa góc yaw trong khoảng [-pi, pi]
        # while self.yaw > math.pi:
        #     self.yaw -= 2 * math.pi
        # while self.yaw < -math.pi:
        #     self.yaw += 2 * math.pi

        # Xuất bản odometry
        self.publish_odometry(Vx, Vy, current_time)

    def imu_callback(self, msg):
        # Lấy vận tốc góc từ IMU
        self.w = msg.angular_velocity.z

        # Lấy yaw từ quaternion
        orientation_q = msg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = euler_from_quaternion(orientation_list)

    def publish_odometry(self, Vx, Vy, current_time):
        # Tạo message Odometry
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_footprint"

        # Vị trí
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.yaw)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        # Vận tốc
        odom.twist.twist.linear.x = Vx
        odom.twist.twist.linear.y = Vy
        odom.twist.twist.angular.z = self.w

        # Xuất bản odometry
        self.odom_pub.publish(odom)

        # Xuất bản transform
        # odom_trans = TransformStamped()
        # odom_trans.header.stamp = current_time
        # odom_trans.header.frame_id = "odom"
        # odom_trans.child_frame_id = "base_footprint"

        # odom_trans.transform.translation.x = self.x
        # odom_trans.transform.translation.y = self.y
        # odom_trans.transform.translation.z = 0.0
        # odom_trans.transform.rotation.x = q[0]
        # odom_trans.transform.rotation.y = q[1]
        # odom_trans.transform.rotation.z = q[2]
        # odom_trans.transform.rotation.w = q[3]

        # self.br.sendTransformMessage(odom_trans)
        
        # Xuất bản transform
        self.br.sendTransform(
            (self.x, self.y, 0.0),
            (q[0], q[1], q[2], q[3]),
            current_time,
            "base_footprint",
            "odom"
        )

if __name__ == '__main__':
    try:
        odometry = OmniOdometry()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
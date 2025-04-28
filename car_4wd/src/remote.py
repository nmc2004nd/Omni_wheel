#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
from kinematics import OmniKinematics4

# Tham số robot
GAIN = 1 # Điều chỉnh tốc độ robot
ROBOT_RADIUS = 0.154  # Khoảng cách từ tâm robot đến bánh xe (m)
WHEEL_RADIUS = 0.0375  # Bán kính bánh xe (m)

# Khởi tạo đối tượng động học
omni = OmniKinematics4(robotRadius=ROBOT_RADIUS, wheelRadius=WHEEL_RADIUS)

# Publishers cho từng bánh xe
pubMotor1 = rospy.Publisher('/omni_car/front_left_joint_velocity_controller/command', 
                            Float64, queue_size=10)
pubMotor2 = rospy.Publisher('/omni_car/rear_left_joint_velocity_controller/command', 
                            Float64, queue_size=10)
pubMotor3 = rospy.Publisher('/omni_car/rear_right_joint_velocity_controller/command', 
                            Float64, queue_size=10)
pubMotor4 = rospy.Publisher('/omni_car/front_right_joint_velocity_controller/command', 
                            Float64, queue_size=10)

def cmdVelCallback(msg):
    # Tính vận tốc bánh xe từ lệnh vận tốc robot
    [m1, m2, m3, m4] = omni.drive(msg.linear.x, msg.linear.y, msg.angular.z)

    # In ra để debug
    rospy.loginfo(f"Motor speeds: m1={m1:.2f}, m2={m2:.2f}, m3={m3:.2f}, m4={m4:.2f}")

    # Gửi vận tốc đến từng bánh xe
    pubMotor1.publish(m1 * GAIN)
    pubMotor2.publish(m2 * GAIN)
    pubMotor3.publish(m3 * GAIN)
    pubMotor4.publish(m4 * GAIN)

if __name__ == '__main__':
    try:
        # Khởi tạo node
        rospy.init_node('remote', anonymous=True)

        # Subscriber cho topic /cmd_vel
        rospy.Subscriber('/cmd_vel', Twist, cmdVelCallback)

        # Đợi kết nối publisher
        rospy.loginfo("Waiting for publisher connections...")
        while (pubMotor1.get_num_connections() == 0 or
               pubMotor2.get_num_connections() == 0 or
               pubMotor3.get_num_connections() == 0 or
               pubMotor4.get_num_connections() == 0) and not rospy.is_shutdown():
            rospy.sleep(0.1)
        rospy.loginfo("Connected! Ready to control robot.")

        # Chạy node
        rospy.spin()

    except rospy.ROSInterruptException:
        pass
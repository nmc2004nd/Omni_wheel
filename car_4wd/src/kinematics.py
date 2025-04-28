import math
import numpy as np
import rospy

NUMBER_OF_WHEELS = 4
l_enc = [0 for _ in range(NUMBER_OF_WHEELS)]
x_pos = 0
y_pos = 0

class OmniKinematics4:
    def __init__(self, robotRadius, wheelRadius):
        self.numOfWheels = 4
        self.robotRadius = robotRadius
        self.wheelRadius = wheelRadius
        self.pos = np.array([0.0, 0.0])
        self.lastOdometryTime = 0

        # Góc đặt bánh, lệch 45 độ
        wheel_angles_deg = [45, 135, 225, 315]  # front_left, rear_left, rear_right, front_right
        self.driveMatrix = np.empty((0, 3))

        for angle_deg in wheel_angles_deg:
            angle = math.radians(angle_deg)
            row = [-math.sin(angle)/wheelRadius,
                   math.cos(angle)/wheelRadius,
                   robotRadius/wheelRadius]
            self.driveMatrix = np.append(self.driveMatrix, [row], axis=0)

        # # Ma trận nghịch đảo để dùng cho odometry
        # self.odometryMatrix = np.linalg.pinv(self.driveMatrix)

    def drive(self, x, y, w):
        velocityVec = np.array([x, y, w])
        motor = np.dot(self.driveMatrix, velocityVec)
        return motor

    # def odometry(self, enc, yaw, dt=1.0):
    #     encVector = np.array(enc)
    #     velocityVec = np.dot(self.odometryMatrix, encVector)
    #     rotation2D = np.array([
    #         [math.cos(yaw), -math.sin(yaw)],
    #         [math.sin(yaw),  math.cos(yaw)]
    #     ])
    #     v_global = np.dot(rotation2D, velocityVec[0:2])
    #     self.pos += v_global * dt
    #     return self.pos, velocityVec[2]  # trả về (x, y), yaw_rate
    
if __name__ == "__main__":
    omni4 = OmniKinematics4(robotRadius=0.2, wheelRadius=0.05)
    [m1, m2, m3, m4] = omni4.drive(1.0, 0.0, 0.5)
    print("Motor speeds:", m1, m2, m3, m4)

    # [xy, w] = omni4.odometry([m1, m2, m3, m4], yaw=0.0, dt=0.1)
    # print("New position:", xy, "Angular rate:", w)
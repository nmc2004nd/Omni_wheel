# Thiết kế robot Omni 4 bánh, triển khai Slam Gmapping và Navigation

## Description:
- Thiết kế và điều khiển xe 4 bánh omni một cách tương đối chính xác thông qua động học, teleop.
- Tính toán odom phục vụ cho gmapping nếu có triển khai thực tế.
- Triển khai cơ bản SLAM Gmapping.
- Triển khai Navigation.

## 1. Setup Environment
- ROS 1 Noetic
- Cài đặt thư viện Python: `pip install -r requirements.txt`
- Cài đặt Plugin cho Gazebo: `sudo apt-get install ros-noetic-gazebo-ros-control ros-noetic-gazebo-ros-imu`
- Cài đặt package SLAM Gmapping: `sudo apt-get install ros-noetic-gmapping`

## Các bước thực hiện:

### 1. Cấp quyền thực thi cho các file Python
Thay `catkin_ws` bằng workspace ROS của bạn (ví dụ: `my_ws`, `dev_ws`, v.v.)
- `cd ~/catkin_ws/src/car_4wd/src`
- `chmod +x *.py`
- `cd ~/catkin_ws/src/car_4wd/scripts`
- `chmod +x *.py`

### 2. Chạy Gazebo và Rviz
- `roslaunch car_4wd display.launch`
- `roslaunch car_4wd gazebo.launch`

### 3. Điều khiển robot Omni
- `roslaunch car_4wd control.launch`
- `rosrun car_4wd wheel_control.py`
Điều khiển thông qua việc publish trực tiếp vào các joint

### 4. SLAM Gmapping
- Có 3 kiểu odom bạn có thể sử dụng:
  - Odom do plugin planar cung cấp.
  - Odom do model Gazebo cung cấp: Trong file cấu hình Gazebo của robot (`.gazebo` hoặc `.world`), thay thế plugin odometry hiện tại bằng plugin được cấu hình để sử dụng file `odometry_automatic.py`.
  - Odom do tính toán thông qua động học: Trong file cấu hình Gazebo của robot, thay thế plugin odometry hiện tại bằng plugin được cấu hình để sử dụng file `odometry_test.py`.
- `roslaunch car_4wd omni_robot_slam_gmapping.launch`
- `rosrun teleop_twist_keyboard teleop_twist_keyboard.py`

### 5. Navigation
- Ở đây nên dùng odom do gazebo cung cấp qua model ( odometry_automatic.py) để Navigation hoạt động đúng.
- 'roslaunch omni_robot_navigation_rviz.launch'

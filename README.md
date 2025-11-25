# ZYF176 / ZYF143 IMU ROS 2 Driver

这是一个适用于 **ZYF176** 和 **ZYF143** 系列惯性测量单元 (IMU) 的 ROS 2 驱动程序。
该驱动程序通过串口读取 IMU 数据，并发布标准的 `sensor_msgs/msg/Imu` 消息。

## 功能特点
- 支持 ROS 2 (Humble/Foxy 等)
- 读取加速度、角速度、欧拉角
- 自动发布 `/imu_data` 话题 (100Hz/200Hz)
- 解决了数据断包和 CRC 校验问题
- 使用标准 TF2 坐标转换

## 依赖 (Dependencies)
本项目依赖 `serial` 库。

## 安装步骤

1. **创建工作空间** (如果你还没有):
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
克隆 serial 库 (ROS 2 版本):
code
Bash
git clone -b ros2 https://github.com/tylerjw/serial.git
克隆本驱动:
code
Bash
git clone https://github.com/litiansuo1145/imu_yzf143_ros2.git
编译:
code
Bash
cd ~/ros2_ws
colcon build
source install/setup.bash
串口权限设置
在使用前，请确保有串口权限：
code
Bash
sudo chmod 777 /dev/ttyUSB0
或者将用户加入 dialout 组 (永久生效):
code
Bash
sudo usermod -a -G dialout $USER
运行
基本运行:
code
Bash
ros2 run imu_yzf143_ros2 imu_node
带参数运行 (修改端口):
code
Bash
ros2 run imu_yzf143_ros2 imu_node --ros-args -p port:=/dev/ttyUSB1 -p baud:=115200
话题 (Topics)
/imu_data (sensor_msgs/msg/Imu): 包含姿态(四元数)、角速度和线加速度。
坐标系
frame_id: 默认为 imu_link (可通过参数修改)。

遵循右手定则，ENU 坐标系。给我的readme改为标准markdown格式

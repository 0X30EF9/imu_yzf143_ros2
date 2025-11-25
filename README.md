# ZYF176 / ZYF143 IMU ROS 2 Driver

这是一个适用于 **ZYF176** 和 **ZYF143** 系列惯性测量单元 (IMU) 的 ROS 2 驱动程序。该驱动程序通过串口读取 IMU 数据，并发布标准的 `sensor_msgs/msg/Imu` 消息。

## 功能特点

- 支持 ROS 2 (Humble/Foxy 等)
- 读取加速度、角速度、欧拉角
- 自动发布 `/imu_data` 话题 (100Hz/200Hz)
- 解决了数据断包和 CRC 校验问题
- 使用标准 TF2 坐标转换

## 依赖

本项目依赖 `serial` 库。

## 安装步骤

### 1. 创建工作空间 (如果还没有)

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 克隆 serial 库 (ROS 2 版本)

```bash
git clone -b ros2 https://github.com/tylerjw/serial.git
```

### 3. 克隆本驱动

```bash
git clone https://github.com/litiansuo1145/imu_yzf143_ros2.git
```

### 4. 编译

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

## 串口权限设置

在使用前，请确保有串口权限：

### 临时权限设置

```bash
sudo chmod 777 /dev/ttyUSB0
```

### 永久权限设置 (将用户加入 dialout 组)

```bash
sudo usermod -a -G dialout $USER
```

**注意：** 执行永久权限设置后需要重新登录生效。

## 运行

### 基本运行

```bash
ros2 run imu_yzf143_ros2 imu_node
```

### 带参数运行 (修改端口)

```bash
ros2 run imu_yzf143_ros2 imu_node --ros-args -p port:=/dev/ttyUSB1 -p baud:=115200
```

## 话题

- `/imu_data` (`sensor_msgs/msg/Imu`): 包含姿态(四元数)、角速度和线加速度。

## 坐标系

- `frame_id`: 默认为 `imu_link` (可通过参数修改)
- 遵循右手定则，ENU 坐标系

## 参数配置

可通过以下参数自定义节点行为：

- `port`: 串口设备路径 (默认: `/dev/ttyUSB0`)
- `baud`: 波特率 (默认: `115200`)
- `frame_id`: 坐标系ID (默认: `imu_link`)

## 故障排除

如果遇到连接问题：

1. 检查串口设备是否正确连接
2. 确认串口权限设置
3. 验证波特率设置与硬件匹配
4. 检查设备是否被其他进程占用

```bash
# 检查串口设备
ls -l /dev/ttyUSB*

# 检查进程占用
lsof /dev/ttyUSB0
```

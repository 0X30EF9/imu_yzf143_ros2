# ZYF176 / ZYF143 IMU ROS 2 Driver

通过串口读取 ZYF176 / ZYF143 IMU 的加速度、角速度和欧拉角，发布 `sensor_msgs/msg/Imu`。节点名为 `imu_yzf143_node`，可执行文件为 `imu_node`，默认话题为 `/imu_data`，保持可靠 QoS 发布。

驱动支持分段接收、连续帧解析和校验失败后的重新同步。设备暂缺、权限未生效或读操作异常后会定时重连，持续收不到有效帧时也会重新打开串口。重连会清除上一次连接的残帧。端口类型或串口配置错误会让节点报错退出，修正配置后重新启动。

## 安装与编译

需要已配置的 ROS 2、`colcon` 和 ROS 2 版本的 `serial` 库。在已加载 ROS 2 环境的终端执行以下命令。

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone -b ros2 https://github.com/tylerjw/serial.git
git clone https://github.com/0X30EF9/imu_yzf143_ros2.git
cd ~/ros2_ws
colcon build --packages-up-to imu_yzf143_ros2
source install/setup.bash
```

在 Linux 上，将运行节点的用户加入串口所属的 `dialout` 组，然后重新登录，使组权限生效。

```bash
sudo usermod -a -G dialout "$USER"
```

## 运行

使用默认参数运行。

```bash
ros2 run imu_yzf143_ros2 imu_node
```

在启动时指定设备路径和波特率。

```bash
ros2 run imu_yzf143_ros2 imu_node --ros-args -p port:=/dev/ttyUSB1 -p baud:=115200
```

通过 launch 加载包内的 `config/imu.yaml`。

```bash
ros2 launch imu_yzf143_ros2 imu.launch.py
```

复制并编辑 YAML 文件后，可以指定自己的参数文件。launch 按文件内容配置端口及其他参数。

```bash
ros2 launch imu_yzf143_ros2 imu.launch.py params_file:=/absolute/path/to/imu.yaml
```

这些参数在启动时读取，运行中为只读。修改参数后需要重新启动节点。

| 参数 | 默认值 | 含义 |
| --- | --- | --- |
| `port` | `/dev/ttyUSB0` | 串口设备路径 |
| `baud` | `115200` | 正整数波特率，需与设备配置一致 |
| `frame_id` | `imu_link` | IMU 消息的坐标系标识 |
| `poll_interval_ms` | `5` | 正整数，串口轮询周期，单位 ms |
| `serial_timeout_ms` | `5` | 非负整数，串口读取超时，单位 ms，`0` 表示不等待额外数据 |
| `reconnect_interval_ms` | `1000` | 正整数，串口重连间隔，单位 ms |
| `data_timeout_ms` | `1000` | 无有效帧的超时，单位 ms，设为 `0` 可禁用此项重连触发条件 |
| `orientation_variance` | `[0.0, 0.0, 0.0]` | 姿态三个轴的方差，单位 rad² |
| `angular_velocity_variance` | `[0.0, 0.0, 0.0]` | 角速度三个轴的方差，单位 (rad/s)² |
| `linear_acceleration_variance` | `[0.0, 0.0, 0.0]` | 加速度三个轴的方差，单位 (m/s²)² |

每组方差参数必须包含三个非负、有限的浮点数，分别填入对应协方差矩阵的三个对角元素。默认全零按 ROS IMU 消息约定表示协方差未知，使用标定结果时可在 YAML 中覆盖。

## 数据与时间

驱动保留已有设备帧的格式和比例转换。

| 字节偏移 | 内容 |
| --- | --- |
| `0–1` | 帧头 `A5 A5` |
| `2–7` | 加速度 `x`、`y`、`z` |
| `8–13` | 角速度 `x`、`y`、`z` |
| `14–19` | 欧拉角 `yaw`、`pitch`、`roll` |
| `20–21` | 设备 `timeStamp` 字段 |
| `22–23` | 原有 16 位累加校验字段 |

每帧共 24 字节，各数据字段按小端 16 位整数解码。校验值采用原驱动的 `gyro.z + yaw + pitch + roll`，按 16 位取模比较。

- 加速度按 `9.80665 / 10920.0` 转换为 m/s²。
- 角速度和欧拉角按 `0.01°` 转换为 rad/s 和 rad，再由 roll、pitch、yaw 构造姿态四元数。
- 消息时间戳采用主机收到并解析有效帧时的 ROS 时间。设备 `timeStamp` 的单位和起点尚未明确，暂不用于时间同步。
- 串口重连和有效帧超时使用单调时间，ROS 仿真时间暂停时仍可恢复串口连接。

发布频率由设备输出频率决定。默认 5 ms 轮询可能在一次回调中发布多个已接收的数据帧，这些帧的主机时间戳可能接近。轮询周期和定时器调度也会影响接收延迟。

消息按设备原始轴向和现有比例发布。`frame_id` 应与设备实际安装匹配，设备到机器人坐标系的变换需由使用方配置。ENU 方向需要结合设备协议和安装姿态确认。

## 验证

在工作空间编译后执行测试。

```bash
colcon test --packages-select imu_yzf143_ros2
colcon test-result --verbose
```

解析回归测试验证分帧、校验和数值解码。Linux PTY 模拟串口测试用于验证节点收发、异常恢复及重连路径，测试使用合成数据帧。真实设备的输出频率、轴向、精度和拔插表现仍需在硬件上验证。

## 故障排除

检查串口是否存在、设备权限是否生效，以及端口是否已被其他进程占用。

```bash
ls -l /dev/ttyUSB*
id -nG
lsof /dev/ttyUSB0
```

端口存在但持续收不到有效帧时，检查波特率和设备协议。若设备正常出帧间隔超过 `data_timeout_ms`，应调大该参数或设为 `0`，以匹配设备行为。

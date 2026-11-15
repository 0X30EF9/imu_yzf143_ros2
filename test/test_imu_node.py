"""Exercise the installed ROS node with Linux pseudoterminals and synthetic frames."""

import math
import os
from pathlib import Path
import random
import signal
import struct
import subprocess
import sys
import time
import uuid

import pytest

if sys.platform != "linux":
    pytest.skip("Serial integration tests require Linux PTYs", allow_module_level=True)

import rclpy
from ament_index_python.packages import get_package_prefix
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu


TO_RAD = 0.01 * math.pi / 180.0
ACCELERATION_SCALE = 9.80665 / 10920.0


def make_frame(tag, *, yaw=9000, pitch=0, roll=0):
    acceleration = (10920, -5460, 0)
    angular_velocity = (tag, -200, 300)
    checksum = (angular_velocity[2] + yaw + pitch + roll) & 0xFFFF
    return b"\xa5\xa5" + struct.pack(
        "<10hH", *acceleration, *angular_velocity, yaw, pitch, roll, 42, checksum
    )


def sample_tag(message):
    return round(message.angular_velocity.x / TO_RAD)


def serial_environment(domain_id):
    environment = os.environ.copy()
    environment.update({
        "ROS_DOMAIN_ID": str(domain_id),
        "ROS_LOCALHOST_ONLY": "1",
        "RCUTILS_COLORIZED_OUTPUT": "0",
        "RCUTILS_LOGGING_BUFFERED_STREAM": "0",
    })
    return environment


@pytest.fixture
def executable():
    binary = Path(get_package_prefix("imu_yzf143_ros2")) / "lib" / "imu_yzf143_ros2" / "imu_node"
    assert Path(binary).is_file(), f"IMU node executable does not exist: {binary}"
    return binary


class SerialDriver:
    def __init__(self, directory, executable, parameters):
        self.port = directory / "imu-port"
        self.log_path = directory / "imu-node.log"
        self.log_stream = self.log_path.open("wb")
        self.executable = executable
        self.parameters = {"reconnect_interval_ms": 100, **parameters}
        self.domain_id = random.SystemRandom().randint(130, 200)
        self.namespace = "imu_test_" + uuid.uuid4().hex[:12]
        self.environment = serial_environment(self.domain_id)
        self.process = None
        self.master = None
        self.slave = None
        self.messages = []

        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=self.domain_id)
        self.probe = rclpy.create_node(
            "imu_probe", namespace=self.namespace, context=self.context
        )
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.probe)
        self.topic = f"/{self.namespace}/imu_data"
        reliable_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.probe.create_subscription(
            Imu, self.topic, self.messages.append, reliable_qos
        )

    def connect(self):
        assert self.master is None and self.slave is None
        self.master, self.slave = os.openpty()
        self.port.unlink(missing_ok=True)
        self.port.symlink_to(os.ttyname(self.slave))

    def disconnect(self):
        for descriptor in (self.master, self.slave):
            if descriptor is not None:
                os.close(descriptor)
        self.master = None
        self.slave = None

    def start(self):
        arguments = [
            self.executable, "--ros-args", "-r", f"__ns:=/{self.namespace}",
            "-p", f"port:={self.port}",
        ]
        for name, value in self.parameters.items():
            arguments.extend(["-p", f"{name}:={value}"])
        self.process = subprocess.Popen(
            arguments,
            stdout=self.log_stream,
            stderr=subprocess.STDOUT,
            env=self.environment,
        )

    def log(self):
        return self.log_path.read_text(errors="replace")

    def write(self, payload):
        assert self.master is not None
        assert os.write(self.master, payload) == len(payload)

    def wait(self, condition, *, timeout=5.0, on_tick=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if condition():
                return
            assert self.process.poll() is None, self.log()
            if on_tick is not None:
                on_tick()
            self.executor.spin_once(timeout_sec=0.02)
        raise AssertionError(f"IMU condition timed out after {timeout}s\n{self.log()}")

    def spin_for(self, duration):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            assert self.process.poll() is None, self.log()
            self.executor.spin_once(timeout_sec=0.02)

    def wait_for_open(self):
        self.wait(lambda: "Opened IMU serial port" in self.log())

    def receive(self, tag):
        next_write = 0.0

        def feed():
            nonlocal next_write
            if time.monotonic() >= next_write:
                self.write(make_frame(tag))
                next_write = time.monotonic() + 0.1

        self.wait(
            lambda: any(sample_tag(message) == tag for message in self.messages),
            on_tick=feed,
        )
        return next(message for message in self.messages if sample_tag(message) == tag)

    def close(self):
        if self.process is not None and self.process.poll() is None:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=2.0)
        self.disconnect()
        self.executor.remove_node(self.probe)
        self.probe.destroy_node()
        self.executor.shutdown(timeout_sec=1.0)
        if self.context.ok():
            self.context.shutdown()
        self.log_stream.close()


@pytest.fixture
def driver(tmp_path, executable):
    drivers = []

    def create(**parameters):
        running = SerialDriver(tmp_path, executable, parameters)
        drivers.append(running)
        return running

    yield create
    for running in reversed(drivers):
        running.close()


def test_missing_port_recovers_and_decodes_stream(driver):
    running = driver(data_timeout_ms=0)
    running.start()
    running.wait(lambda: "Cannot open" in running.log())
    running.connect()
    running.wait_for_open()
    running.receive(1000)  # Wait for DDS discovery before the one-shot stream below.

    bad_frame = bytearray(make_frame(999))
    bad_frame[-1] ^= 1
    first = make_frame(100)
    running.write(b"\x00\xff\xa5\x00" + bad_frame + first[:7])
    running.spin_for(0.12)
    assert not any(sample_tag(message) in (100, 999) for message in running.messages)
    running.write(first[7:] + make_frame(200) + make_frame(300))
    wanted = (100, 200, 300)
    running.wait(lambda: all(
        any(sample_tag(message) == tag for message in running.messages) for tag in wanted
    ))
    decoded = [message for message in running.messages if sample_tag(message) in wanted]
    assert [sample_tag(message) for message in decoded] == list(wanted)
    assert not any(sample_tag(message) == 999 for message in running.messages)

    message = decoded[0]
    assert message.header.frame_id == "imu_link"
    assert message.header.stamp.sec > 0
    assert [message.linear_acceleration.x, message.linear_acceleration.y,
            message.linear_acceleration.z] == pytest.approx(
                [10920 * ACCELERATION_SCALE, -5460 * ACCELERATION_SCALE, 0.0]
            )
    assert [message.angular_velocity.x, message.angular_velocity.y,
            message.angular_velocity.z] == pytest.approx([100 * TO_RAD, -200 * TO_RAD, 300 * TO_RAD])
    assert [message.orientation.x, message.orientation.y, message.orientation.z,
            message.orientation.w] == pytest.approx([0.0, 0.0, math.sqrt(0.5), math.sqrt(0.5)])
    for covariance in (message.orientation_covariance, message.angular_velocity_covariance,
                       message.linear_acceleration_covariance):
        assert list(covariance) == [0.0] * 9
    publishers = running.probe.get_publishers_info_by_topic(running.topic)
    assert publishers and all(
        publisher.qos_profile.reliability == ReliabilityPolicy.RELIABLE
        for publisher in publishers
    )


def test_covariance_parameters_reach_ros_messages(driver):
    variances = {
        "orientation_variance": [0.1, 0.2, 0.3],
        "angular_velocity_variance": [0.4, 0.5, 0.6],
        "linear_acceleration_variance": [0.7, 0.8, 0.9],
    }
    running = driver(data_timeout_ms=0, frame_id="calibrated_imu", **variances)
    running.connect()
    running.start()
    running.wait_for_open()
    message = running.receive(500)
    assert message.header.frame_id == "calibrated_imu"
    for field, values in variances.items():
        expected = [0.0] * 9
        expected[0], expected[4], expected[8] = values
        covariance = getattr(message, field.replace("_variance", "_covariance"))
        assert list(covariance) == pytest.approx(expected)


def test_reconnect_clears_the_previous_ports_partial_frame(driver):
    running = driver(data_timeout_ms=300)
    running.connect()
    running.start()
    running.wait_for_open()
    running.receive(600)
    previous_opens = running.log().count("Opened IMU serial port")

    partial = make_frame(610)
    running.write(partial[:10])
    running.spin_for(0.12)
    running.disconnect()
    running.connect()
    running.wait(lambda: running.log().count("Opened IMU serial port") > previous_opens)
    running.write(partial[10:] + make_frame(620))
    running.wait(lambda: any(sample_tag(message) == 620 for message in running.messages))
    assert not any(sample_tag(message) == 610 for message in running.messages)


def test_valid_frame_timeout_reconnects_with_paused_ros_time(driver):
    running = driver(data_timeout_ms=200, use_sim_time="true")
    running.connect()
    running.start()
    running.wait_for_open()
    message = running.receive(700)
    assert (message.header.stamp.sec, message.header.stamp.nanosec) == (0, 0)
    previous_opens = running.log().count("Opened IMU serial port")
    previous_timeouts = running.log().count("No valid IMU frames before data timeout")

    bad_frame = bytearray(make_frame(799))
    bad_frame[-1] ^= 1
    running.wait(
        lambda: running.log().count("No valid IMU frames before data timeout") > previous_timeouts,
        on_tick=lambda: running.write(bad_frame),
    )
    running.wait(lambda: running.log().count("Opened IMU serial port") > previous_opens)
    assert sample_tag(running.receive(710)) == 710
    assert not any(sample_tag(message) == 799 for message in running.messages)


def test_zero_data_timeout_keeps_an_idle_port_open(driver):
    running = driver(data_timeout_ms=0)
    running.connect()
    running.start()
    running.wait_for_open()
    running.receive(800)
    previous_opens = running.log().count("Opened IMU serial port")
    running.spin_for(1.3)
    assert running.log().count("Opened IMU serial port") == previous_opens
    assert "No valid IMU frames before data timeout" not in running.log()
    assert sample_tag(running.receive(810)) == 810


def test_non_serial_device_fails_without_infinite_retries(executable):
    result = subprocess.run(
        [executable, "--ros-args", "-p", "port:=/dev/null"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=serial_environment(random.SystemRandom().randint(130, 200)),
        timeout=3.0,
        check=False,
    )
    assert result.returncode != 0, result.stdout
    assert "IMU node failed" in result.stdout, result.stdout


@pytest.mark.parametrize("name,value", [
    ("baud", "-1"),
    ("poll_interval_ms", "0"),
    ("data_timeout_ms", "-1"),
    ("orientation_variance", "[0.0, 0.0]"),
    ("linear_acceleration_variance", "[0.0, -1.0, 0.0]"),
    ("angular_velocity_variance", "[.nan, 0.0, 0.0]"),
])
def test_invalid_startup_parameters_exit_with_an_error(executable, tmp_path, name, value):
    result = subprocess.run(
        [executable, "--ros-args", "-p", f"port:={tmp_path / 'unused-port'}",
         "-p", f"{name}:={value}"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=serial_environment(random.SystemRandom().randint(130, 200)),
        timeout=4.0,
        check=False,
    )
    assert result.returncode != 0, result.stdout
    assert name in result.stdout, result.stdout

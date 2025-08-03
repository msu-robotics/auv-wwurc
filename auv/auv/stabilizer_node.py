#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, WrenchStamped
from sensor_msgs.msg import Imu, FluidPressure
import math
from rclpy.qos import qos_profile_sensor_data

ATM_PRESSURE = 101325.0  # Па
WATER_DENSITY = 997.0    # кг/м³
G = 9.80665               # м/с²


class PID:
    def __init__(self, kp, ki, kd, name=''):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_error = 0.0
        self.name = name

    def compute(self, target, current, dt):
        error = target - current
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt if dt > 0.0 else 0.0
        self.prev_error = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative


class StabilizerNode(Node):
    def __init__(self):
        super().__init__('stabilizer_node')

        self.depth_pid = PID(2.0, 0.0, 0.5, "depth")
        self.yaw_pid = PID(1.5, 0.0, 0.2, "yaw")

        self.depth = 10.0
        self.yaw = 0.0
        self.cmd_vel = Twist()
        self.last_time = self.get_clock().now()

        self.create_subscription(Imu, '/sensors/imu', self.imu_callback, qos_profile_sensor_data)
        self.create_subscription(FluidPressure, '/sensors/fluid_pressure', self.pressure_callback, qos_profile_sensor_data)
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)

        self.wrench_pub = self.create_publisher(WrenchStamped, '/wrench_cmd', 10)
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info("Stabilization node was running")

    def imu_callback(self, msg: Imu):
        q = msg.orientation
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny_cosp, cosy_cosp)

    def pressure_callback(self, msg: FluidPressure):
        pressure = msg.fluid_pressure
        self.depth = (pressure - ATM_PRESSURE) / (WATER_DENSITY * G)

    def cmd_callback(self, msg: Twist):
        self.cmd_vel = msg

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        # Целевые значения
        target_depth = -self.cmd_vel.linear.z
        target_yaw_rate = self.cmd_vel.angular.z
        forward_thrust = self.cmd_vel.linear.x

        # PID управление
        fz = self.depth_pid.compute(target_depth, self.depth, dt)
        mz = self.yaw_pid.compute(target_yaw_rate, 0.0, dt)

        # Формирование WrenchStamped
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = now.to_msg()
        wrench_msg.header.frame_id = 'base_link'
        wrench_msg.wrench.force.x = forward_thrust
        wrench_msg.wrench.force.z = fz
        wrench_msg.wrench.torque.z = mz

        self.wrench_pub.publish(wrench_msg)

        # LOG: вывод в stdout
        self.get_logger().info(
            f"[STAB] depth: {self.depth:.2f} m → {target_depth:.2f} m | Fz: {fz:.2f} N\n"
            f"[STAB] yaw: {math.degrees(self.yaw):.1f}° | target yaw rate: {target_yaw_rate:.2f} rad/s | Mz: {mz:.2f} Nm\n"
            f"[STAB] Fx: {forward_thrust:.2f} N"
        )


def main(args=None):
    rclpy.init(args=args)
    node = StabilizerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()

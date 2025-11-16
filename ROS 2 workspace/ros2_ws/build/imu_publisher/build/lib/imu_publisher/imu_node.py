#!/usr/bin/env python3
import time
import board
import busio
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import adafruit_lsm303_accel
import adafruit_lsm303dlh_mag
import adafruit_l3gd20


class IMUPublisher(Node):
    def __init__(self):
        super().__init__('imu_publisher')

        # I2C setup
        i2c = busio.I2C(board.SCL, board.SDA)

        # Sensors
        self.accel = adafruit_lsm303_accel.LSM303_Accel(i2c)
        self.mag = adafruit_lsm303dlh_mag.LSM303DLH_Mag(i2c)
        self.gyro = adafruit_l3gd20.L3GD20_I2C(i2c, address=0x69)

        # Publisher
        self.publisher_ = self.create_publisher(Imu, 'imu/data_raw', 10)
        timer_period = 0.05  # 20 Hz
        self.timer = self.create_timer(timer_period, self.publish_imu)

    def publish_imu(self):
        msg = Imu()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'base_link'

        # Accel in m/s^2
        ax, ay, az = self.accel.acceleration
        msg.linear_acceleration.x = ax
        msg.linear_acceleration.y = ay
        msg.linear_acceleration.z = az

        # Gyro in rad/s
        gx, gy, gz = self.gyro.gyro
        msg.angular_velocity.x = gx
        msg.angular_velocity.y = gy
        msg.angular_velocity.z = gz

        # Orientation left empty (will be filled by Madgwick filter later)
        msg.orientation_covariance[0] = -1.0

        self.publisher_.publish(msg)
        self.get_logger().info(f'IMU Published: Accel=({ax:.2f},{ay:.2f},{az:.2f}) Gyro=({gx:.2f},{gy:.2f},{gz:.2f})')


def main(args=None):
    rclpy.init(args=args)
    node = IMUPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


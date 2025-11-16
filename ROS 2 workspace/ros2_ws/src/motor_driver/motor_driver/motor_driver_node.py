#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import RPi.GPIO as GPIO

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')

        # Updated GPIO pin mapping
        self.IN1, self.IN2, self.ENA = 16, 27, 12   # Left motor
        self.IN3, self.IN4, self.ENB = 6, 24, 13    # Right motor

        # GPIO setup
        GPIO.setmode(GPIO.BCM)
        GPIO.setup([self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB], GPIO.OUT)

        # Force all control pins LOW at startup to prevent unwanted motion
        for p in [self.IN1, self.IN2, self.IN3, self.IN4, self.ENA, self.ENB]:
            GPIO.output(p, GPIO.LOW)

        # PWM setup on true hardware PWM pins
        self.pwm_left = GPIO.PWM(self.ENA, 100)   # 100 Hz
        self.pwm_right = GPIO.PWM(self.ENB, 100)
        self.pwm_left.start(0)
        self.pwm_right.start(0)

        # Subscribe to /cmd_vel
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        # --- Watchdog setup ---
        self.last_cmd_time = self.get_clock().now()
        self.timeout = 1.0  # seconds before automatic stop
        self.timer = self.create_timer(0.1, self.watchdog_callback)

        self.get_logger().info("Motor driver initialized and ready to receive /cmd_vel")

    def cmd_vel_callback(self, msg: Twist):
        self.last_cmd_time = self.get_clock().now()

        linear = msg.linear.x
        angular = msg.angular.z

        # Differential drive control
        left_speed = linear - angular
        right_speed = linear + angular
        self.set_motor(left_speed, right_speed)

    def watchdog_callback(self):
        elapsed = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if elapsed > self.timeout:
            self.set_motor(0.0, 0.0)
            self.get_logger().info("No /cmd_vel received for {:.1f}s — stopping motors.".format(elapsed))

    def set_motor(self, left_speed, right_speed):
        # Clamp to [-1, 1]
        left_speed = max(min(left_speed, 1.0), -1.0)
        right_speed = max(min(right_speed, 1.0), -1.0)

        # --- Left motor ---
        if left_speed >= 0:
            GPIO.output(self.IN1, GPIO.HIGH)
            GPIO.output(self.IN2, GPIO.LOW)
        else:
            GPIO.output(self.IN1, GPIO.LOW)
            GPIO.output(self.IN2, GPIO.HIGH)
        self.pwm_left.ChangeDutyCycle(abs(left_speed) * 100)

        # --- Right motor ---
        if right_speed >= 0:
            GPIO.output(self.IN3, GPIO.HIGH)
            GPIO.output(self.IN4, GPIO.LOW)
        else:
            GPIO.output(self.IN3, GPIO.LOW)
            GPIO.output(self.IN4, GPIO.HIGH)
        self.pwm_right.ChangeDutyCycle(abs(right_speed) * 100)

    def destroy_node(self):
        self.set_motor(0.0, 0.0)
        self.pwm_left.stop()
        self.pwm_right.stop()
        GPIO.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


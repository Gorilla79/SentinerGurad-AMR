import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy  # Joystick message type
import serial

class Motor:
    """Wrapper class for motor driver serial (RS232) communication"""

    SPEED_MODE = 1

    def __init__(self, port, baudrate=57600, parity=serial.PARITY_NONE):
        self.ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            parity=parity
        )
        self.mode = self.SPEED_MODE

    def send_serial(self, *data):
        msg = ""
        for d in data:
            msg += "{:02X} ".format(d)
        self.ser.write(bytearray.fromhex(msg.strip()))

    def send_serial_checksum(self, checksum=True, *data):
        if checksum:
            check = sum(data) % 256
            self.send_serial(*(list(data) + [check]))
        else:
            self.send_serial(*data)

    def run(self):
        self.ser.write(bytearray.fromhex("00 00 01 01"))

    def stop(self):
        self.ser.write(bytearray.fromhex("00 00 00 00"))

    def set_speed_mode(self, acc_time=1, dcc_time=1):
        acc = int(acc_time * 10)
        dcc = int(dcc_time * 10)
        self.send_serial_checksum(True, 0x02, 0x00, 0xC4, 0xC6)
        self.send_serial_checksum(True, 0x0A, acc, dcc)

    def set_rpm(self, rpm):
        rpm_val = int(rpm * 8192 / 3000)
        high_byte = (rpm_val >> 8) & 0xFF
        low_byte = rpm_val & 0xFF
        self.send_serial_checksum(True, 0x06, high_byte, low_byte)


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Default speed limits
        self.default_angular_speed = 0.1
        self.default_linear_speed = 0.5
        self.angular_speed = self.default_angular_speed
        self.linear_speed = self.default_linear_speed

        # Initialize motors
        self.left_motor = Motor('/dev/left_wheel_usb')
        self.right_motor = Motor('/dev/right_wheel_usb')
        self.left_motor.set_speed_mode(acc_time=1, dcc_time=1)
        self.right_motor.set_speed_mode(acc_time=1, dcc_time=1)

        # Subscribe to the /joy topic
        self.subscription = self.create_subscription(
            Joy,
            'joy',
            self.joy_callback,
            10
        )

    def joy_callback(self, msg):
        # Extract joystick values
        left_joystick_lr = msg.axes[1]  # Left joystick left-right for angular speed adjustment
        left_joystick_ud = msg.axes[0]  # Left joystick up-down for linear speed adjustment
        right_joystick_lr = msg.axes[4]  # Right joystick left-right
        right_joystick_ud = msg.axes[3]  # Right joystick up-down

        # Adjust maximum angular and linear speeds
        self.angular_speed = 0.3 + (left_joystick_lr * 0.2)  # Ranges from 0.1 to 0.5
        self.angular_speed = max(0.1, min(self.angular_speed, 0.5))

        self.linear_speed = 0.5 + (left_joystick_ud * 0.3)  # Ranges from 0.2 to 0.8
        self.linear_speed = max(0.3, min(self.linear_speed, 0.8))

        # Determine motor speeds based on right joystick input
        forward_speed = -right_joystick_ud * self.linear_speed * 0.5
        turn_speed = right_joystick_lr * self.angular_speed 

        left_speed = forward_speed - turn_speed
        right_speed = forward_speed + turn_speed

        # Debugging output
        self.get_logger().info("-----------------------------------------------------------")
        self.get_logger().info(f"Joystick Axes: {msg.axes}")
        self.get_logger().info(f"Adjusted Angular Speed: {self.angular_speed}")
        self.get_logger().info(f"Adjusted Linear Speed: {self.linear_speed}")
        self.get_logger().info(f"Left Motor Speed: {left_speed}")
        self.get_logger().info(f"Right Motor Speed: {right_speed}")
        self.get_logger().info("-----------------------------------------------------------")

        # Set motor RPMs
        self.left_motor.set_rpm(int(left_speed * 100))
        self.right_motor.set_rpm(int(right_speed * 100))

        # Run motors
        self.left_motor.run()
        self.right_motor.run()

    def destroy_node(self):
        # Safely stop and close motors on shutdown
        self.left_motor.stop()
        self.right_motor.stop()
        self.left_motor.close()
        self.right_motor.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    motor_control_node = MotorControlNode()

    try:
        rclpy.spin(motor_control_node)
    except KeyboardInterrupt:
        pass
    finally:
        motor_control_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

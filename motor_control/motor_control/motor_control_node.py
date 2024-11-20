import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

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
        self.left_motor = Motor('/dev/left_wheel_usb')
        self.right_motor = Motor('/dev/right_wheel_usb')
        self.left_motor.set_speed_mode(acc_time=1, dcc_time=1)
        self.right_motor.set_speed_mode(acc_time=1, dcc_time=1)
        
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z

        # Convert linear and angular velocity to motor speeds
        left_speed = -(linear + angular)
        right_speed = linear - angular

        self.left_motor.set_rpm(int(left_speed * 100))
        self.right_motor.set_rpm(int(right_speed * 100))

        self.left_motor.run()
        self.right_motor.run()

    def destroy_node(self):
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

    motor_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


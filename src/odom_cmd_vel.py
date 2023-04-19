#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import serial


class Serial_pub_sub(Node):
    def __init__(self):
        super().__init__('odom_cmd_vel')
        self.setup()
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.cmd_vel_callback, 10
        )
        self.timer = self.create_timer(
            0.001, self.timer_callback
        )

    def setup(self):
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.5)

    def cmd_vel_callback(self, msg):
        try:
            msg.linear.x = 100 * msg.linear.x
            msg.angular.z = 100 * msg.angular.z
            x = int(msg.linear.x)
            z = int(msg.angular.z)
            x = str(x)
            z = str(z)
            if len(x) < 2:
                x = '00' + x
            elif len(x) < 3:
                x = '0' + x
            if len(z) < 2:
                z = '00' + z
            elif len(z) < 3:
                z = '0' + z
            st = x + ',' + z + '\n'
            self.ser.write(st.encode('ascii'))
            print('Vallue sent :' + st)
        except ValueError:
            print('null errors')

    def timer_callback(self):
        self.odom_calc()

    def odom_calc(self):
        line = self.ser.readline()
        try:
            string_received = line.decode()
            words = string_received.split(',')
            words[0] = words[0].replace('{', '')
            words[0] = words[0].replace('\x00\x00\x00', '')
            words[0] = words[0].replace('\x00', '')
            if len(words) == 2:
                words[1] = words[1].replace('\n', '')
            velocity_x = (float(words[0]) + float(words[1])) * 0.0216 * 2
            angular_z = (float(words[1]) - float(words[0])) * 0.0585 * 2
            print("value recived"+velocity_x+angular_z)
            msg = Odometry()
            msg.twist.covariance[0] = 8.9e-4
            msg.twist.covariance[35] = 7.01e-3
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.twist.linear.x = velocity_x
            msg.twist.twist.angular.z = angular_z
            self.odom_publisher.publish(msg)
        except ValueError:
            print("Null error")


def main(args=None):
    rclpy.init(args=args)
    pub_sub = Serial_pub_sub()
    rclpy.spin(pub_sub)
    pub_sub.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    pass

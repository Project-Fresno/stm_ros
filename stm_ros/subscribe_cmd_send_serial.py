# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# from std_msgs.msg import String
from nav_msgs.msg import Odometry
import serial
import time


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('cmd_vel_sub')

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
        self.odom_publisher = self.create_publisher(Odometry, '/odom', 10)
        self.subscription = self.create_subscription(
            Twist, 'cmd_vel', self.listener_callback, 10
        )  # prevent unused variable warning
        # self.publish_odometry()
        self.timer = self.create_timer(
            0.01, self.timer_callback  # publishing every 0.1 second
        )

    def listener_callback(self, msg):
        # self.get_logger().info('I heard: "%s"' % msg.data)
        # send_word = ["0","0"]
        print(1)
        try:
            msg.linear.x = 100 * msg.linear.x
            msg.angular.z = 100 * msg.angular.z
            x = int(msg.linear.x)
            z = int(msg.angular.z)
            # send_word[0] = str(x)
            # send_word[1] = str(z)
            # self.get_logger().info("%s" % str(send_word))
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
            print(st)
            self.ser.write(st.encode('ascii'))
        except ValueError:
            print('fuck akash')

    def timer_callback(self):
        # while True:
        line = self.ser.readline()
        # print(line)
        # Converting Byte Strings into unicode strings
        # string_received = line.replace('0x8d','')
        if self.ser.in_waiting:
            # if (line is not None):
            string_received = line.decode()
            # Conerting Unicode String into integer
            # print(string_received)
            words = string_received.split(',')
            print(words)
            words[0] = words[0].replace('{', '')
            words[0] = words[0].replace('\x00\x00\x00', '')
            words[0] = words[0].replace('\x00', '')
            if len(words) == 2:
                words[1] = words[1].replace('\n', '')
            print(words)
            velocity_x = (float(words[0]) + float(words[1])) * 0.0216 * 2
            angular_z = (float(words[1]) - float(words[0])) * 0.0585 * 2
            # except:
            # pass
            print(velocity_x, angular_z)

            msg = Odometry()
            msg.twist.covariance[0] = 8.9e-4
            msg.twist.covariance[35] = 7.01e-3
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.twist.twist.linear.x = velocity_x
            msg.twist.twist.angular.z = angular_z
            self.odom_publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()
    # minimal_subscriber.publish_odometry()
    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
from std_msgs.msg import String
from nav_msgs.msg import Odometry
import serial
import time

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.odom_publisher = self.create_publisher(Odometry,'/odom',10)
        self.i = 0

        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)
    
        self.velocity_x = 0
        self.angualar_z = 0

    def publish_odometry(self):
        while True:
            line = self.ser.readline()
            if line:
                # Converting Byte Strings into unicode strings
                string_received = line.decode()
                # Converting Unicode String into integer
                words = string_received.split(',')
                words[1] = words[1].replace("\n", '')

                velocity_x = (float(words[0]) + float(words[1]))*1651
                angular_z = (float(words[0]) - float(words[1]))*1651
                msg = Odometry()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.twist.twist.linear.x = velocity_x
                msg.twist.twist.angular.z = angular_z
                self.odom_publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()
    minimal_publisher.publish_odometry()
    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
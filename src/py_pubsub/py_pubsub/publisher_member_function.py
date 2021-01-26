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

from std_msgs.msg import String
from sensor_msgs.msg import Temperature


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_temp_1_ = self.create_publisher(Temperature, 'temp_1', 10)
        self.publisher_temp_2_ = self.create_publisher(Temperature, 'temp_2', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        stamp = self.get_clock().now().to_msg()

        msg_temp_1 = Temperature()
        msg_temp_1.temperature = 1.0
        msg_temp_1.header.stamp = stamp
        self.publisher_temp_1_.publish(msg_temp_1)
        self.get_logger().info(f'Published temp_1 with ts {msg_temp_1.header.stamp.sec}.{msg_temp_1.header.stamp.nanosec} sec: {msg_temp_1.temperature}')

        msg_temp_2 = Temperature()
        msg_temp_2.temperature = 2.0
        msg_temp_2.header.stamp = stamp
        self.publisher_temp_2_.publish(msg_temp_2)
        self.get_logger().info(f'Published temp_2 with ts {msg_temp_2.header.stamp.sec}.{msg_temp_2.header.stamp.nanosec} sec: {msg_temp_2.temperature}')

        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

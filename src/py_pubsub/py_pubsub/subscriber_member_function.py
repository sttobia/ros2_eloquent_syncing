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
import message_filters


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        # self.subscription_temp1 = self.create_subscription(
        #     Temperature,
        #     'temp_1',
        #     self.temp_1_callback,
        #     10)
        # self.subscription_temp2 = self.create_subscription(
        #     Temperature,
        #     'temp_2',
        #     self.temp_2_callback,
        #     10)

        self.temp1_sync_sub = message_filters.Subscriber(self, Temperature, 'temp_1')
        self.temp2_sync_sub = message_filters.Subscriber(self, Temperature, 'temp_2')

        # This seems to work on the jetson but not on all other machines.
        self.synchronizer = message_filters.TimeSynchronizer([self.temp1_sync_sub, self.temp2_sync_sub], 10)
        self.synchronizer.registerCallback(self.temp_sync_callback)

    # def temp_1_callback(self, msg):
    #     self.get_logger().info(f'I heard temp_1 with ts {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec: {msg.temperature}')

    # def temp_2_callback(self, msg):
    #     self.get_logger().info(f'I heard temp_2 with ts {msg.header.stamp.sec}.{msg.header.stamp.nanosec} sec: {msg.temperature}')

    # This seems to work on the jetson but not on all other machines.
    def temp_sync_callback(self, temp1, temp2):
        self.get_logger().info(f'I heard a synced temp!')



def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

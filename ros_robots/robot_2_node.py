#!/usr/bin/env python3

import json
import os
from typing import Dict
import rclpy
from rclpy.node import Node
from std_msgs.msg import String 

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

COMMAND_TOPIC_2 = os.getenv('COMMAND_TOPIC_2', '/robot_2/commands')
FEEDBACK_TOPIC_2 = os.getenv('FEEDBACK_TOPIC_2', '/robot_2/feedback')

COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,   # never drop, retry until delivered
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10   # buffer a few commands if subscriber is slow
)

FEEDBACK_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE, # drop if subscriber is slow
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10   # buffer a few feedback messages
)

class Robot2Node(Node):
    def __init__(self):
        super().__init__('robot_2_node')
        self.get_logger().info('Robot 2 Node has been started.')
        self.publisher = self.create_publisher(String, FEEDBACK_TOPIC_2, FEEDBACK_QOS)
        self.subscriber = self.create_subscription(String, COMMAND_TOPIC_2, self.command_callback, COMMAND_QOS)

    def command_callback(self, msg: String):
        self.get_logger().info('Received command: "%s"' % msg.data)
        try:
            payload = json.loads(msg.data)
        except Exception:
            return
        
        if payload.get("event") == "task.execute":
            response = {
                "event": "task.completed",
                "deviceId": "robot_2",
                "stackId": payload.get("stackId"),
                "taskIndex": payload.get("taskIndex"),
            }
            out = String()
            out.data = json.dumps(response)
            self.publisher.publish(out)

    def publish_feedback(self, payload: Dict):
        msg = String()
        msg.data = json.dumps(payload)
        self.publisher.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)
    robot_node = Robot2Node()
    rclpy.spin(robot_node)
    robot_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
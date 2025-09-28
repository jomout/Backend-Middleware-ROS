#!/usr/bin/env python3

import os
import rclpy
from time import sleep
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from pydantic import ValidationError

from core.dto import Command, Feedback
from core.config import settings

# Device configuration
DEVICE = settings.device

# QoS settings
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

FEEDBACK_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)


class RobotNode(Node):
    """
    Simulated Robot Node that listens for commands and publishes feedback.
    """

    def __init__(self):
        node_name = f'Robot_{str(DEVICE.device_id).replace("-", "_")}'
        super().__init__(node_name)
        self.get_logger().info(f'Robot [{DEVICE.device_id}] Node has been started.')
        self.publisher = self.create_publisher(String, DEVICE.feedback_topic, FEEDBACK_QOS)
        self.subscriber = self.create_subscription(String, DEVICE.command_topic, self.command_callback, COMMAND_QOS)

        self.device_id = DEVICE.device_id

    def command_callback(self, msg: String):
        """
        Callback for processing incoming command messages.
        Simulates executing the command and publishes feedback.
        """
        self.get_logger().info(f'Received command: "{msg.data}"')

        try:
            cmd = Command.model_validate_json(msg.data)
        except ValidationError as ve:
            self.get_logger().error(f"Invalid command: {ve.errors()} | {msg.data}")
            return
        except Exception as e:
            self.get_logger().error(f"Command not JSON: {e} | {msg.data}")
            return

        # Simulate task execution
        try:
            self.get_logger().info(f'Executing task: {cmd.task}')
            
            # Simulate task processing time
            sleep(2)

            # Create feedback message
            feedback = Feedback(
                event="task.completed",
                stack_id=cmd.stack_id,
                task_index=cmd.task_index,
            )
        except Exception as e:
            self.get_logger().error(f'Error executing command: {e}')
            feedback = Feedback(
                event="task.failed",
                stack_id=cmd.stack_id,
                task_index=cmd.task_index,
                error=str(e),
            )
        finally:
            self.publish_feedback(feedback)

    def publish_feedback(self, payload: Feedback):
        """
        Publish feedback message to the feedback topic.
        """
        msg = String()
        msg.data = payload.model_dump_json()
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: "{msg.data}"')


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error occurred: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

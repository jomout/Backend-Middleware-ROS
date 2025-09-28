import json
from typing import Dict, Tuple, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import asyncio
from uuid import UUID
from app.core.logging import get_logger

from app.core.config import settings
from app.core.dto import Command, Feedback
from pydantic import ValidationError

logger = get_logger(__name__)


DEVICES = settings.devices

# QoS settings for command topics
# Reliable delivery, keep last 10 messages
COMMAND_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# QoS settings for feedback topics
# Reliable delivery, keep last 10 messages
FEEDBACK_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

class MiddlewareNode(Node):
    """
    Middleware Node for handling communication between the application and ROS 2.
    Publishes commands to robots and subscribes to their feedback.
    """
    def __init__(self):
        super().__init__('middleware_node')
        logger.info('Middleware Node has been started.')

        self.device_publishers = {}
        self.device_subscribers = {}

        for device_id, config in DEVICES.items():
            command_topic = config.command_topic
            feedback_topic = config.feedback_topic
            logger.info(f"Setting up device {device_id} with command topic {command_topic} and feedback topic {feedback_topic}")

            self.device_publishers[device_id] = self.create_publisher(String, command_topic, COMMAND_QOS)
            self.device_subscribers[device_id] = self.create_subscription(String, feedback_topic, self.feedback_callback, FEEDBACK_QOS)


        # For coordinating async waits from outside the ROS thread
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # Dictionary to track waiting futures for task completions
        # key: (stack_id, task_index) -> future that resolves on completion/failed
        self._waiters: Dict[Tuple[str, int], asyncio.Future] = {}
        

    def attach_loop(self, loop: asyncio.AbstractEventLoop):
        """Attach the asyncio loop used by the FastAPI app so callbacks can resolve futures thread-safely."""
        self._loop = loop

    def feedback_callback(self, msg: String):
        """
        Callback for handling feedback messages from robot nodes.
        Tries to resolve any waiting futures based on the feedback content.

        Args:
            msg (String): The incoming ROS message containing feedback in JSON format.
        """
        try:
            feedback = Feedback.model_validate_json(msg.data)
        except ValidationError as ve:
            logger.warning(f"Invalid feedback JSON: {ve.errors()} | {msg.data}")
            return
        except Exception:
            logger.warning(f"Feedback not JSON: {msg.data}")
            return

        event = feedback.event
        stack_id = str(feedback.stack_id)
        task_index = feedback.task_index

        logger.info(
            f'Received feedback: event={event} stack={stack_id} taskIndex={task_index} data={feedback.model_dump()}'
        )

        # If we have a waiter for this stack/task, resolve it
        key = (stack_id, task_index)
        future: Optional[asyncio.Future] = self._waiters.get(key)
        if future and not future.done():
            self._loop.call_soon_threadsafe(future.set_result, feedback)


    def publish_command(self, payload: Command, device_id: UUID):
        """
        Publish a command to the appropriate robot.

        Args:
            payload (Command): The command payload to publish. Validated against the Command DTO.
        """

        msg = String()
        msg.data = payload.model_dump_json()

        self.device_publishers[device_id].publish(msg)
        logger.info(f'Publishing to {device_id}: "{msg.data}"')


    async def execute_task_stack(self, *, device_id: UUID, stack_id: UUID, tasks: list, timeout: float = 20.0) -> bool:
        """
        Publish each task to the appropriate robot and wait for completion feedback.
        Returns True if all tasks completed, False if any failed or timed out.

        Args:
            device_id (UUID): The unique identifier for the robot device.
            stack_id (UUID): The unique identifier for the task stack.
            tasks (list): List of task dictionaries to execute.
            timeout (float): Timeout in seconds to wait for each task's feedback.
        """
        logger.info(f"[ROS] Starting execution of stack {stack_id} on Robot [{device_id}] with {len(tasks)} tasks")

        # All tasks must complete successfully
        all_ok = True
        
        # Publish all task commands and wait for their completion
        for idx, task in enumerate(tasks):
            
            # Build and validate command DTO
            try:
                cmd = Command(
                    event="task.execute",
                    stack_id=stack_id,
                    task_index=idx,
                    task=task,
                )
            except ValidationError as e:
                logger.error(f"[ROS] Invalid task/command for stack {stack_id}, task {idx}: {e.errors()}")
                all_ok = False
                break
            
            # Create waiter future
            key = (str(stack_id), idx)
            future = asyncio.get_running_loop().create_future()
            self._waiters[key] = future

            # Publish command
            self.publish_command(cmd, device_id)

            # Wait for feedback
            try:
                result: Feedback = await asyncio.wait_for(future, timeout=timeout)
            except asyncio.TimeoutError:
                logger.error(f"[ROS] Timeout waiting for feedback for stack {stack_id}, task {idx}")
                all_ok = False
                break
            finally:
                # cleanup waiter
                self._waiters.pop(key, None)

            # Check result
            if result.event != "task.completed":
                if result.error:
                    logger.error(f"[ROS] Task {idx} failed for stack {stack_id}: {result.error}")
                all_ok = False
                break

        return all_ok

def main(args=None):
    rclpy.init(args=args)
    middleware_node = MiddlewareNode()

    try:
        rclpy.spin(middleware_node)
    except Exception as e:
        logger.error(f"[ROS] Error occurred: {e}")
    finally:
        middleware_node.destroy_node()
        rclpy.shutdown()
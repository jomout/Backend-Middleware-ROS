import json
from typing import Dict, Tuple, Optional
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import asyncio
import uuid
from app.core.logging import get_logger

from app.core.config import settings

logger = get_logger(__name__)


# Load ROS topic settings from environment/config
COMMAND_TOPIC_1 = settings.command_topic_1
FEEDBACK_TOPIC_1 = settings.feedback_topic_1

COMMAND_TOPIC_2 = settings.command_topic_2
FEEDBACK_TOPIC_2 = settings.feedback_topic_2

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
        self.publisher_1 = self.create_publisher(String, COMMAND_TOPIC_1, COMMAND_QOS)
        self.subscriber_1 = self.create_subscription(String, FEEDBACK_TOPIC_1, self.feedback_callback, FEEDBACK_QOS)
        
        self.publisher_2 = self.create_publisher(String, COMMAND_TOPIC_2, COMMAND_QOS)
        self.subscriber_2 = self.create_subscription(String, FEEDBACK_TOPIC_2, self.feedback_callback, FEEDBACK_QOS)

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
            payload = json.loads(msg.data)
        except Exception:
            logger.warning(f"Feedback not JSON: {msg.data}")
            return

        # Extract relevant fields from the payload
        event = payload.get("event")
        stack_id = payload.get("stackId")
        task_index = payload.get("taskIndex")
        logger.info(f'Received feedback: event={event} stack={stack_id} taskIndex={task_index} data={payload}')

        # If we have a waiter for this stack/task, resolve it
        if stack_id is not None and isinstance(task_index, int):
            key = (stack_id, task_index)
            fut: Optional[asyncio.Future] = self._waiters.get(key)
            if fut and not fut.done():
                # Resolve based on event type
                result = {
                    "event": event,
                    "payload": payload,
                }
                if self._loop:
                    self._loop.call_soon_threadsafe(fut.set_result, result)
                else:
                    # Fallback (may not be thread-safe if no loop attached)
                    try:
                        fut.set_result(result)
                    except Exception as e:
                        logger.error(f"Failed to set future result: {e}")


    def publish_command(self, payload: Dict):
        """
        Publish a command to the appropriate robot.

        Args:
            payload (Dict): The command payload to publish, must include 'deviceId'.
        """
        msg = String()
        msg.data = json.dumps(payload)

        # Determine which publisher to use based on deviceName
        if payload.get("deviceName") == "robot_1":
            self.publisher_1.publish(msg)
            logger.info(f'Publishing: "{msg.data}"')

        elif payload.get("deviceName") == "robot_2":
            self.publisher_2.publish(msg)
            logger.info(f'Publishing: "{msg.data}"')

        else:
            logger.error(f"Unknown deviceId in payload: {payload.get('deviceId')}")


    async def execute_task_stack(self, *, device_name: str, stack_id: uuid.UUID, tasks: list, timeout: float = 20.0) -> bool:
        """
        Publish each task to the appropriate robot and wait for completion feedback.
        Returns True if all tasks completed, False if any failed or timed out.

        Args:
            device_name (str): The name of the robot device (e.g., "robot_1").
            stack_id (uuid.UUID): The unique identifier for the task stack.
            tasks (list): List of task dictionaries to execute.
            timeout (float): Timeout in seconds to wait for each task's feedback.
        """
        stack_id_str = str(stack_id)
        logger.info(f"[ROS] Starting execution of stack {stack_id_str} on {device_name} with {len(tasks)} tasks")

        # All tasks must complete successfully
        all_ok = True
        
        # Publish all task commands and wait for their completion
        for idx, task in enumerate(tasks):
            
            # Validate task type
            ttype = task.get("type")
            if ttype not in ("pick", "place"):
                logger.error(f"Unknown task type {ttype} in stack {stack_id_str}")
                all_ok = False
                break
            
            # Prepare command
            cmd = {
                "deviceName": device_name,
                "event": "task.execute",
                "stackId": stack_id_str,
                "taskIndex": idx,
                "task": task,
            }
            
            # Create waiter future
            key = (stack_id_str, idx)
            fut = asyncio.get_running_loop().create_future()
            self._waiters[key] = fut
            
            # Publish command
            self.publish_command(cmd)

            # Wait for feedback
            try:
                result = await asyncio.wait_for(fut, timeout=timeout)
            except asyncio.TimeoutError:
                logger.error(f"Timeout waiting for feedback for stack {stack_id_str}, task {idx}")
                all_ok = False
                break
            finally:
                # cleanup waiter
                self._waiters.pop(key, None)

            # Check result
            if result.get("event") != "task.completed":
                logger.error(f"Task {idx} failed for stack {stack_id_str}: {result}")
                all_ok = False
                break

        return all_ok

def main(args=None):
    rclpy.init(args=args)
    middleware_node = MiddlewareNode()
    rclpy.spin(middleware_node)
    middleware_node.destroy_node()
    rclpy.shutdown()
import asyncio
import threading
from typing import List, Optional
from uuid import UUID

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from app.ros.middleware_node import MiddlewareNode
from .logging import get_logger

logger = get_logger(__name__)


class RosBridge:
    """
    Manages a single MiddlewareNode instance and ROS2 executor in background thread, and
    provides async helpers to execute task stacks.
    """

    def __init__(self) -> None:
        """
        Initialize the ROS bridge.
        """
        self._executor = None
        self._thread = None
        self.node: Optional[MiddlewareNode] = None

    def start(self, loop: asyncio.AbstractEventLoop):
        """
        Start the ROS2 executor in a background thread.
        If an asyncio loop is provided, it will be attached to the node for thread-safe callbacks. If not, a new loop will be created.
        
        Args:
            loop: The asyncio event loop to attach to the ROS node for thread-safe operations.
        """
        if self._thread and self._thread.is_alive():
            return

        def ros_thread():
            """Thread target to run the ROS2 executor."""
            rclpy.init()
            self.node = MiddlewareNode()
            
            # Attach the asyncio loop
            self.node.attach_loop(loop)

            # Initialize executor thread
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self.node)

            try:
                self._executor.spin()
            except ExternalShutdownException as e:
                logger.info(f"ROS executor shutdown: {e}")
            except Exception as e:
                logger.error(f"ROS executor crashed: {e}")
            finally:
                if self.node is not None:
                    self._executor.remove_node(self.node)
                    self.node.destroy_node()

        # Run ROS executor in background thread
        self._thread = threading.Thread(target=ros_thread, name="ros2-executor", daemon=True)
        self._thread.start()


    async def execute_task_stack(self, device_id: UUID, stack_id: UUID, tasks: List[dict]) -> bool:
        """
        Execute a task stack on the specified device via the ROS middleware node.
        This method waits until the ROS node is initialized before delegating the task execution.

        Args:
            device_id (UUID): The UUID of the device to execute the task stack on.
            stack_id (UUID): The UUID of the task stack.
            tasks (List[dict]): A list of task dictionaries to execute.

        Returns:
            bool: True if all tasks completed successfully, False otherwise.
        """
        # Wait until node is available
        while self.node is None:
            await asyncio.sleep(0.05)
        return await self.node.execute_task_stack(device_id=device_id, stack_id=stack_id, tasks=tasks)


    def stop(self):
        """
        Stop the ROS2 executor and background thread.
        Gracefully shuts down the executor and joins the thread.
        """
        if self._executor:
            self._executor.shutdown()
        if self._thread:
            self._thread.join(timeout=2)
        if rclpy.ok():
            rclpy.shutdown()

# Singleton bridge instance
bridge = RosBridge()

import asyncio
import logging
import threading
from typing import List, Optional
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor, ExternalShutdownException

from app.ros.middleware_node import MiddlewareNode

logger = logging.getLogger(__name__)


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

    def start(self, loop: Optional[asyncio.AbstractEventLoop] = None):
        """
        Start the ROS2 executor in a background thread.
        If an asyncio loop is provided, it will be attached to the node for thread-safe callbacks. If not, a new loop will be created.
        
        Args:
            loop: The asyncio event loop to attach to the ROS node for thread-safe operations.
        """
        if self._thread and self._thread.is_alive():
            return

        app_loop = loop

        def ros_thread():
            """Thread target to run the ROS2 executor."""
            rclpy.init()
            self.node = MiddlewareNode()
            # Attach the current asyncio loop for callbacks resolution
            if app_loop:
                self.node.attach_loop(app_loop)

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


    async def execute_task_stack(self, device_name: str, stack_id: uuid.UUID, tasks: List[dict]) -> bool:
        """
        Execute a task stack on the specified device via the ROS middleware node.
        This method waits until the ROS node is initialized before delegating the task execution.

        Args:
            device_name: The name of the device to execute the task stack on.
            stack_id: The UUID of the task stack.
            tasks: A list of task dictionaries to execute.

        Returns:
            bool: True if all tasks completed successfully, False otherwise.
        """
        # Wait until node is available
        while self.node is None:
            await asyncio.sleep(0.05)
        return await self.node.execute_task_stack(device_name=device_name, stack_id=stack_id, tasks=tasks)


    def stop(self):
        """
        Stop the ROS2 executor and background thread.
        """
        if self._executor:
            self._executor.shutdown()
        if self._thread:
            self._thread.join(timeout=2)
        if rclpy.ok():
            rclpy.shutdown()


# Singleton bridge instance
bridge = RosBridge()

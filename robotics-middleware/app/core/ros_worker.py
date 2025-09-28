import logging
from sqlalchemy.orm import Session
from .models import Device, TaskStack, TaskStackStatus
from .ros_bridge import bridge
from .logging import get_logger

logger = get_logger(__name__)


async def process_task_stack(device_name: str, stack: TaskStack) -> bool:
    """
    Process a task stack by dispatching it to the ROS bridge for execution.
    Marks the stack as in_progress before dispatching, and updates status to completed or failed based
    on the ROS execution result.
    
    Args:
        device_name (str): The name of the device associated with the task stack.
        stack (TaskStack): The task stack to process.
    """

    # Execute via ROS bridge
    try:
        logger.info(f"[Robot {device_name} - {stack.device_id}] Processing stack {stack.stack_id}")

        return await bridge.execute_task_stack(
            device_name=device_name,
            stack_id=stack.stack_id,
            tasks=stack.tasks,
        )
    except Exception as e:
        logger.error(f"[Robot {device_name} - {stack.device_id}] Error in stack {stack.stack_id}: {e}")
        
        return False
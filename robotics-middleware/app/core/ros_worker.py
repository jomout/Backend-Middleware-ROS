from uuid import UUID
from .models import TaskStack
from .ros_bridge import bridge
from .logging import get_logger

logger = get_logger(__name__)


async def process_task_stack(stack: TaskStack) -> bool:
    """
    Process a task stack by dispatching it to the ROS bridge for execution.
    Marks the stack as in_progress before dispatching, and updates status to completed or failed based
    on the ROS execution result.
    
    Args:
        stack (TaskStack): The task stack to process.
    """

    # Execute via ROS bridge
    try:
        logger.info(f"[Middleware] Sending stack {stack.stack_id}")

        return await bridge.execute_task_stack(
            device_id=stack.device_id,
            stack_id=stack.stack_id,
            tasks=stack.tasks,
        )
    except Exception as e:
        logger.error(f"[Middleware] Error in stack {stack.stack_id}: {e}")
        
        return False
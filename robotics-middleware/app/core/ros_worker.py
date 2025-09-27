import logging
from sqlalchemy.orm import Session
from .models import Device, TaskStack, TaskStackStatus
from .ros_bridge import bridge

logger = logging.getLogger(__name__)


async def process_task_stack(stack: TaskStack, db: Session):
    """
    Process a task stack by dispatching it to the ROS bridge for execution.
    Marks the stack as in_progress before dispatching, and updates status to completed or failed based
    on the ROS execution result.
    
    Args:
        stack (TaskStack): The task stack to process.
        db (Session): The database session for committing status updates.  
    """
    # Mark in progress before dispatching to ROS
    stack.status = TaskStackStatus.in_progress
    db.commit()

    # Map device id to ROS robot name
    device_obj = db.query(Device).filter(Device.device_id == stack.device_id).first()
    robot_name = _device_to_robot_name(device_obj.name if device_obj else None)

    # Execute via ROS bridge
    try:
        logger.info(f"[Robot {robot_name} - {stack.device_id}] Processing stack {stack.stack_id}")
    
        success = await bridge.execute_task_stack(
            device_name=robot_name,
            stack_id=stack.stack_id,
            tasks=stack.tasks,
        )
    except Exception as e:
        logger.error(f"[Robot {robot_name} - {stack.device_id}] Error in stack {stack.stack_id}: {e}")
        success = False
    finally:
        stack.status = TaskStackStatus.completed if success else TaskStackStatus.failed
        db.commit()


def _device_to_robot_name(device_name: str) -> str:
    """Simple mapping of DB device name to ROS robot node namespace."""

    lowered = device_name.lower()
    if lowered.endswith("1") or "-1" in lowered:
        return "robot_1"
    if lowered.endswith("2") or "-2" in lowered:
        return "robot_2"
    # Default
    return "robot_1"
import asyncio
import logging
from sqlalchemy.orm import Session
from .models import TaskStack, TaskStackStatus

from .logging import get_logger

logger = get_logger(__name__)


async def process_task_stack(stack: TaskStack, db: Session):
    """
    Simulates processing a task stack without ROS by logging actions and updating status.
    Marks the stack as in_progress, simulates task execution, and updates status to completed or failed.
    
    Args:
        stack (TaskStack): The task stack to process.
        db (Session): The database session for committing status updates.  
    """
    logger.info(f"[Robot {stack.device_id}] Processing stack {stack.stack_id}")

    # Mark in progress
    stack.status = TaskStackStatus.in_progress
    db.commit()

    try:
        # Simulate task execution with delay 1 second per task
        for task in stack.tasks:
            ttype = task.get("type")
            if ttype not in ("pick", "place"):
                raise ValueError(f"Unknown task type {ttype}")
            logger.info(f"[Robot {stack.device_id}] Sending command: {ttype} -> {task}")
            await asyncio.sleep(1)

        stack.status = TaskStackStatus.completed
        logger.info(f"[Robot {stack.device_id}] Completed stack {stack.stack_id}")

    except Exception as e:
        logger.error(f"[Robot {stack.device_id}] Error in stack {stack.stack_id}: {e}")
        stack.status = TaskStackStatus.failed
    finally:
        db.commit()

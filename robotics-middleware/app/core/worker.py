# app/core/worker.py
import asyncio
import logging
from sqlalchemy.orm import Session
from .db import SessionLocal
from .models import TaskStack, TaskStackStatus

logging.basicConfig(level=logging.INFO)

async def process_task_stack(stack: TaskStack, db: Session):
    logging.info(f"[Robot {stack.device_id}] Processing stack {stack.stack_id}")
    stack.status = TaskStackStatus.in_progress
    db.commit()

    try:
        for task in stack.tasks:
            ttype = task.get("type")
            if ttype not in ("pick", "place"):
                raise ValueError(f"Unknown task type {ttype}")
            logging.info(f"[Robot {stack.device_id}] Sending command: {ttype} -> {task}")
            await asyncio.sleep(1)

        stack.status = TaskStackStatus.completed
        logging.info(f"[Robot {stack.device_id}] Completed stack {stack.stack_id}")

    except Exception as e:
        logging.error(f"[Robot {stack.device_id}] Error in stack {stack.stack_id}: {e}")
        stack.status = TaskStackStatus.failed
    finally:
        db.commit()
